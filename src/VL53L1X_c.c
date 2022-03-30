// Most of the functionality of this library is based on the VL53L1X API
// provided by ST (STSW-IMG007), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2356), and
// VL53L1X datasheet.

#include "VL53L1X_c.h"
#include "I2C_tiny.h"
// #include <Serial.h> // KD TODO

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
static const uint8_t AddressDefault = 0b0101001;

// value used in measurement timing budget calculations
// assumes PresetMode is LOWPOWER_AUTONOMOUS
//
// vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
//       (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
//     = 245 + 3 * 245 = 980
// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
//               LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
//             = 1448 + 2100 + 980 = 4528
static const uint32_t TimingGuard = 4528;

// value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS
// calculations
static const uint16_t TargetRate = 0x0A00;

// Record the current time to check an upcoming timeout against
static void VL53L1X_startTimeout(struct VL53L1X* vl) { 
  vl->timeout_start_ms = millis(); 
}

// Check if timeout is enabled (set to nonzero value) and has expired
static bool VL53L1X_checkTimeoutExpired(struct VL53L1X* vl) {
  return (vl->io_timeout > 0) && ((uint16_t)(millis() - vl->timeout_start_ms) > vl->io_timeout); 
}

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
static void VL53L1X_setupManualCalibration(struct VL53L1X* vl) {
  // "save original vhv configs"
  vl->saved_vhv_init = VL53L1X_readReg(vl, VHV_CONFIG__INIT);
  vl->saved_vhv_timeout = VL53L1X_readReg(vl, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  VL53L1X_writeReg(vl, VHV_CONFIG__INIT, vl->saved_vhv_init & 0x7F);

  // "set loop bound to tuning param"
  VL53L1X_writeReg(vl, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (vl->saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

  // "override phasecal"
  VL53L1X_writeReg(vl, PHASECAL_CONFIG__OVERRIDE, 0x01);
  VL53L1X_writeReg(vl, CAL_CONFIG__VCSEL_START, VL53L1X_readReg(vl, PHASECAL_RESULT__VCSEL_START));
}

// read measurement results into buffer
static void VL53L1X_readResults(struct VL53L1X* vl) {
  vl->last_status = I2C_write2(vl->address, RESULT__RANGE_STATUS);
  // bus->beginTransmission(address);
  // bus->write((RESULT__RANGE_STATUS >> 8) & 0xFF); // reg high byte
  // bus->write( RESULT__RANGE_STATUS       & 0xFF); // reg low byte
  // last_status = bus->endTransmission();

  I2C_read(vl->address, 17);
  // bus->requestFrom(address, (uint8_t)17);

  vl->results.range_status = I2C_receive();

  I2C_receive(); // report_status: not used

  vl->results.stream_count = I2C_receive();

  vl->results.dss_actual_effective_spads_sd0  = (uint16_t)I2C_receive() << 8; // high byte
  vl->results.dss_actual_effective_spads_sd0 |=           I2C_receive();      // low byte

  I2C_receive(); // peak_signal_count_rate_mcps_sd0: not used
  I2C_receive();

  vl->results.ambient_count_rate_mcps_sd0  = (uint16_t)I2C_receive() << 8; // high byte
  vl->results.ambient_count_rate_mcps_sd0 |=           I2C_receive();      // low byte

  I2C_receive(); // sigma_sd0: not used
  I2C_receive();

  I2C_receive(); // phase_sd0: not used
  I2C_receive();

  vl->results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)I2C_receive() << 8; // high byte
  vl->results.final_crosstalk_corrected_range_mm_sd0 |=           I2C_receive();      // low byte

  vl->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)I2C_receive() << 8; // high byte
  vl->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           I2C_receive();      // low byte
}

// Convert count rate from fixed point 9.7 format to float
#if 0
static float VL53L1X_countRateFixedToFloat(uint16_t count_rate_fixed) { 
  return (float)count_rate_fixed / (1 << 7); 
}
#endif

// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
static void VL53L1X_updateDSS(struct VL53L1X* vl) {
  uint16_t spadCount = vl->results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)vl->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      vl->results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      VL53L1X_writeReg16Bit(vl, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   VL53L1X_writeReg16Bit(vl, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
void VL53L1X_getRangingData(struct VL53L1X* vl) {
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = vl->results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  vl->ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(vl->results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      vl->ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      vl->ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      vl->ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      vl->ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      vl->ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      vl->ranging_data.range_status = SigmaFail;
      break;

    case 7: // PHASECONSISTENCY
      vl->ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      vl->ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      vl->ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (vl->results.stream_count == 0)
      {
        vl->ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        vl->ranging_data.range_status = RangeValid;
      }
      break;

    default:
      vl->ranging_data.range_status = None;
  }

  // from SetSimpleData()
  // vl->ranging_data.peak_signal_count_rate_MCPS =
    // VL53L1X_countRateFixedToFloat(vl->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  // vl->ranging_data.ambient_count_rate_MCPS =
    // VL53L1X_countRateFixedToFloat(vl->results.ambient_count_rate_mcps_sd0);
  vl->ranging_data.peak_signal_count_rate_MCPS =
    vl->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
  vl->ranging_data.ambient_count_rate_MCPS =
    vl->results.ambient_count_rate_mcps_sd0;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
#if 0
static uint32_t VL53L1X_decodeTimeout(uint16_t reg_val) {
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}
#endif

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
static uint16_t VL53L1X_encodeTimeout(uint32_t timeout_mclks) {
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
#if 0
static uint32_t VL53L1X_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us) {
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}
#endif

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
static uint32_t VL53L1X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us) {
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
static uint32_t VL53L1X_calcMacroPeriod(struct VL53L1X* vl, uint8_t vcsel_period) {
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / vl->fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

// Constructors ////////////////////////////////////////////////////////////////

static void VL53L1X_create(struct VL53L1X* vl) {
  vl->address = AddressDefault;
  vl->io_timeout = 500;
  vl->did_timeout = false;
  vl->calibrated = false;
  vl->saved_vhv_init = 0;
  vl->saved_vhv_timeout = 0;
  vl->distance_mode = Unknown;
}

// Public Methods //////////////////////////////////////////////////////////////

void VL53L1X_setAddress(struct VL53L1X* vl, uint8_t new_addr)
{
  VL53L1X_writeReg(vl, I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
  vl->address = new_addr;
}

void VL53L1X_setAddressNoSend(struct VL53L1X* vl, uint8_t new_addr)
{
  vl->address = new_addr;
}

uint8_t VL53L1X_getAddress(struct VL53L1X* vl)
{ 
  return vl->address; 
}

// Initialize sensor using settings taken mostly from VL53L1_DataInit() and
// VL53L1_StaticInit().
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L1X_init(struct VL53L1X* vl)
{
  VL53L1X_create(vl);

  // check model ID and module type registers (values specified in datasheet)
  if (VL53L1X_readReg16Bit(vl, IDENTIFICATION__MODEL_ID) != 0xEACC) { return false; }

  // VL53L1_software_reset() begin

  VL53L1X_writeReg(vl, SOFT_RESET, 0x00);
  delayMicroseconds(100);
  VL53L1X_writeReg(vl, SOFT_RESET, 0x01);

  // give it some time to boot; otherwise the sensor NACKs during the readReg()
  // call below and the Arduino 101 doesn't seem to handle that well
  delay(1);

  // VL53L1_poll_for_boot_completion() begin

  VL53L1X_startTimeout(vl);

  // check last_status in case we still get a NACK to try to deal with it correctly
  while ((VL53L1X_readReg(vl, FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || vl->last_status != 0)
  {
    if (VL53L1X_checkTimeoutExpired(vl))
    {
      vl->did_timeout = true;
      return false;
    }
  }
  // VL53L1_poll_for_boot_completion() end

  // VL53L1_software_reset() end

  // VL53L1_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  VL53L1X_writeReg(vl, PAD_I2C_HV__EXTSUP_CONFIG,
    VL53L1X_readReg(vl, PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);

  // store oscillator info for later use
  vl->fast_osc_frequency = VL53L1X_readReg16Bit(vl, OSC_MEASURED__FAST_OSC__FREQUENCY);
  vl->osc_calibrate_val = VL53L1X_readReg16Bit(vl, RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end

  // VL53L1_StaticInit() begin

  // Note that the API does not actually apply the configuration settings below
  // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
  // register contents in memory and doesn't actually write them until a
  // measurement is started. Writing the configuration here means we don't have
  // to keep it all in memory and avoids a lot of redundant writes later.

  // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
  // VL53L1_set_preset_mode() begin

  // VL53L1_preset_mode_standard_ranging() begin

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // (API uses these in VL53L1_init_tuning_parm_storage_struct())

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
  // that? (seems like it would disable 2V8 mode)
  VL53L1X_writeReg16Bit(vl, DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
  VL53L1X_writeReg(vl, GPIO__TIO_HV_STATUS, 0x02);
  VL53L1X_writeReg(vl, SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
  VL53L1X_writeReg(vl, SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
  VL53L1X_writeReg(vl, ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  VL53L1X_writeReg(vl, ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  VL53L1X_writeReg(vl, ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
  VL53L1X_writeReg(vl, ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

  // general config
  VL53L1X_writeReg16Bit(vl, SYSTEM__THRESH_RATE_HIGH, 0x0000);
  VL53L1X_writeReg16Bit(vl, SYSTEM__THRESH_RATE_LOW, 0x0000);
  VL53L1X_writeReg(vl, DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  VL53L1X_writeReg16Bit(vl, RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
  VL53L1X_writeReg16Bit(vl, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config

  VL53L1X_writeReg(vl, SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  VL53L1X_writeReg(vl, SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  VL53L1X_writeReg(vl, SD_CONFIG__QUANTIFIER, 2); // tuning parm default

  // VL53L1_preset_mode_standard_ranging() end

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
  // and things don't seem to work if we don't set GPH back to 0 (which the API
  // does here).
  VL53L1X_writeReg(vl, SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  VL53L1X_writeReg(vl, SYSTEM__SEED_CONFIG, 1); // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  VL53L1X_writeReg(vl, SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
  VL53L1X_writeReg16Bit(vl, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8); // overflow?
  VL53L1X_writeReg(vl, DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

  // VL53L1_set_preset_mode() end

  // default to long range, 50 ms timing budget
  // note that this is different than what the API defaults to
  VL53L1X_setDistanceMode(vl);
  VL53L1X_setMeasurementTimingBudget(vl);

  // VL53L1_StaticInit() end

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  VL53L1X_writeReg16Bit(vl, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
    VL53L1X_readReg16Bit(vl, MM_CONFIG__OUTER_OFFSET_MM) * 4);

  return true;
}

// Write an 8-bit register
void VL53L1X_writeReg(struct VL53L1X* vl, uint16_t reg, uint8_t value)
{
  vl->last_status = I2C_write2_reg(vl->address, reg, value);
  // vl->last_status = I2C_write2_sn(vl->address, reg, (char*) &value, sizeof(value));
  // bus->beginTransmission(address);
  // bus->write((reg >> 8) & 0xFF); // reg high byte
  // bus->write( reg       & 0xFF); // reg low byte
  // bus->write(value);
  // last_status = bus->endTransmission();
}

// Write a 16-bit register
void VL53L1X_writeReg16Bit(struct VL53L1X* vl, uint16_t reg, uint16_t value)
{
  vl->last_status = I2C_write2_sn(vl->address, reg, (char*) &value, sizeof(value));
  // bus->beginTransmission(address);
  // bus->write((reg >> 8) & 0xFF); // reg high byte
  // bus->write( reg       & 0xFF); // reg low byte
  // bus->write((value >> 8) & 0xFF); // value high byte
  // bus->write( value       & 0xFF); // value low byte
  // last_status = bus->endTransmission();
}

// Write a 32-bit register
void VL53L1X_writeReg32Bit(struct VL53L1X* vl, uint16_t reg, uint32_t value)
{
  vl->last_status = I2C_write2_sn(vl->address, reg, (char*) &value, sizeof(value));
  // bus->beginTransmission(address);
  // bus->write((reg >> 8) & 0xFF); // reg high byte
  // bus->write( reg       & 0xFF); // reg low byte
  // bus->write((value >> 24) & 0xFF); // value highest byte
  // bus->write((value >> 16) & 0xFF);
  // bus->write((value >>  8) & 0xFF);
  // bus->write( value        & 0xFF); // value lowest byte
  // last_status = bus->endTransmission();
}

// Read an 8-bit register
uint8_t VL53L1X_readReg(struct VL53L1X* vl, enum VL53L1X_regAddr reg)
{
  uint8_t value;

  vl->last_status = I2C_write2(vl->address, reg);
  I2C_read(vl->address, 1);
  value = I2C_receive();

  return value;
}

// Read a 16-bit register
uint16_t VL53L1X_readReg16Bit(struct VL53L1X* vl, uint16_t reg)
{
  uint16_t value;

  vl->last_status = I2C_write2(vl->address, reg);
  I2C_read(vl->address, 2);
  value  = (uint16_t)I2C_receive() << 8; // value high byte
  value |=           I2C_receive();      // value low byte

  return value;
}

// Read a 32-bit register
uint32_t VL53L1X_readReg32Bit(struct VL53L1X* vl, uint16_t reg)
{
  uint32_t value;

  vl->last_status = I2C_write2(vl->address, reg);
  I2C_read(vl->address, 4);
  value  = (uint32_t)I2C_receive() << 24; // value highest byte
  value |= (uint32_t)I2C_receive() << 16;
  value |= (uint16_t)I2C_receive() <<  8;
  value |=           I2C_receive();       // value lowest byte

  return value;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool VL53L1X_setDistanceMode(struct VL53L1X* vl)
{
  // save existing timing budget
  // uint32_t budget_us = VL53L1X_getMeasurementTimingBudget(vl);

  // from VL53L1_preset_mode_standard_ranging_short_range()

  // timing config
  VL53L1X_writeReg(vl, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
  VL53L1X_writeReg(vl, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
  VL53L1X_writeReg(vl, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

  // dynamic config
  VL53L1X_writeReg(vl, SD_CONFIG__WOI_SD0, 0x07);
  VL53L1X_writeReg(vl, SD_CONFIG__WOI_SD1, 0x05);
  VL53L1X_writeReg(vl, SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
  VL53L1X_writeReg(vl, SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

  // reapply timing budget
  VL53L1X_setMeasurementTimingBudget(vl);

  // save mode so it can be returned by getDistanceMode()
  // vl->distance_mode = mode;

  return true;
}

#if 0
enum VL53L1X_DistanceMode VL53L1X_getDistanceMode(struct VL53L1X* vl) {
   return vl->distance_mode; 
}
#endif

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool VL53L1X_setMeasurementTimingBudget(struct VL53L1X* vl)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  // if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = 500000 - TimingGuard;
  // if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = VL53L1X_calcMacroPeriod(vl, VL53L1X_readReg(vl, RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = VL53L1X_timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  VL53L1X_writeReg(vl, PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  VL53L1X_writeReg16Bit(vl, MM_CONFIG__TIMEOUT_MACROP_A, VL53L1X_encodeTimeout(
    VL53L1X_timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  VL53L1X_writeReg16Bit(vl, RANGE_CONFIG__TIMEOUT_MACROP_A, VL53L1X_encodeTimeout(
    VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = VL53L1X_calcMacroPeriod(vl, VL53L1X_readReg(vl, RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  VL53L1X_writeReg16Bit(vl, MM_CONFIG__TIMEOUT_MACROP_B, VL53L1X_encodeTimeout(
    VL53L1X_timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  VL53L1X_writeReg16Bit(vl, RANGE_CONFIG__TIMEOUT_MACROP_B, VL53L1X_encodeTimeout(
    VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return true;
}

#if 0
// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t VL53L1X_getMeasurementTimingBudget(struct VL53L1X* vl)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = VL53L1X_calcMacroPeriod(vl, VL53L1X_readReg(vl, RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

  uint32_t range_config_timeout_us = VL53L1X_timeoutMclksToMicroseconds(VL53L1X_decodeTimeout(
    VL53L1X_readReg16Bit(vl, RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}
#endif

/*
// Set the width and height of the region of interest
// based on VL53L1X_SetROI() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully.
void VL53L1X_setROISize(struct VL53L1X* vl, uint8_t width, uint8_t height)
{
  if ( width > 16) {  width = 16; }
  if (height > 16) { height = 16; }

  // Force ROI to be centered if width or height > 10, matching what the ULD API
  // does. (This can probably be overridden by calling setROICenter()
  // afterwards.)
  if (width > 10 || height > 10)
  {
    VL53L1X_writeReg(vl, ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
  }

  VL53L1X_writeReg(vl, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
           (height - 1) << 4 | (width - 1));
}

// Get the width and height of the region of interest (ROI)
// based on VL53L1X_GetROI_XY() from STSW-IMG009 Ultra Lite Driver
void VL53L1X_getROISize(struct VL53L1X* vl, uint8_t * width, uint8_t * height)
{
  uint8_t reg_val = VL53L1X_readReg(vl, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
  *width = (reg_val & 0xF) + 1;
  *height = (reg_val >> 4) + 1;
}

// Set the center SPAD of the region of interest (ROI)
// based on VL53L1X_SetROICenter() from STSW-IMG009 Ultra Lite Driver
//
// ST user manual UM2555 explains ROI selection in detail, so we recommend
// reading that document carefully. Here is a table of SPAD locations from
// UM2555 (199 is the default/center):
//
// 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
// 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
// 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
// 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
// 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
// 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
// 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
// 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255
//
// 127,119,111,103, 95, 87, 79, 71,   63, 55, 47, 39, 31, 23, 15,  7
// 126,118,110,102, 94, 86, 78, 70,   62, 54, 46, 38, 30, 22, 14,  6
// 125,117,109,101, 93, 85, 77, 69,   61, 53, 45, 37, 29, 21, 13,  5
// 124,116,108,100, 92, 84, 76, 68,   60, 52, 44, 36, 28, 20, 12,  4
// 123,115,107, 99, 91, 83, 75, 67,   59, 51, 43, 35, 27, 19, 11,  3
// 122,114,106, 98, 90, 82, 74, 66,   58, 50, 42, 34, 26, 18, 10,  2
// 121,113,105, 97, 89, 81, 73, 65,   57, 49, 41, 33, 25, 17,  9,  1
// 120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0 <- Pin 1
//
// This table is oriented as if looking into the front of the sensor (or top of
// the chip). SPAD 0 is closest to pin 1 of the VL53L1X, which is the corner
// closest to the VDD pin on the Pololu VL53L1X carrier board:
//
//   +--------------+
//   |             O| GPIO1
//   |              |
//   |             O|
//   | 128    248   |
//   |+----------+ O|
//   ||+--+  +--+|  |
//   |||  |  |  || O|
//   ||+--+  +--+|  |
//   |+----------+ O|
//   | 120      0   |
//   |             O|
//   |              |
//   |             O| VDD
//   +--------------+
//
// However, note that the lens inside the VL53L1X inverts the image it sees
// (like the way a camera works). So for example, to shift the sensor's FOV to
// sense objects toward the upper left, you should pick a center SPAD in the
// lower right.
void VL53L1X_setROICenter(struct VL53L1X* vl, uint8_t spadNumber)
{
  VL53L1X_writeReg(vl, ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNumber);
}

// Get the center SPAD of the region of interest
// based on VL53L1X_GetROICenter() from STSW-IMG009 Ultra Lite Driver
uint8_t VL53L1X_getROICenter(struct VL53L1X* vl)
{
  return VL53L1X_readReg(vl, ROI_CONFIG__USER_ROI_CENTRE_SPAD);
}
*/

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void VL53L1X_startContinuous(struct VL53L1X* vl, uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  VL53L1X_writeReg32Bit(vl, SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * vl->osc_calibrate_val);

  VL53L1X_writeReg(vl, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  VL53L1X_writeReg(vl, SYSTEM__MODE_START, 0x40); // mode_range__timed
}

/*
// Stop continuous measurements
// based on VL53L1_stop_range()
void VL53L1X_stopContinuous(struct VL53L1X* vl)
{
  VL53L1X_writeReg(vl, SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  vl->calibrated = false;

  // "restore vhv configs"
  if (vl->saved_vhv_init != 0)
  {
    VL53L1X_writeReg(vl, VHV_CONFIG__INIT, vl->saved_vhv_init);
  }
  if (vl->saved_vhv_timeout != 0)
  {
     VL53L1X_writeReg(vl, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, vl->saved_vhv_timeout);
  }

  // "remove phasecal override"
  VL53L1X_writeReg(vl, PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}
*/

// Returns a range reading in millimeters when continuous mode is active. If
// blocking is true (the default), this function waits for a new measurement to
// be available. If blocking is false, it will try to return data immediately.
// (readSingle() also calls this function after starting a single-shot range
// measurement)
uint16_t VL53L1X_read(struct VL53L1X* vl)
{
  VL53L1X_readResults(vl);

  if (!vl->calibrated)
  {
    VL53L1X_setupManualCalibration(vl);
    vl->calibrated = true;
  }

  VL53L1X_updateDSS(vl);

  VL53L1X_getRangingData(vl);

  VL53L1X_writeReg(vl, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

  return vl->ranging_data.range_mm;
}

/*
// Starts a single-shot range measurement. If blocking is true (the default),
// this function waits for the measurement to finish and returns the reading.
// Otherwise, it returns 0 immediately.
uint16_t VL53L1X_readSingle(struct VL53L1X* vl, bool blocking)
{
  VL53L1X_writeReg(vl, SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  VL53L1X_writeReg(vl, SYSTEM__MODE_START, 0x10); // mode_range__single_shot

  if (blocking)
  {
    return VL53L1X_read(vl, true);
  }
  else
  {
    return 0;
  }
}

uint16_t VL53L1X_readRangeSingleMillimeters(struct VL53L1X* vl, bool blocking) { 
  return VL53L1X_readSingle(vl, blocking); 
}
*/

#if 0
bool VL53L1X_dataReady(struct VL53L1X* vl) { 
  return (VL53L1X_readReg(vl, GPIO__TIO_HV_STATUS) & 0x01) == 0; 
}
#endif

uint16_t VL53L1X_readRangeContinuousMillimeters(struct VL53L1X* vl) { 
  return VL53L1X_read(vl); 
}

/*
// convert a RangeStatus to a readable string
// Note that on an AVR, these strings are stored in RAM (dynamic memory), which
// makes working with them easier but uses up 200+ bytes of RAM (many AVR-based
// Arduinos only have about 2000 bytes of RAM). You can avoid this memory usage
// if you do not call this function in your sketch.
const char * VL53L1X_rangeStatusToString(enum VL53L1X_RangeStatus status)
{
  switch (status)
  {
    case RangeValid:
      return "range valid";

    case SigmaFail:
      return "sigma fail";

    case SignalFail:
      return "signal fail";

    case RangeValidMinRangeClipped:
      return "range valid, min range clipped";

    case OutOfBoundsFail:
      return "out of bounds fail";

    case HardwareFail:
      return "hardware fail";

    case RangeValidNoWrapCheckFail:
      return "range valid, no wrap check fail";

    case WrapTargetFail:
      return "wrap target fail";

    case XtalkSignalFail:
      return "xtalk signal fail";

    case SynchronizationInt:
      return "synchronization int";

    case MinRangeFail:
      return "min range fail";

    case None:
      return "no update";

    default:
      return "unknown status";
  }
}
*/

/*
void VL53L1X_setTimeout(struct VL53L1X* vl, uint16_t timeout) 
{ 
  vl->io_timeout = timeout; 
}

uint16_t VL53L1X_getTimeout(struct VL53L1X* vl) 
{ 
  return vl->io_timeout; 
}
*/

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L1X_timeoutOccurred(struct VL53L1X* vl)
{
  bool tmp = vl->did_timeout;
  vl->did_timeout = false;
  return tmp;
}
