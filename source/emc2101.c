// SPDX-License-Identifier: GPL-2.0-or-later

#include "fsl_lpi2c.h"
#include "emc2101.h"
#include "fsl_debug_console.h"

static status_t emc2101_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    lpi2c_master_transfer_t read_transfer;

    read_transfer.slaveAddress = device_addr;
    read_transfer.direction = kLPI2C_Read;
    read_transfer.subaddress = reg_addr;
    read_transfer.subaddressSize = 1;
    read_transfer.data = reg_data;
    read_transfer.dataSize = len;
    read_transfer.flags = kLPI2C_TransferDefaultFlag;

    status_t status = LPI2C_MasterTransferBlocking(LPI2C2, &read_transfer);



    return status;
}


/*!
 * I2C write function map to COINES platform
 */
static status_t emc2101_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    lpi2c_master_transfer_t write_transfer;

	write_transfer.slaveAddress = device_addr;
	write_transfer.direction = kLPI2C_Write;
	write_transfer.subaddress = reg_addr;
	write_transfer.subaddressSize = 1;
	write_transfer.data = reg_data;
	write_transfer.dataSize = len;
	write_transfer.flags = kLPI2C_TransferDefaultFlag;

	status_t status = LPI2C_MasterTransferBlocking(LPI2C2, &write_transfer);



	return status;
}

bool emc2101_init(void)
{
	uint8_t id_reg;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_WHOAMI, &id_reg, 1, &device_addr);

	if(status != kStatus_Success)
	{
		PRINTF("Failed to read device id \r\n");
		return false;
	}
	else
	{
		PRINTF("The device id is 0x%X\r\n", id_reg);
	}

	emc2101_enable_tach_input(true);
	emc2101_invert_fan_speed(false);
	emc2101_set_PWM_freq(0x1F);
	emc2101_config_PWM_clock(true, false);
	enable_emc2101_DACOut(false);
	emc2101_enable_lut(false);
	emc2101_set_duty_cycle(100);
	emc2101_set_data_rate(EMC2101_RATE_32_HZ);

	return true;
}

/**
 * @brief Set the fan speed.
 *
 *
 * @param pwm_duty_cycle The  duty cycle percentage as an integer
 * The speed is  given as the fan's PWM duty cycle and **roughly** approximates
 * the percentage of the fan's maximum speed
 * @return true: success false: failure
 */
bool emc2101_set_duty_cycle(uint8_t pwm_duty_cycle) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	uint8_t calculated_pwm = (uint16_t) (pwm_duty_cycle * 63) / 100;

	bool lut_enabled = get_emc2101_lut_enabled_status();
	emc2101_enable_lut(false);
	status_t status = emc2101_i2c_write(EMC2101_REG_FAN_SETTING, &calculated_pwm, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Unable to write to EMC2101 REG FAN SETTING\n");
		return false;
	}

	return emc2101_enable_lut(lut_enabled);

}

/**
 * @brief Get the fan speed setting used while the LUT is being updated and is
 * unavailable or not in use. The speed is  given as the fan's PWM duty cycle
 * represented as a float percentage. The value **roughly** approximates the
 * percentage of the fan's maximum speed"""
 *
 * @return float The current manually set fan duty cycle
 */
bool emc2101_get_duty_cycle(uint8_t* duty_cycle) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_read(EMC2101_REG_FAN_SETTING, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write pwm freq \r\n");
		return false;
	}

	uint8_t raw_value = reg_value & MAX_LUT_SPEED;

	*duty_cycle = (uint8_t)((raw_value / (float)MAX_LUT_SPEED) * 100);

	return true;
}
/**
 * @brief Enable using the TACH/ALERT pin as an input to read the fan speed
 * signal from a 4-pin fan
 *
 * @param tach_enable true: to enable tach signal input, false to disable and
 * use the tach pin as interrupt & status output
 * @return true: sucess false: failure
 */
bool emc2101_enable_tach_input(bool tach_enable) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_REG_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read tach input \r\n");
		return false;
	}

	if(tach_enable)
		reg_value |= (1 << 2);
	else
		reg_value &= ~(1 << 2);

	status = emc2101_i2c_write(EMC2101_REG_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write tach input \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Set the cotroller to interperate fan speed settings opposite of the
 * normal behavior
 *
 * @param invert_speed If true, fan duty cycle / DAC value settings will work
 * backwards; Setting the highest value (100) will set the fan to it's lowest
 * PWM value, and setting the fan to the lowest value (0) will set the fan
 * output to it's highest setting.
 * @return true:sucess false:failure
 */
bool emc2101_invert_fan_speed(bool invert_speed) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}

	if(invert_speed)
		reg_value |= (1 << 4);
	else
		reg_value &= ~(1 << 4);

	status = emc2101_i2c_write(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write fan config \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Configure the PWM clock by selecting the clock source and overflow bit
 *
 * @param clksel The clock select true: Use a 1.4kHz base PWM clock
 * false: Use the default 360kHz PWM clock
 *
 * @param clkovr Clock override
 * When true, override the base clock selected by `clksel` and use the frequency
 * divisor to set the PWM frequency
 *
 * @return true:success false:failure
 */
bool emc2101_config_PWM_clock(bool clksel, bool clkovr) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}

	if(clksel)
		reg_value |= (1 << 3);
	else
		reg_value &= ~(1 << 3);

	if(clkovr)
		reg_value |= (1 << 2);
	else
		reg_value &= ~(1 << 2);

	status = emc2101_i2c_write(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write fan config \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Configure the fan's spinup behavior when transitioning from
 * off/minimal speed to a higher speed (except on power up)
 *
 * @param spinup_drive The duty cycle to drive the fan with during spin up
 * **defaults to 100%**
 *
 * @param spinup_time The amount of time to keep the fan at the given drive
 * setting. **Defaults to 3.2 seconds**
 *
 * @return true:success false: failure
 */
bool emc2101_config_fan_spinup_time(uint8_t spinup_drive,
                                       uint8_t spinup_time) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_SPINUP, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}

	// mask for
	uint8_t mask = 0x0F;
	reg_value &= ~mask;

	reg_value |= (spinup_drive & 0x03) << 3;
	reg_value |= (spinup_time & 0x07);

	status = emc2101_i2c_write(EMC2101_FAN_SPINUP, &reg_value, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Failed to write config value\n");
		return false;
	}

	return true;
}

/**
 * @brief Configure the fan's spinup behavior when transitioning from
 * off/minimal speed to a higher speed (except on power up)
 *
 * @param tach_spinup If true, drive the fan at 100% until the speed is above
 * the speed set with `setFanMinRPM`. If previously set, `spinup_drive` and
 * `spinup_time` are ignored
 *
 * @return true:success false: failure
 */
bool emc2101_config_fan_spinup(bool tach_spinup) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_SPINUP, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}
	// Invert a bit
	if(tach_spinup)
		reg_value |= (1 << 5);
	else
		reg_value &= ~(1 << 5);


	status = emc2101_i2c_write(EMC2101_FAN_SPINUP, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write fan config \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Read the final PWM frequency and "effective resolution" of the PWM
 * driver. No effect when DAC output is enabled
 *
 * See the datasheet for additional information:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/2101.pdf
 *
 * @return uint8_t The PWM freq register setting
 */
 bool emc2101_get_PWM_freq(uint8_t* pwm_freq) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_read(EMC2101_PWM_FREQ, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write pwm freq \r\n");
		return false;
	}

	*pwm_freq = reg_value;

  return true;
}

/**
 * @brief Set the final PWM frequency and "effective resolution" of the PWM
 * driver. No effect when DAC output is enabled
 *
 * See the datasheet for additional information:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/2101.pdf
 *
 * @param pwm_freq The new PWM frequency setting
 *
 * @return bool true:success false:failure
 */
bool emc2101_set_PWM_freq(uint8_t pwm_freq) {

	uint8_t reg_value = pwm_freq;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_write(EMC2101_PWM_FREQ, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write pwm freq \r\n");
		return false;
	}


  return true;
}

/**
 * @brief Get the alternate PWM frequency digide value to use instead of the
 * clock selection bit when the clock select override is set
 *
 * @return uint8_t The alternate divisor setting
 */
bool emc2101_get_PWM_divisor(uint8_t* pwm_divisor) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_PWM_DIV, pwm_divisor, 1, &device_addr);

	if(kStatus_Success != status){
		PRINTF("Failed set PWM division\n");
		return false;
	}

	return true;
}

/**
 * @brief Get the alternate PWM frequency digide value to use instead of the
 * clock selection bit when the clock select override is set
 * @param pwm_divisor The alternate divisor setting
 * @return true:success false: failure
 */
bool emc2101_set_PWM_divisor(uint8_t pwm_divisor) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_write(EMC2101_PWM_DIV, &pwm_divisor, 1, &device_addr);

	if(kStatus_Success != status){
		PRINTF("Failed set PWM division\n");
		return false;
	}

	return true;
}

/**
 * @brief Enable or disable outputting the fan control signal as a DC voltage
 * instead of the default PWM output
 *
 * @param enable_dac_out true will enable DAC output, false disables DAC output
 * @return true:success false: failure
 */
bool enable_emc2101_DACOut(bool enable_dac_out) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_REG_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read tach input \r\n");
		return false;
	}

	if(enable_dac_out)
		reg_value |= (1 << 4);
	else
		reg_value &= ~(1 << 4);

	status = emc2101_i2c_write(EMC2101_REG_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write tach input \r\n");
		return false;
	}
  return true;

}

/**
 * @brief Get the current DAC output enable setting
 *
 * @return true: DAC output enabled
 * @return false DAC output disabled
 */
bool get_emc2101_dac_status(void){

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_REG_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read tach input \r\n");
	}

	return reg_value & (1 << 4);
}

bool get_emc2101_lut_enabled_status(void){

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
	}

	return reg_value & (1 << 5);
}

/**
 * @brief Enable or disable the temperature-to-fan speed Look Up Table (LUT)
 *
 * @param enable_lut True to enable the LUT, setting the fan speed depending on
 * the configured temp to speed mapping. False disables the LUT, defaulting to
 * the speed setting from `setDutyCycle`
 * @return true:success false: failure
 */
bool emc2101_enable_lut(bool enable_lut) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}
	// Invert a bit
	if(enable_lut)
		reg_value &= ~(1 << 5);
	else
		reg_value |= (1 << 5);


	status = emc2101_i2c_write(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write fan config \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Force the LUT to use the temperature set by `setForcedTemperature`.
 *
 * This can be used to use the LUT to set the fan speed based on a different
 * source than the external temperature diode. This can also be used to verify
 * LUT configuration
 *
 * @param enable_forced True to force the LUT to use the forced temperature
 * @return true: success false: failure
 */
bool emc2101_enable_forced_temperature(bool enable_forced) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read tach input \r\n");
		return false;
	}

	if(enable_forced)
		reg_value |= (1 << 6);
	else
		reg_value &= ~(1 << 6);

	status = emc2101_i2c_write(EMC2101_FAN_CONFIG, &reg_value, 1, &device_addr);
	if(status != kStatus_Success){
		PRINTF("Failed to write tach input \r\n");
		return false;
	}
  return true;
}

/**
 * @brief Set the alternate temperature to use to look up a fan setting in the
 * look up table
 *
 * @param forced_temperature The alternative temperature reading used for LUT
 * lookups.
 * @return true: success false: falure
 */
bool emc2101_set_forced_temperature(int8_t forced_temperature) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_write(EMC2101_TEMP_FORCE, &forced_temperature, 1, &device_addr);

	if(kStatus_Success != status){
		PRINTF("Failed to set forced temperature\n");
		return false;
	}
	return true;
}

/**
 * @brief Get the alternate temperature to use to look up a fan setting in the
 * look up table
 *
 * @return int8_t The alternative temperature reading used for LUT
 * lookups
 */
bool emc2101_get_forced_temperature(int8_t* forced_temp_reg) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_TEMP_FORCE, forced_temp_reg, 1, &device_addr);

	if(status != kStatus_Success){
		PRINTF("Failed to read fan config \r\n");
		return false;
	}

	return true;
}

/**
 * @brief Gets the current rate at which pressure and temperature measurements
 * are taken
 *
 * @return emc2101_rate_t The current data rate
 */
 bool emc2101_get_data_rate(emc2101_rate_t* rate) {

	uint8_t reg_value;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_read(EMC2101_REG_DATA_RATE, &reg_value, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to retrieve data rate\n");
		return false;
	}

	*rate = reg_value & 0x0F;

	return true;
}

/**
 * @brief Sets the rate at which pressure and temperature measurements
 *
 * @param new_data_rate The data rate to set. Must be a `emc2101_rate_t`
 * @return bool true:success false:failure
 */
bool emc2101_set_data_rate(emc2101_rate_t new_data_rate) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_write(EMC2101_REG_DATA_RATE, &new_data_rate, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to retrieve data rate\n");
		return false;
	}

	return true;
}

/**
 * @brief Read the external temperature diode
 *
 * @return float the current temperature in degrees C
 */
bool emc2101_get_external_temperature(float* ext_temperature) {
  // chip doesn't like doing multi-byte reads so we'll get each byte separately
  // and join
  uint8_t buffer[2];
  uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

  status_t status = emc2101_i2c_read(EMC2101_EXTERNAL_TEMP_LSB, buffer + 1, 1, &device_addr);
  if(kStatus_Success != status){
	PRINTF("Unable to read internal temperature\r\n");
	return false;
  }

  status = emc2101_i2c_read(EMC2101_EXTERNAL_TEMP_MSB, buffer, 1, &device_addr);
  if(kStatus_Success != status){
	PRINTF("Unable to read internal temperature\r\n");
	return false;
  }


  int16_t raw_ext = buffer[0] << 8;
  raw_ext |= buffer[1];

  raw_ext >>= 5;
  *ext_temperature = raw_ext * _TEMP_LSB;
  return true;
}

/**
 * @brief Read the internal temperature sensor
 *
 * @return int8_t the current temperature in degrees celcius
 */
bool emc2101_get_internalTemperature(int8_t* temperature_value) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_INTERNAL_TEMP, temperature_value, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

	return true;
}

/**
 * @brief Get the mimnum RPM setting for the attached fan
 *
 * @return uint16_t the current minimum RPM setting
 */
bool emc2101_get_fan_min_rpm(uint16_t* min_rpm){

  uint8_t buffer[2];
  uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

  status_t status = emc2101_i2c_read(EMC2101_TACH_LIMIT_LSB, buffer + 1, 1, &device_addr);
  if(kStatus_Success != status){
	PRINTF("Unable to read internal temperature\r\n");
	return false;
  }

  status = emc2101_i2c_read(EMC2101_TACH_LIMIT_MSB, buffer, 1, &device_addr);
  if(kStatus_Success != status){
	PRINTF("Unable to read internal temperature\r\n");
	return false;
  }

  uint16_t raw_limit = buffer[0] << 8;
  raw_limit |= buffer[1];
  if (raw_limit == 0xFFFF){
	  return false;
  }

  *min_rpm = EMC2101_FAN_RPM_NUMERATOR / raw_limit;

  return true;
}

/**
 * @brief Set the minimum speed of the attached fan
 *
 * Used to determine the fan state
 *
 * @param min_rpm The minimum speed of the fan. Any setting below this will
 * return 0 and mark the fan as non-operational
 * @return true: success false: failure
 */
bool emc2101_set_fan_min_rpm(uint16_t min_rpm){

	// speed is given in RPM, convert to raw value (MSB+LSB):
	uint16_t raw_value = EMC2101_FAN_RPM_NUMERATOR / min_rpm;
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	uint8_t lsb = (raw_value & 0xFF);
	uint8_t msb = ((raw_value >> 8) & 0xFF);

	status_t status = emc2101_i2c_write(EMC2101_TACH_LIMIT_LSB, &lsb, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

	status = emc2101_i2c_write(EMC2101_TACH_LIMIT_MSB, &msb, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

    return true;
}

/**
 * @brief Read the current fan speed in RPM.
 *
 * @return uint16_t The current fan speed, 0 if no tachometer input
 */
bool emc2101_get_fan_rpm(uint16_t* fan_rpm) {
	uint8_t buffer[2];
	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;

	status_t status = emc2101_i2c_read(EMC2101_TACH_LSB, buffer + 1, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

    status = emc2101_i2c_read(EMC2101_TACH_MSB, buffer, 1, &device_addr);
    if(kStatus_Success != status){
    	PRINTF("Unable to read internal temperature\r\n");
    	return false;
    }

    uint16_t raw_limit = buffer[0] << 8;
    raw_limit |= buffer[1];
    if (raw_limit == 0xFFFF){
  	  return false;
    }

    *fan_rpm = EMC2101_FAN_RPM_NUMERATOR / raw_limit;

    return true;
}

/**
 * @brief Create a new mapping between temperature and fan speed in the Look Up
 * Table. Requires the LUT to be enabled with `LUTEnabled(true)`
 *
 * @param index The index in the LUT, from 0-7. Note that the temperature
 * thresholds should increase with the LUT index, so the temperature value for a
 * given LUT entry is higher than the temperature for the previous index:
 * @code
 * // NO!:
 * setLUT(0, 30, 25); // 25% PWM @ 30 Degrees C
 * setLUT(1, 20, 10); // WRONG! 10% PWM @ 20 Degrees C should be before 30
 * // degrees from index 0 setLUT(2, 40, 50); // 50% PWM @ 40 Degrees C
 *
 * // YES:
 * setLUT(0, 20, 10); // 10% PWM @ 20 Degrees C
 * setLUT(1, 30, 25); // 25% PWM @ 30 Degrees C
 * setLUT(2, 40, 50); // 50% PWM @ 40 Degrees C
 *@endcode
 *
 * @param temp_thresh When the temperature is more than this threshold, the fan
 * will be set to the given PWM value
 * @param fan_pwm The pwm-based fan speed for the given temperature threshold.
 * When DAC output is enabled, this determins the percentage of the maximum
 * output voltage to be used for the given temperature threshold
 * @return true:success false:failure
 */
bool emc2101_set_lut(uint8_t index, uint8_t temp_thresh,
                              uint8_t fan_pwm) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	if (index > 7) {
		return false;
	}
	if (temp_thresh > MAX_LUT_TEMP) {
		return false;
	}
	if (fan_pwm > 100) {
		return false;
	}

  float scalar = (float)fan_pwm / 100.0;
  uint8_t scaled_pwm = (uint8_t)(scalar * MAX_LUT_SPEED);

  uint8_t reg_addr = EMC2101_LUT_START + (2 * index); // speed/pwm is +1

  bool lut_enabled = get_emc2101_lut_enabled_status();

  status_t status = emc2101_i2c_write(reg_addr, &temp_thresh, 1, &device_addr);
  if(kStatus_Success != status){
  	PRINTF("Unable to read internal temperature\r\n");
  	return false;
  }

  status = emc2101_i2c_write(reg_addr + 1, &scaled_pwm, 1, &device_addr);
  if(kStatus_Success != status){
	PRINTF("Unable to read internal temperature\r\n");
	return false;
  }

  emc2101_enable_lut(lut_enabled);

  return true;
}

/**
 * @brief Get the amount of hysteresis in Degrees celcius of hysteresis applied
 * to temperature readings used for the LUT. As the temperature drops, the
 * controller will switch to a lower LUT entry when the measured value is
 * belowthe lower entry's threshold, minus the hysteresis value
 *
 * @return uint8_t The current LUT hysteresis value
 */
bool emc2101_get_lut_hysteresis(uint8_t* hysteresis){

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_read(EMC2101_LUT_HYSTERESIS, hysteresis, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

	return true;
}

/**
 * @brief Set the amount of hysteresis in degrees celcius of hysteresis applied
 * to temperature readings used for the LUT.
 *
 * @param hysteresis  The hysteresis value in degrees celcius. As the
 * temperature drops, the controller will switch to a lower LUT entry when the
 * measured value is `hystersis` degrees below the lower entry's temperature
 * threshold
 *
 * @return uint8_t The current LUT hysteresis value
 */
bool emc2101_set_lut_hysteresis(uint8_t hysteresis) {

	uint8_t device_addr = EMC2101_I2CADDR_DEFAULT;
	status_t status = emc2101_i2c_write(EMC2101_LUT_HYSTERESIS, &hysteresis, 1, &device_addr);
	if(kStatus_Success != status){
		PRINTF("Unable to read internal temperature\r\n");
		return false;
	}

	return true;
}
