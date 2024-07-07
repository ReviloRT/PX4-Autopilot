/****************************************************************************
 *
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file NAU7802.cpp
 *
 * @author Agustin Soto and Oliver Vannoort
 */

#include "NAU7802.hpp"

// PX4 Specific ****************************************************

NAU7802::NAU7802(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{}

NAU7802::~NAU7802() {}

// Returns whether the sensor is good
int NAU7802::probe() {
	uint8_t code;
	int status = getRevisionCode(&code);
	if (status != PX4_OK) return status;
	// if (code != 0x0F) return PX4_ERROR;

	return PX4_OK;
}

// Initalises the Sensor
int NAU7802::init() {
	PX4_WARN("NAU7802 Initalising!");
	int status = I2C::init();
	if (status != PX4_OK) return status;
	status = begin();
	if (status != PX4_OK) return status;

	ScheduleNow();
	return PX4_OK;
}

// Prints the current status of the I2C connection
void NAU7802::print_status() {
	I2CSPIDriverBase::print_status();
}

// Runs a sensor reading and reschedules itself to run repeatedly
void NAU7802::RunImpl() {

	// Take and publish sensor reading
	PublishMessage();

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}

// Generates and publishes a UORB message by taking a reading
void NAU7802::PublishMessage() {

	const hrt_abstime timestamp = hrt_absolute_time();
	int32_t reading = 0;
	int error = getReading(&reading);

	float force_n = reading/128.0f*gainAdj+zeroOffset;

	force_sensor_s force_msg{};
	force_msg.timestamp = timestamp;
	force_msg.error_status = error;
	force_msg.force_measurement_n = force_n;

	_force_sensor_pub.publish(force_msg);

}


// Sensor Specific ****************************************************

// Initalises and sets up the sensor, returns true if initalised correctly
int NAU7802::begin() {
	int status = PX4_OK;
	status = reset(); //Reset all register;
		if (status != PX4_OK) return status;
	status = powerUp(); //Power on analog and digital sections of the scal;
		if (status != PX4_OK) return status;
	status = setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3;
		if (status != PX4_OK) return status;
	status = setGain(NAU7802_GAIN_128); //Set gain to 12;
		if (status != PX4_OK) return status;
	status = setSampleRate(NAU7802_SPS_80); //Set samples per second to 8;
		if (status != PX4_OK) return status;
	uint8_t adc;
	status = getRegister(NAU7802_ADC,&adc); //Turn off CLK_CHP. From 9.1 power on sequencing.
		if (status != PX4_OK) return status;
	adc |= 0x30;
	status = setRegister(NAU7802_ADC, adc);
		if (status != PX4_OK) return status;
	status = setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note;
		if (status != PX4_OK) return status;
	status = clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA); //Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 oh;
		if (status != PX4_OK) return status;

	usleep(_ldoRampDelay * 1000); // Wait for LDO to stabilize - takes about 200ms

	// getReading(nullptr); // Take ten readings to flush - Not working in PX4
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);
	// getReading(nullptr);

	status = calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channe;
		if (status != PX4_OK) return status;

	return status;
}

int NAU7802::reset() {
	int status = setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);  //Set R
		if (status != PX4_OK) return status;

	px4_usleep(1000);

	status = clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);  //Clear RR to leave reset stat
		if (status != PX4_OK) return status;

  	return PX4_OK;
}

int NAU7802::powerUp() {
	int status = setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	if (status != PX4_OK) return status;
	status = setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);
	if (status != PX4_OK) return status;

	//Wait for Power Up bit to be set - takes approximately 200us
	int counter = 0;
	bool bit = false;
	while (bit == false) {
		status = getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &bit);
		if (status != PX4_OK) return status;
		px4_usleep(1000);
		if (counter++ > 100)
			return PX4_ERROR; //Error
	}
	status = setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);  // Set Cycle Start bit. See 9.1 point ;
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::getRevisionCode(uint8_t *code) {
	int status = getRegister(NAU7802_DEVICE_REV, code);
	if (status != PX4_OK) return status;

	// *code = (*code & 0xFF);

  	return PX4_OK;
}

int NAU7802::setGain(uint8_t gainValue) {
	if (gainValue > 0b111)
    		gainValue = 0b111; //Error check

	uint8_t reg;
	int status = getRegister(NAU7802_CTRL1, &reg);
	if (status != PX4_OK) return status;

	reg &= 0b11111000; //Clear gain bits
	reg |= gainValue;  //Mask in new bits

	status = setRegister(NAU7802_CTRL1, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::setLDO(uint8_t ldoValue) {
	if (ldoValue > 0b111)
    		ldoValue = 0b111; //Error check

	//Set the value of the LDO
	uint8_t reg;
	int status = getRegister(NAU7802_CTRL1, &reg);
	if (status != PX4_OK) return status;

	reg &= 0b11000111;    //Clear LDO bits
	reg |= ldoValue << 3; //Mask in new LDO bits


	status = setRegister(NAU7802_CTRL1, reg);
	if (status != PX4_OK) return status;
	status = setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::setSampleRate(uint8_t rate) {
	 if (rate > 0b111)
   		 rate = 0b111; //Error check

	uint8_t reg;
	int status = getRegister(NAU7802_CTRL2, &reg);
	if (status != PX4_OK) return status;

	reg &= 0b10001111; //Clear CRS bits
	reg |= rate << 4;  //Mask in new CRS bits

	status = setRegister(NAU7802_CTRL2, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}


int NAU7802::getReading(int32_t *reading) {
	int32_t val = 0;
	int status = get24BitRegister(NAU7802_ADCO_B2, &val);
	if (status != PX4_OK) return status;

	*reading = val;
	return PX4_OK;

}


int NAU7802::calibrateAFE(NAU7802_Cal_Mode mode) {
	int status = beginCalibrateAFE(NAU7802_CALMOD_INTERNAL);
	if (status != PX4_OK) return status;
	status = waitForCalibrateAFE(1000);
	if (status != PX4_OK) return status;
  	return PX4_OK;
}

int NAU7802::beginCalibrateAFE(NAU7802_Cal_Mode mode) {
	uint8_t reg;
	int status = getRegister(NAU7802_CTRL2, &reg);
	if (status != PX4_OK) return status;

	reg &= 0xFC; // Clear CALMOD bits
	uint8_t calMode = (uint8_t)mode;
  	calMode &= 0x03; // Limit mode to 2 bits
  	reg |= calMode; // Set the mode

	status = setRegister(NAU7802_CTRL2, reg);
	if (status != PX4_OK) return status;

	status = setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
	if (status != PX4_OK) return status;

  	return PX4_OK;
}

int NAU7802::waitForCalibrateAFE(unsigned long timeout_ms) {

	uint64_t startTime = hrt_absolute_time()/1000;
	NAU7802_Cal_Status cal_ready = calAFEStatus();

	while (cal_ready == NAU7802_CAL_IN_PROGRESS) {
		if ((timeout_ms > 0) && (((hrt_absolute_time()/1000) - startTime) > timeout_ms)) {
			break;
		}
		px4_usleep(1000);
		cal_ready = calAFEStatus();
	}
	if (cal_ready == NAU7802_CAL_PX4_ERROR) {
		return PX4_ERROR;
	}
	if (cal_ready == NAU7802_CAL_FAILURE) {
		return PX4_ERROR;
	}
	return PX4_OK;
}

NAU7802_Cal_Status NAU7802::calAFEStatus() {
	bool bit;

	int status = getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2, &bit);
	if (status != PX4_OK) return NAU7802_CAL_PX4_ERROR;

	if (bit) return NAU7802_CAL_IN_PROGRESS;

	status = getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2, &bit);
	if (status != PX4_OK) return NAU7802_CAL_PX4_ERROR;

	if (bit) return NAU7802_CAL_FAILURE;

	// Calibration passed
	return NAU7802_CAL_SUCCESS;
}


int NAU7802::getRegister(uint8_t registerAddress, uint8_t *data) {
	// // PX4 Transfer Function
	// // I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)

	// // Send the register address and return 1 byte response
	// uint8_t val;
	// uint8_t cmd = registerAddress;
	// transfer(&cmd, 1, &val, sizeof(val));

	// // TODO: Handle errors here
	// return val;

	uint8_t cmd = registerAddress;
	int status = transfer(&cmd, sizeof(cmd), data, sizeof(*data));
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::setRegister(uint8_t registerAddress, uint8_t value) {
	// Send the two byte command and return success
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>(value);

	int status = transfer(&cmd[0], 2, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::get24BitRegister(uint8_t registerAddress, int32_t *data) {
	// PX4 Transfer Function
	// I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)

	// Send the 1 byte command and record the 3 byte response
	uint8_t val[3] = {0,0,0};
	uint8_t cmd = registerAddress;

	int status = transfer(&cmd, 1, &val[0], sizeof(val));
	if (status != PX4_OK) return status;

	// Construct a union for conversion between unsigned and signed
	union {
		uint32_t usign;
		int32_t  sign;
	} union32_t;

	// Fill the union, convert to signed integer and return
	union32_t.usign = (uint32_t)val[0] << 16; //MSB
	union32_t.usign |= (uint32_t)val[1] << 8; //MidSB
	union32_t.usign |= (uint32_t)val[2];      //LSB
	if ((union32_t.usign & 0x00800000) == 0x00800000)
		union32_t.usign |= 0xFF000000; // Preserve 2's complement

	*data = (union32_t.sign);
	return PX4_OK;

}

int NAU7802::set24BitRegister(uint8_t registerAddress, int32_t value) {

	// Construct a union for conversion between unsigned and signed and fill with value
	union {
		uint32_t usign;
		int32_t  sign;
	} union32_t;
	union32_t.sign = value;

	// Fill the array of commands
	uint8_t cmd[4];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>((union32_t.usign >> 16) & 0xFF);
	cmd[2] = static_cast<uint8_t>((union32_t.usign >> 8) & 0xFF);
	cmd[3] = static_cast<uint8_t>(union32_t.usign & 0xFF);

	int status = transfer(&cmd[0], 4, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::get32BitRegister(uint8_t registerAddress, uint32_t *data) {
	// Send the register address and return 4 byte response
	uint8_t val[4] = {0,0,0,0};
	uint8_t cmd = registerAddress;

	int status = transfer(&cmd, 1, &(val[0]), sizeof(val));
	if (status != PX4_OK) return status;

	// Convert the 4 bytes to a single 32 bit unsigned integer
	uint32_t val32;
	val32  = (uint32_t)val[0] << 24; //MSB
	val32 |= (uint32_t)val[1] << 16;
	val32 |= (uint32_t)val[2] << 8;
	val32 |= (uint32_t)val[3];       //LSB

	// TODO: Handle errors here
	*data = val32;
	return PX4_OK;

}

int NAU7802::set32BitRegister(uint8_t registerAddress, uint32_t value) {
	// Fill the command array with the value and send
	uint8_t cmd[5];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>((value >> 24) & 0xFF);
	cmd[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
	cmd[3] = static_cast<uint8_t>((value >> 8 ) & 0xFF);
	cmd[4] = static_cast<uint8_t>( value        & 0xFF);

	int status = transfer(&cmd[0], 5, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;

}

int NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress, &reg);
	if (status != PX4_OK) return status;

	reg |= (1 << bitNumber); //Set this bit
	status = setRegister(registerAddress, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress, &reg);
	if (status != PX4_OK) return status;

	reg &= ~(1 << bitNumber); //Set this bit
	status =  setRegister(registerAddress, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

int NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress, bool *data) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress,&reg);
	if (status != PX4_OK) return status;

	reg &= (1 << bitNumber);
	*data = (bool)reg;

	return PX4_OK;
}


// int NAU7802::module_set(const BusCLIArguments &cli, BusInstanceIterator &iterator)
// {
// 	bool is_running = false;

// 	while (iterator.next()) {
// 		if (iterator.instance()) {
// 			NAU7802 *instance = (NAU7802 *)iterator.instance();
// 			instance->zeroOffset = cli.zero;
// 			instance->gainAdj = cli.gain_adj;
// 			PX4_INFO("Set Zero Offset to %f and Gain Adjustment to %f", instance->_zeroOffset, instance->gainAdj);
// 			is_running = true;
// 		}
// 	}

// 	if (!is_running) {
// 		PX4_INFO("Not running");
// 		return -1;
// 	}

// 	return 0;
// }
