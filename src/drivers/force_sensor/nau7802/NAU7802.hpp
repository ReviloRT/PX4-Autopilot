/****************************************************************************
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file NAU7802.hpp
 *
 * Driver for NAU7802 connected via I2C.
 *
 * Supported sensors:
 *
 *    - NAU7802
 *
 * Interface application notes:
 *
 *
 * @author Oliver Vannoort and Agustin Soto <opvannoort@gmail.com>
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

// #include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/force_sensor.h>
#include <uORB/topics/differential_pressure.h>

#include <px4_platform_common/module.h>
#include <lib/systemlib/mavlink_log.h>


//Register Map
typedef enum
{
  NAU7802_PU_CTRL = 0x00,
  NAU7802_CTRL1,
  NAU7802_CTRL2,
  NAU7802_OCAL1_B2,
  NAU7802_OCAL1_B1,
  NAU7802_OCAL1_B0,
  NAU7802_GCAL1_B3,
  NAU7802_GCAL1_B2,
  NAU7802_GCAL1_B1,
  NAU7802_GCAL1_B0,
  NAU7802_OCAL2_B2,
  NAU7802_OCAL2_B1,
  NAU7802_OCAL2_B0,
  NAU7802_GCAL2_B3,
  NAU7802_GCAL2_B2,
  NAU7802_GCAL2_B1,
  NAU7802_GCAL2_B0,
  NAU7802_I2C_CONTROL,
  NAU7802_ADCO_B2,
  NAU7802_ADCO_B1,
  NAU7802_ADCO_B0,
  NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
  NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
  NAU7802_OTP_B0,     //OTP 15:8
  NAU7802_PGA = 0x1B,
  NAU7802_PGA_PWR = 0x1C,
  NAU7802_DEVICE_REV = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
  NAU7802_CTRL1_GAIN = 2,
  NAU7802_CTRL1_VLDO = 5,
  NAU7802_CTRL1_DRDY_SEL = 6,
  NAU7802_CTRL1_CRP = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
  NAU7802_CTRL2_CALMOD = 0,
  NAU7802_CTRL2_CALS = 2,
  NAU7802_CTRL2_CAL_ERROR = 3,
  NAU7802_CTRL2_CRS = 4,
  NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
  NAU7802_PGA_CHP_DIS = 0,
  NAU7802_PGA_INV = 3,
  NAU7802_PGA_BYPASS_EN,
  NAU7802_PGA_OUT_EN,
  NAU7802_PGA_LDOMODE,
  NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
  NAU7802_PGA_PWR_PGA_CURR = 0,
  NAU7802_PGA_PWR_ADC_CURR = 2,
  NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
  NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
  NAU7802_LDO_2V4 = 0b111,
  NAU7802_LDO_2V7 = 0b110,
  NAU7802_LDO_3V0 = 0b101,
  NAU7802_LDO_3V3 = 0b100,
  NAU7802_LDO_3V6 = 0b011,
  NAU7802_LDO_3V9 = 0b010,
  NAU7802_LDO_4V2 = 0b001,
  NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
  NAU7802_GAIN_128 = 0b111,
  NAU7802_GAIN_64 = 0b110,
  NAU7802_GAIN_32 = 0b101,
  NAU7802_GAIN_16 = 0b100,
  NAU7802_GAIN_8 = 0b011,
  NAU7802_GAIN_4 = 0b010,
  NAU7802_GAIN_2 = 0b001,
  NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
  NAU7802_SPS_320 = 0b111,
  NAU7802_SPS_80 = 0b011,
  NAU7802_SPS_40 = 0b010,
  NAU7802_SPS_20 = 0b001,
  NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
  NAU7802_CHANNEL_1 = 0,
  NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
  NAU7802_CAL_SUCCESS = 0,
  NAU7802_CAL_IN_PROGRESS = 1,
  NAU7802_CAL_FAILURE = 2,
} NAU7802_Cal_Status;

//Calibration mode
typedef enum
{
  NAU7802_CALMOD_INTERNAL = 0,
  NAU7802_CALMOD_OFFSET = 2,
  NAU7802_CALMOD_GAIN,
} NAU7802_Cal_Mode;



/* Measurement rate is 200Hz */
#define MEAS_RATE 200
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */


/* Configuration Constants */
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x2A; /* 0x2A */
static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

// using namespace time_literals;


class NAU7802 : public device::I2C, public I2CSPIDriver<NAU7802> {
public:
	// PX4 Specific ****************************************************

	NAU7802(const I2CSPIDriverConfig &config);
	~NAU7802() override;
	static void print_usage();
	void print_status() override;
	void RunImpl();
	int init() override;


private:
	// PX4 Specific ****************************************************

	int probe() override;

	// hrt_abstime last_sample_time = hrt_absolute_time(); // Not sure if we need this?
	// orb_advert_t 	_mavlink_log_pub {nullptr}; //log send to


	// uint32_t _measure_interval{CONVERSION_INTERVAL};
	// uint32_t _conversion_interval{CONVERSION_INTERVAL};

	// bool _sensor_ok{false};

	uORB::PublicationMulti<force_sensor_s> _force_sensor_pub{ORB_ID(force_sensor)};

	// perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	// perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
	// perf_counter_t _fault_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": fault detected")};




	// Sensor Specific ****************************************************

	// Variables
	int32_t _zeroOffset = 0;
	float _calibrationFactor = 1.0;

	unsigned long _ldoRampDelay = 250;

	// Functions
  	bool begin(bool reset = true); //Check communication and initialize sensor
  	bool reset(); //Resets all registers to Power Of Defaults
  	bool powerUp();   //Power up digital and analog sections of scale, ~2mA
	uint8_t getRevisionCode()

	bool setGain(uint8_t gainValue);
	bool setLDO(uint8_t ldoValue);
	bool setSampleRate(uint8_t rate);

	int32_t getReading();

	bool calibrateAFE(NAU7802_Cal_Mode mode = NAU7802_CALMOD_INTERNAL);      //Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
	void beginCalibrateAFE(NAU7802_Cal_Mode mode = NAU7802_CALMOD_INTERNAL); //Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
	bool waitForCalibrateAFE(unsigned long timeout_ms = 0); //Wait for asynchronous AFE calibration to complete with optional timeout.
	NAU7802_Cal_Status calAFEStatus();                 //Check calibration status.

	uint8_t getRegister(uint8_t registerAddress);             //Get contents of a register
	bool setRegister(uint8_t registerAddress, uint8_t value); //Send a given value to be written to given address. Return true if successful
	int32_t get24BitRegister(uint8_t registerAddress);        //Get contents of a 24-bit signed register (conversion result and offsets)
	bool set24BitRegister(uint8_t registerAddress, int32_t value); //Send 24 LSBs of value to given register address. Return true if successful
	uint32_t get32BitRegister(uint8_t registerAddress);       //Get contents of a 32-bit register (gains)
	bool set32BitRegister(uint8_t registerAddress, uint32_t value); //Send a given value to be written to given address. Return true if successful
	bool setBit(uint8_t bitNumber, uint8_t registerAddress);   //Mask & set a given bit within a register
	bool clearBit(uint8_t bitNumber, uint8_t registerAddress); //Mask & clear a given bit within a register
	bool getBit(uint8_t bitNumber, uint8_t registerAddress);   //Return a given bit within a register

};

