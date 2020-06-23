/*
 * struct.h
 *
 *  Created on: Mar 24, 2020
 *      Author: maorchar
 */

#ifndef STRUCT_H_
#define STRUCT_H_

struct gpio_pin_count {
	uint8_t port[11];
};

struct gpio {
	uint32 model, modeh, dout;
};

struct __attribute__((__packed__)) tx_power {
	uint16 id;
  int16 attempt, set;
};

enum dut_measurement_modes { DMM_EM1, DMM_EM2, DMM_EM3, DMM_EM4H, DMM_EM4S, DMM_DTM_TX, DMM_RANDOM };

struct __attribute__((packed)) read_data {
	uint32 reason, interval, delay, connection_interval;
	int16 reqTxPower, TxPower, sleep_clock_accuracy, random_lower, random_upper, random_count;
	uint8 adLen, pa_mode, pa_input, em01vscale, em23vscale, dcdc_mode, measurement_mode, dtm_channel, flags;
} read_data = {
		.random_lower = -270,
		.random_upper = 200,
		.random_count = 20,
};

struct __attribute__((packed)) power_settings {
	uint16 count;
	int16 values[255];
};

#endif /* STRUCT_H_ */
