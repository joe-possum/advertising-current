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

struct __attribute__((packed)) read_data {
	uint32 reason, interval;
	int16 reqTxPower, TxPower;
	uint8 adLen, pa_mode, flags;
} read_data;

struct __attribute__((packed)) powers {
	uint16 count;
	int16 values[256];
};

#endif /* STRUCT_H_ */
