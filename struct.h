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

#define FLAGS_RETENTION_WRITE       (1u)
#define FLAGS_CHKSUM_FAIL           (2u)
#define FLAGS_OTA_ON_CLOSE          (4u)
#define FLAGS_MEASUREMENT_ACTIVATED (8u)
#define FLAGS_ERROR_NOTIFY_STATE   (16u)

enum dut_measurement_modes { DMM_EM1, DMM_EM2, DMM_EM3, DMM_EM4H, DMM_EM4S, DMM_DTM_TX, DMM_RANDOM, DMM_TEST };

struct __attribute__((packed)) read_data {
	uint32 devinfo_part, reason, interval, delay, connection_interval;
	int16 reqTxPower, TxPower, sleep_clock_accuracy, random_lower, random_upper, random_count;
	uint8 adLen, pa_mode, pa_input, em01vscale, em23vscale, dcdc_mode, measurement_mode, dtm_channel, flags, pti, vcom, mx25_dp;
};

struct __attribute__((packed)) power_settings {
	uint16 count;
	int16 values[255];
};

struct __attribute__((packed)) lynx_dcdc {
	uint32 ipversion, en, ctrl, em12ctrl0,
	em23ctrl0, ien, status, lockstatus;
};

struct __attribute__((packed)) lynx_cmu {
	uint32 ipversion, status, ien, calctrl, calcnt, clken0, clken1,
	sysclkctrl, traceclkctrl, exportclkctrl, dpllrefclkctrl,
	em01grpaclkctrl, em01grpbclkctrl, em23grpaclkctrl, em4grpaclkctrl,
	iadcclkctrl, wdog0clkctrl, euart0clkctrl, rtccclkctrl,
	cryptoaccclkctrl, radioclkctrl;
};

union peripheral_data {
	struct lynx_dcdc lynx_dcdc;
	struct lynx_cmu lynx_cmu;
	uint8 buf[255];
};

enum notification_states { N_NONE = 0,
	N_START_CLOCKS, N_WAIT_CLOCKS, N_CONTINUE_CLOCKS,
	N_START_PERIPHERALS, N_WAIT_PERIPHERALS, N_CONTINUE_PERIPHERALS,
	N_DONE, N_WAIT_DONE, N_ERROR };

struct __attribute__((packed)) status {
	uint16_t count;
	uint8_t type, value_length;
};

#endif /* STRUCT_H_ */
