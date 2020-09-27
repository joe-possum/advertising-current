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

struct __attribute__((packed)) lynx_msc {
  uint32_t ipversion, readctrl, writectrl, addrb, wdata, status, ien, userdatasize, misclockword, pwrctrl, pagelock0, pagelock1;
};

struct __attribute__((packed)) lynx_syscfg {
  uint32_t ien, chiprevhw, chiprev, ctrl, dmem0retnctrl, dmem0eccaddr, dmem0eccctrl, radioramretnctrl, radioeccctrl, seqrameccaddr, frcrameccaddr, rootdata0, rootdata1, rootlockstatus;
};

struct __attribute__((packed)) lynx_burtc {
	uint32 ipversion, en, cfg, status, ien, precnt, cnt, em4wuen, comp;
};

struct __attribute__((packed)) lynx_cmu {
	uint32 ipversion, status, ien, calctrl, calcnt, clken0, clken1,
	sysclkctrl, traceclkctrl, exportclkctrl, dpllrefclkctrl,
	em01grpaclkctrl, em01grpbclkctrl, em23grpaclkctrl, em4grpaclkctrl,
	iadcclkctrl, wdog0clkctrl, euart0clkctrl, rtccclkctrl,
	cryptoaccclkctrl, radioclkctrl;
};

struct __attribute__((packed)) lynx_hfxo {
  uint32_t ipversion, xtalcfg, xtalctrl, cfg, ctrl, status, ien;
};

struct __attribute__((packed)) lynx_hfrco {
  uint32_t ipversion, ctrl, cal, status, ien;
};

struct __attribute__((packed)) lynx_dpll {
  uint32_t ipversion, en, cfg, cfg1, ien, status;
};

struct __attribute__((packed)) lynx_lfxo {
  uint32_t ipversion, ctrl, cfg, status, cal, ien, syncbusy;
};

struct __attribute__((packed)) lynx_lfrco {
  uint32_t ipversion, ctrl, status, ien, cfg, nomcal, nomcalinv;
};

struct __attribute__((packed)) lynx_emu {
  uint32_t decbod, bod3sense, vregvddcmpctrl, pd1paretctrl, ien, em4ctrl, ctrl, templimits, status, temp, rstctrl, rstcause, dgif, dgien, efpif, efpien;
};

struct __attribute__((packed)) lynx_dcdc {
  uint32_t ipversion, en, ctrl, em01ctrl0, em23ctrl0, ien, status, lockstatus;
};

struct __attribute__((packed)) lynx_rtcc {
  uint32_t ipversion, en, cfg, status, ien, precnt, cnt, combcnt, syncbusy;
  struct {
	  uint32_t ctrl, ocvalue, icvalue;
  } cc[3];
};

struct __attribute__((packed)) lynx_usart {
	uint32 ipversion, en, ctrl, frame, trigctrl, status, clkdiv, ien, irctrl, i2sctrl, timing, ctrlx, timecmp0, timecmp1, timecmp2;
};

struct __attribute__((packed)) lynx_euart {
	uint32 ipversion, en, cfg0, cfg1, framecfg, irhfcfg, irlfcfg, timingcfg, startframecfg, sigframecfg, clkdiv, trigctrl, ien;
};

struct __attribute__((packed)) lynx_gpio_port {
	uint32 ctrl, model, modeh, dout, din;
};

struct __attribute__((packed)) lynx_gpio {
	struct lynx_gpio_port ports[4];
};

union peripheral_data {
	struct lynx_msc lynx_msc;
	struct lynx_syscfg lynx_syscfg;
	struct lynx_cmu lynx_cmu;
	struct lynx_hfxo lynx_hfxo;
	struct lynx_hfrco lynx_hfrco;
	struct lynx_dpll lynx_dpll;
	struct lynx_lfxo lynx_lfxo;
	struct lynx_lfrco lynx_lfrco;
	struct lynx_emu lynx_emu;
	struct lynx_dcdc lynx_dcdc;
	struct lynx_rtcc lynx_rtcc;
	struct lynx_burtc lynx_burtc;
	struct lynx_usart lynx_usart;
	struct lynx_euart lynx_euart;
	struct lynx_gpio lynx_gpio;
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
