/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"
#include "dump.h"

#include "common.h"
#include "struct.h"
#include <stdio.h>
#include "em_device.h"
#include "em_rmu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "sleep.h"
#include <stdlib.h>
#include "bsp.h"
#include "pti.h"
#include "mx25flash_spi.h"

uint8 adData[31];
uint8 notify = 0; /* perform notifications of power settings */

struct read_data read_data = {
		.random_lower = -270,
		.random_upper = 200,
		.random_count = 20,
};
struct gpio_pin_count gpio_pin_count = {{
		_GPIO_PORT_A_PIN_COUNT,
		_GPIO_PORT_B_PIN_COUNT,
		_GPIO_PORT_C_PIN_COUNT,
		_GPIO_PORT_D_PIN_COUNT,
		_GPIO_PORT_E_PIN_COUNT,
		_GPIO_PORT_F_PIN_COUNT,
		_GPIO_PORT_G_PIN_COUNT,
		_GPIO_PORT_H_PIN_COUNT,
		_GPIO_PORT_I_PIN_COUNT,
		_GPIO_PORT_J_PIN_COUNT,
		_GPIO_PORT_K_PIN_COUNT
}};

#ifdef _SILICON_LABS_32B_SERIES_1
const uint32 GPIO_PORT_COUNT = (sizeof(GPIO->P)/sizeof(GPIO_P_TypeDef));
#endif
#ifdef _SILICON_LABS_32B_SERIES_2
const uint32 GPIO_PORT_COUNT = (sizeof(GPIO->P)/sizeof(GPIO_PORT_TypeDef));
#endif
struct gpio gpio_config[12u];

struct manData {
	uint16 id;
} manData = { .id = 0xffff, };

uint16 mtu = 27;
struct read_data read_data;
struct power_settings power_settings;

#ifdef DUMP
#undef gecko_cmd_system_set_tx_power
#endif
void set_power(void) {
	struct gecko_msg_system_set_tx_power_rsp_t *resp;
	gecko_cmd_system_halt(1);
	resp = gecko_cmd_system_set_tx_power(read_data.reqTxPower);
	gecko_cmd_system_halt(0);
	read_data.TxPower = resp->set_power;
}

uint8 measurement_active;
struct tx_power tx_power = { .id = 0xffff };

void adv_random_power(void) {
  int16 try;
  do {
    try = read_data.random_lower + (rand() % (read_data.random_upper - read_data.random_lower + 1));
  } while(try == tx_power.attempt);
  tx_power.attempt = try;
  gecko_cmd_system_halt(1);
  tx_power.set = gecko_cmd_system_set_tx_power(tx_power.attempt)->set_power;
  gecko_cmd_system_halt(0);
  uint8 buf[31];
  uint8 len = 0;
  len += ad_name(&buf[len],"TX Power");
  len += ad_manufacturer(&buf[len],sizeof(tx_power),(uint8*)&tx_power);
  gecko_cmd_le_gap_bt5_set_adv_data(1,0,len,buf);
  gecko_cmd_le_gap_start_advertising(1,le_gap_user_data,le_gap_non_connectable);
}


void set_ad_data(void) {
	uint8 len = 0;
	len += ad_flags(&adData[len],6);
	if(read_data.adLen > (len+7)) {
		len += ad_name(&adData[len],"BG Control");
	}
	if(read_data.adLen > (len + 4)) {
		ad_manufacturer(&adData[len],read_data.adLen-len-2,(uint8*)&manData);
	}
	gecko_cmd_le_gap_bt5_set_adv_data(0,0,read_data.adLen,adData);
}

int16 req, set, prev;
uint8 pending = 0, conn, reset_on_close, ota_on_close;
uint16 result;

void send_power_settings(uint8 connection, uint16 handle, uint16 offset) {
	uint8 *ptr = (uint8*)&power_settings;
	uint16 total = 2 + 2*power_settings.count - offset;
	if(total > (mtu-1)) {
		total = mtu - 1;
	}
	gecko_cmd_gatt_server_send_user_read_response(connection,handle,0,total,ptr+offset);
}

union peripheral_data peripheral_data;

#ifdef DCDC
uint8 get_dcdc(void) {
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
	struct lynx_dcdc *p = &peripheral_data.lynx_dcdc;
	p->ipversion = DCDC->IPVERSION;
	p->en = DCDC->EN;
	p->ctrl = DCDC->CTRL;
	p->em12ctrl0 = DCDC->EM01CTRL0;
	p->em23ctrl0 = DCDC->EM23CTRL0;
	p->ien = DCDC->IEN;
	p->status = DCDC->STATUS;
	p->lockstatus = DCDC->LOCKSTATUS;
	return sizeof(struct lynx_dcdc);
#else
#  error get_dcdc is not implemented for this family
#endif
}
#endif

uint8 get_cmu(void) {
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
	struct lynx_cmu *p = &peripheral_data.lynx_cmu;
	p->ipversion = CMU->IPVERSION;
	p->status = CMU->STATUS;
	p->ien = CMU->IEN;
	p->calctrl = CMU->CALCTRL;
	p->calcnt = CMU->CALCNT;
	p->clken0 = CMU->CLKEN0;
	p->clken1 = CMU->CLKEN1;
	p->sysclkctrl = CMU->SYSCLKCTRL;
	p->traceclkctrl = CMU->TRACECLKCTRL;
	p->exportclkctrl = CMU->EXPORTCLKCTRL;
	p->dpllrefclkctrl = CMU->DPLLREFCLKCTRL;
	p->em01grpaclkctrl = CMU->EM01GRPACLKCTRL;
	p->em01grpbclkctrl = CMU->EM01GRPBCLKCTRL;
	p->em23grpaclkctrl = CMU->EM23GRPACLKCTRL;
	p->em4grpaclkctrl = CMU->EM4GRPACLKCTRL;
	p->iadcclkctrl = CMU->IADCCLKCTRL;
	p->wdog0clkctrl = CMU->WDOG0CLKCTRL;
	p->euart0clkctrl = CMU->EUART0CLKCTRL;
	p->rtccclkctrl = CMU->RTCCCLKCTRL;
	p->cryptoaccclkctrl = CMU->CRYPTOACCCLKCTRL;
	p->radioclkctrl = CMU->RADIOCLKCTRL;
	return sizeof(struct lynx_cmu);
#else
#  error get_cmu is not implemented for this family
#endif
}

uint8 get_usart(USART_TypeDef *USART) {
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
	struct lynx_usart *p = &peripheral_data.lynx_usart;
	p->ipversion = USART->IPVERSION;
	p->en = USART->EN;
	p->ctrl = USART->CTRL;
	p->frame = USART->FRAME;
	p->trigctrl = USART->TRIGCTRL;
	p->status = USART->STATUS;
	p->clkdiv = USART->CLKDIV;
	p->ien = USART->IEN;
	p->irctrl = USART->IRCTRL;
	p->i2sctrl = USART->I2SCTRL;
	p->timing = USART->TIMING;
	p->timecmp0 = USART->TIMECMP0;
	p->timecmp1 = USART->TIMECMP1;
	p->timecmp2 = USART->TIMECMP2;
	return sizeof(struct lynx_usart);
#else
#  error get_usart is not implemented for this family
#endif
}

#define M(X) { X, #X }
const struct __attribute__((packed)) clocks {
	CMU_Clock_TypeDef clock;
	const char *name;
} clocks[] = {
		M(cmuClock_DCDC),
		M(cmuClock_GPIO),
		M(cmuClock_BURAM),
		M(cmuClock_BURTC),
		M(cmuClock_I2C0),
		M(cmuClock_I2C1),
		M(cmuClock_IADC0),
		M(cmuClock_IADCCLK),
		M(cmuClock_LDMA),
		M(cmuClock_LDMAXBAR),
		M(cmuClock_LETIMER0),
		M(cmuClock_TIMER0),
		M(cmuClock_TIMER1),
		M(cmuClock_TIMER2),
		M(cmuClock_TIMER3),
		M(cmuClock_TIMER4),
		M(cmuClock_TRACECLK),
		M(cmuClock_USART0),
		M(cmuClock_USART1),
		M(cmuClock_PRS),
};
const struct __attribute__((packed)) peripherals {
	void* address;
	const char *name;
} peripherals[] = {
		M(MSC),
#ifdef DBG
		M(DBG),
#endif
		M(CMU),
		M(HFXO0),
		M(HFRCO0),
		M(DPLL0),
		M(LFXO),
		M(LFRCO),
		M(SMU),
		M(CRYPTOACC),
		M(EMU),
		M(DCDC),
		M(PRS),
		M(GPCRC),
		M(RTCC),
		M(BURTC),
		M(BURAM),
		M(LETIMER0),
		M(TIMER0),
		M(TIMER1),
		M(TIMER2),
		M(TIMER3),
		M(TIMER4),
		M(PDM),
		M(USART0),
		M(USART1),
		M(EUART0),
		M(I2C0),
		M(I2C1),
		M(IADC0),
		M(GPIO),
		M(LDMA),
		M(LDMAXBAR),
		M(WDOG0),
};

uint8_t notification_active = 0;
enum notification_states state = N_DONE;
uint8_t n_index;
uint16 n_count;

void start_notification (enum notification_states new_state) {
#ifdef DUMP
	printf("start_notification(%d) state: %d\n",new_state, state);
#endif
	struct status status;
	state = new_state;
	notification_active = 0;
	n_index = 0;
	switch(state) {
	case N_DONE:
		status.type = N_DONE;
		status.value_length = 0;
		status.count = 0;
		break;
	default:
		read_data.flags |= FLAGS_ERROR_NOTIFY_STATE;
		// no break
	case N_ERROR:
		state = N_DONE;
		status.type = N_ERROR;
		break;
	case N_START_CLOCKS:
		n_count = sizeof(clocks)/sizeof(clocks[0]);
		status.type = state;
		status.value_length = sizeof(CMU_Clock_TypeDef);
		status.count = n_count;
		break;
	case N_START_PERIPHERALS:
		n_count = sizeof(peripherals)/sizeof(peripherals[0]);
		status.type = state;
		status.value_length = sizeof(void*);
		status.count = n_count;
		break;
	}
	if(gecko_cmd_gatt_server_send_characteristic_notification(conn,gattdb_device_data_status,sizeof(status),(uint8*)&status)->result) {
		gecko_external_signal(1); // nasty hack to retry
	} else {
		state++;
	}
}

void continue_notification(void) {
#ifdef DUMP
	printf("continue_notification() state: %d\n", state);
#endif
	uint8 len_const, len_str;
	const void *data_const, *data_str;
	switch(state) {
	case N_CONTINUE_CLOCKS:
		data_const = &clocks[n_index];
		len_const = sizeof(clocks[n_index]) - sizeof(const char *);
		data_str = clocks[n_index].name;
		len_str = strlen(clocks[n_index].name);
		break;
	case N_CONTINUE_PERIPHERALS:
		data_const = &peripherals[n_index];
		len_const = sizeof(peripherals[n_index]) - sizeof(const char *);
		data_str = peripherals[n_index].name;
		len_str = strlen(peripherals[n_index].name);
		break;
	default:
		start_notification(N_ERROR);
		return;
	}
	memcpy(peripheral_data.buf,data_const,len_const);
	memcpy(&peripheral_data.buf[len_const],data_str,len_str);
	struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t *resp;
	resp = gecko_cmd_gatt_server_send_characteristic_notification(conn, gattdb_device_data_data, len_const + len_str, &peripheral_data.buf[0]);
	switch(resp->result) {
	case 0:
		n_index++;
		if(n_index == n_count) {
			start_notification(state+1);
		}
	}
}

#define N_RET 4
void retention_read(void) {
#if defined(_SILICON_LABS_32B_SERIES_2)
#  if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    CMU_ClockEnable(cmuClock_BURAM,1);
#  endif
#  define RET BURAM->RET
#else
#  if defined(_SILICON_LABS_32B_SERIES_1)
#    define RET RTCC->RET
    CMU_ClockEnable(cmuClock_RTCC,1);
#  endif
#endif
 	uint32 chksum = 0;
	for(int i = 0; i < N_RET; i++) {
		chksum += RET[i].REG;
#ifdef DUMP
		printf("RET[%d].REG: %08lx\n",i,RET[i].REG);
#endif
	}
#ifdef DUMP
	printf("chksum: %08lx\n",chksum);
#endif
	if(0 == chksum) {
	  read_data.pa_mode = RET[1].REG & 0xff;
	  read_data.pa_input = (RET[1].REG >> 8) & 0xff;
	  read_data.sleep_clock_accuracy = (RET[1].REG >> 16) & 0xff;
	  read_data.dcdc_mode = (RET[1].REG >> 24) & 0xff;
	  read_data.em01vscale = RET[2].REG & 0xff;
	  read_data.em23vscale = (RET[2].REG >> 8) & 0xff;
	  read_data.pti = (RET[2].REG >> 16) & 1;
	  read_data.mx25_dp = (RET[2].REG >> 17) & 1;
	} else {
		read_data.flags |= FLAGS_CHKSUM_FAIL;
	}
#if defined(_SILICON_LABS_32B_SERIES_2)
#  if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    CMU_ClockEnable(cmuClock_BURAM,0);
#  endif
#else
#  if defined(_SILICON_LABS_32B_SERIES_1)
#  endif
#endif
}

void retention_write(void) {
#if defined(_SILICON_LABS_32B_SERIES_2)
#  if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    CMU_ClockEnable(cmuClock_BURAM,1);
#  endif
#  define RET BURAM->RET
#else
#  if defined(_SILICON_LABS_32B_SERIES_1)
#    define RET RTCC->RET
#  endif
#endif
 	uint32 chksum = 0;
 	uint16 init_bits = 0;
 	if(read_data.pti) init_bits |= 1;
 	if(read_data.mx25_dp) init_bits |= 2;
 	RET[1].REG = read_data.pa_mode | (read_data.pa_input << 8) | (read_data.sleep_clock_accuracy << 16) | (read_data.dcdc_mode << 24);
 	RET[2].REG =  read_data.em01vscale | (read_data.em23vscale << 8) | (init_bits << 16);
	for(int i = 1; i < N_RET; i++) {
		chksum += RET[i].REG;
	}
	RET[0].REG = -chksum;
#ifdef DUMP
	for(int i = 1; i < N_RET; i++) {
		printf("RET[%d].REG: %08lx\n",i,RET[i].REG);
	}
#endif
	read_data.flags |= FLAGS_RETENTION_WRITE;
#if defined(_SILICON_LABS_32B_SERIES_2)
#  if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    CMU_ClockEnable(cmuClock_BURAM,0);
#  endif
#else
#  if defined(_SILICON_LABS_32B_SERIES_1)
#  endif
#endif
}

#ifdef EMU_RSTCAUSE_SYSREQ
#define SOFTWARE_RESET EMU_RSTCAUSE_SYSREQ
#endif
#ifdef RMU_RSTCAUSE_SYSREQRST
#define SOFTWARE_RESET RMU_RSTCAUSE_SYSREQRST
#endif
#ifndef SOFTWARE_RESET
#error SOFTWARE_RESET not defined
#endif
/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
  read_data.reason = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
  /*----------------------------------------------------------------- Set default values of parameters */
  read_data.devinfo_part = DEVINFO->PART;
  read_data.pa_mode = pconfig->pa.pa_mode;
  read_data.pa_input = pconfig->pa.input;
  read_data.sleep_clock_accuracy = pconfig->bluetooth.sleep_clock_accuracy;
#ifdef DCDC
#  if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
  	  read_data.dcdc_mode = DCDC->STATUS & 1;
#  else
#    error DCDC mode not defined
#  endif
#endif
#if defined(EMU_VSCALE_PRESENT)
  read_data.em23vscale = (EMU->CTRL & _EMU_CTRL_EM23VSCALE_MASK) >> _EMU_CTRL_EM23VSCALE_SHIFT;
  read_data.em01vscale = EMU_VScaleGet();
#endif
  read_data.flags = 0;
  read_data.vcom = HAL_VCOM_ENABLE;
  read_data.pti = 1;
  read_data.mx25_dp = 1;
  /* ---------------------------------------------------------------- Override defaults with save settings is valid */
  if(read_data.reason & SOFTWARE_RESET) { /* software reset */
	  retention_read();
  }
  pconfig->pa.pa_mode = read_data.pa_mode;
  reset_on_close = 0;
  ota_on_close = 0;
#ifdef DCDC
  if(read_data.dcdc_mode) {
#  ifdef DUMP
	  printf("EMU_DCDCModeSet(%d)\n",read_data.dcdc_mode);
#  endif
	  EMU_DCDCModeSet((EMU_DcdcMode_TypeDef)read_data.dcdc_mode);
  }
#endif

  if(read_data.mx25_dp) {
#ifdef DUMP
	  printf("MX25_init();\nMX25_DP();\nMX25_deinit();\n");
#endif
	  MX25_init();
	  MX25_DP();
	  MX25_deinit();
  }
  if(read_data.pti) {
#ifdef DUMP
	  printf("configEnablePti();\n");
#endif
	  configEnablePti();
  }
  gecko_init(pconfig);
#if defined(EMU_VSCALE_PRESENT)
  EMU_VScaleEM01(read_data.em01vscale,1);
#endif
#if defined(EMU_VSCALE_PRESENT)
  EMU->CTRL = (EMU->CTRL & ~_EMU_CTRL_EM23VSCALE_MASK) | ((uint32_t)read_data.em23vscale << _EMU_CTRL_EM23VSCALE_SHIFT);
#endif
  CMU_ClockEnable(cmuClock_GPIO,1);
  while (1) {
    struct gecko_cmd_packet* evt;
    if(notification_active) {
    	evt = gecko_peek_event();
    	if(NULL == evt) {
    		continue_notification();
    		continue;
    	}
    } else {
    	evt = gecko_wait_event();
    }
#ifdef DUMP
    dump_event(evt);
#endif
    switch (BGLIB_MSG_ID(evt->header)) {
      case gecko_evt_system_boot_id:
    	  read_data.reqTxPower = 0;
    	  read_data.interval = 160;
    	  read_data.adLen = 10;
    	  read_data.delay = 0;
    	  measurement_active = 0;
    	  power_settings.count = 0;
    	  for(int16 req = -300; req < 300; req++) {
    		  power_settings.values[power_settings.count] = gecko_cmd_system_set_tx_power(req)->set_power;
    		  if(!power_settings.count ||(power_settings.values[power_settings.count] != power_settings.values[power_settings.count-1])) {
    			  power_settings.count++;
    		  }
    	  }
    	  /* no break */
      case gecko_evt_le_connection_closed_id:
    	  notification_active = 0;
    	  if(ota_on_close) {
    		  gecko_cmd_dfu_reset(2);
    	  }
    	  if(reset_on_close) {
			  retention_write();
    		  gecko_cmd_system_reset(0);
    	  }
    	  set_power();
    	  set_ad_data();
    	  if(read_data.delay) {
    		  measurement_active = 1;
    		  switch(read_data.measurement_mode) {
    		  case DMM_EM1:
    			  SLEEP_SleepBlockBegin(sleepEM2);
    			  break;
    		  case DMM_EM2:
    			  break;
    		  case DMM_DTM_TX:
        		  gecko_cmd_test_dtm_tx(test_pkt_prbs9,255,read_data.dtm_channel,test_phy_1m);
        		  break;
    		  case DMM_RANDOM:
    	          gecko_cmd_le_gap_set_advertise_timing(1, read_data.interval, read_data.interval, 0, read_data.random_count);
    			  adv_random_power();
    			  break;
    		  case DMM_TEST:
    			  printf("Before clock swap\n");
#  if defined(_SILICON_LABS_32B_SERIES_1)
    			  CMU_OscillatorEnable(cmuOsc_HFRCO,1,1);
    			  CMU_OscillatorTuningSet(cmuOsc_HFRCO,cmuHFRCOFreq_38M0Hz);
    			  CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFRCO);
    			  CMU_OscillatorEnable(cmuOsc_HFXO, false, true);
#  endif
    			  printf("After clock swap\n");
    			  while(1);
    			  break;
    		  default:
        		  measurement_active = 0;
    			  read_data.delay = 0;
    			  break;
    		  }
    	  }
    	  if(read_data.delay) {
    		  gecko_cmd_hardware_set_soft_timer(read_data.delay,0,1);
    		  break;
    	  }
    	  /* no break */
      case gecko_evt_hardware_soft_timer_id:
    	  if(read_data.delay) {
    		  switch(read_data.measurement_mode) {
    		  case DMM_EM1:
    			  SLEEP_SleepBlockEnd(sleepEM2);
    			  break;
    		  case DMM_EM2:
    			  break;
    		  case DMM_DTM_TX:
    			  gecko_cmd_test_dtm_end();
        		  break;
    		  case DMM_RANDOM:
    			  measurement_active = 0;
    			  gecko_cmd_le_gap_stop_advertising(1);
    			  break;
    		  default:
    			  break;
    		  }
    		  read_data.delay = 0;
    	  }
    	  gecko_cmd_le_gap_set_advertise_timing(0, read_data.interval, read_data.interval, 0, 0);
    	  gecko_cmd_le_gap_start_advertising(0, le_gap_user_data, le_gap_connectable_scannable);
    	  break;

      case gecko_evt_le_connection_opened_id: /***************************************************************** le_connection_opened **/
#define ED evt->data.evt_le_connection_opened
    	  read_data.delay = 0;
    	  conn = ED.connection;
        break;
#undef ED

      case gecko_evt_le_connection_parameters_id: /********************************************************* le_connection_parameters **/
    #define ED evt->data.evt_le_connection_parameters
      read_data.connection_interval = ED.interval;
      break;
    #undef ED

      case gecko_evt_gatt_mtu_exchanged_id: /********************************************************************* gatt_mtu_exchanged **/
    #define ED evt->data.evt_gatt_mtu_exchanged
        mtu = ED.mtu;
        break;
    #undef ED

      case gecko_evt_gatt_server_user_write_request_id: /********************************************* gatt_server_user_write_request **/
#define ED evt->data.evt_gatt_server_user_write_request
    	  result = 0;
    	  if(gattdb_cmu_clockenable == ED.characteristic) {
    		  if(ED.value.len != (1+sizeof(CMU_Clock_TypeDef))) {
    			  result = bg_err_bt_unspecified_error;
#ifdef DUMP
    			  printf("bad size CMU_ClockEnable\n");
#endif
    		  } else {
    			  CMU_Clock_TypeDef clock;
    			  _Bool enable = ED.value.data[0];
    			  memcpy(&clock,&ED.value.data[1],sizeof(CMU_Clock_TypeDef));
    			  for(int i = 0; i < sizeof(clocks)/sizeof(clocks[0]); i++) {
    				  if(clocks[i].clock == clock) {
#ifdef DUMP
    					  printf("CMU_ClockEnable(%s,%d)",clocks[i].name,enable);
#endif
    	    			  CMU_ClockEnable(clock,enable);
    	    			  break;
    				  }
    			  }
    		  }
    	  }
    	  if(gattdb_peripheral_request == ED.characteristic) {
    		  if(ED.value.len != 4) {
    			  result = bg_err_bt_unspecified_error;
    		  } else {
    			  uint32_t address;
    			  memcpy(&address,&ED.value.data[0],4);
    			  switch(address) {
    			  case (uint32)CMU:
    				gecko_cmd_gatt_server_send_characteristic_notification(ED.connection,gattdb_device_data_data,get_cmu(),(uint8*)&peripheral_data);
    			  	break;
    			  case (uint32)USART0:
    			  case (uint32)USART1:
      				gecko_cmd_gatt_server_send_characteristic_notification(ED.connection,gattdb_device_data_data,get_usart((USART_TypeDef*)address),(uint8*)&peripheral_data);
      				break;
    			  default:
      				gecko_cmd_gatt_server_send_characteristic_notification(ED.connection,gattdb_device_data_data,0,NULL);
      			  	break;
    			  }
    		  }
    	  }
    	  if(gattdb_gpio_config == ED.characteristic) {
    		  if(4 == ED.value.len) {
    			  GPIO_PinModeSet(ED.value.data[0],ED.value.data[1],ED.value.data[2],ED.value.data[3]);
    		  } else {
    			  result = 1;
    		  }
    	  } else if(gattdb_general_control == ED.characteristic) {
    		  switch(ED.value.data[0]) {
    		  case 1:
    			  ota_on_close = 1;
    			  read_data.flags |= FLAGS_OTA_ON_CLOSE;
    			  break;
#ifdef _SILICON_LABS_32B_SERIES_2
    		  case 2:
    			  if(ED.value.data[1]) {
    				  EMU->CTRL |= EMU_CTRL_EM2DBGEN;
    			  } else {
    				  EMU->CTRL &= ~EMU_CTRL_EM2DBGEN;
    			  }
    			  break;
#endif
    		  case 3:
    			  switch(ED.value.data[1]) {
    			  case DMM_EM1:
    			  case DMM_EM2:
    			  case DMM_RANDOM:
    			  case DMM_DTM_TX:
    			  case DMM_TEST:
        			  read_data.measurement_mode = ED.value.data[1];
        			  memcpy(&read_data.delay,&ED.value.data[2],4);
        			  read_data.flags |= FLAGS_MEASUREMENT_ACTIVATED;
        			  break;
    			  default:
    				  result = bg_err_not_implemented;
    				  break;
    			  }
    			  break;

			  case 4:
				  switch(ED.value.data[1]) {
				  case 0:
					  if(read_data.pti != ED.value.data[2]) {
						  reset_on_close = 1;
						  read_data.pti = ED.value.data[2];
					  }
					  break;
				  case 1:
					  if(read_data.mx25_dp != ED.value.data[2]) {
						  reset_on_close = 1;
						  read_data.mx25_dp = ED.value.data[2];
					  }
					  break;
				  case 2:
#ifdef DCDC
#  ifdef DUMP
					  printf("EMU_DCDCModeSet(%d)\n",ED.value.data[2]);
#  endif
					  EMU_DCDCModeSet((EMU_DcdcMode_TypeDef)ED.value.data[2]);
					  read_data.dcdc_mode = ED.value.data[2];
#endif
					  break;
    			  default:
    				  result = bg_err_not_implemented;
    				  break;
				  }
				  break;

    		  case 5:
    			  read_data.interval = 0;
    			  for(int i = 0; i < 4; i++) {
    				  read_data.interval += ED.value.data[1+i] << (8*i);
    			  }
    			  break;
    		  case 6:
    			  memcpy(&read_data.reqTxPower,&ED.value.data[1],2);
    			  gecko_cmd_system_halt(1);
    			  set_power();
    			  gecko_cmd_system_halt(0);
    			  break;
    		  case 7:
    			  read_data.adLen = ED.value.data[1];
    			  break;
    		  case 8:
    			  read_data.pa_mode = ED.value.data[1];
    			  reset_on_close = 1;
    			  break;
    		  case 9:
    			  read_data.pa_input = ED.value.data[1];
    			  reset_on_close = 1;
    			  break;
    		  case 10:
    			  read_data.sleep_clock_accuracy = ED.value.data[1] | (ED.value.data[2] << 8);
    			  reset_on_close = 1;
    			  break;
    		  case 11:
    			  read_data.dtm_channel = ED.value.data[1];
    			  break;
    		  case 12:
    			  CMU_ClockEnable(cmuClock_GPIO,1);
    			  break;
    		  }
    	  }
    	  if(0x12 == ED.att_opcode) {
    		  gecko_cmd_gatt_server_send_user_write_response(ED.connection,ED.characteristic,result);
    	  }
        break;
#undef ED

      case gecko_evt_gatt_server_user_read_request_id: /*********************************************** gatt_server_user_read_request **/
#define ED evt->data.evt_gatt_server_user_read_request
    	  switch(ED.characteristic) {
    	  case gattdb_gpio_pin_count:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_gpio_pin_count,0,sizeof(gpio_pin_count),(uint8*)&gpio_pin_count);
    		  break;
    	  case gattdb_gpio_config:
    		  for(int port = 0; port < GPIO_PORT_COUNT; port++) {
    			  gpio_config[port].model = GPIO->P[port].MODEL;
    			  gpio_config[port].modeh = GPIO->P[port].MODEH;
    			  gpio_config[port].dout = GPIO->P[port].DOUT;
    		  }
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_gpio_config,0,sizeof(gpio_config),(uint8*)&gpio_config);
    		  break;
    	  case gattdb_general_control:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_general_control,0,sizeof(read_data),(uint8*)&read_data);
    		  read_data.flags = 0;
    		  break;
    	  case gattdb_power_settings:
    		  send_power_settings(ED.connection,ED.characteristic,ED.offset);
      		  break;
    	  case gattdb_emu_ctrl:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_emu_ctrl,0,4,(uint8*)&EMU->CTRL);
			  break;
#ifndef RMU
    	  case gattdb_emu_rstctrl:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_emu_rstctrl,0,4,(uint8*)&EMU->RSTCTRL);
			  break;
#endif
#ifdef DCDC
    	  case gattdb_dcdc:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_dcdc,0,get_dcdc(),(uint8*)&peripheral_data);
			  break;
#endif
    	  case gattdb_cmu:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_cmu,0,get_cmu(),(uint8*)&peripheral_data);
			  break;

    	  default:
    	        gecko_cmd_gatt_server_send_user_read_response(ED.connection,ED.characteristic,1,0,0);
    	  }
        break;
#undef ED

    	  case gecko_evt_le_gap_adv_timeout_id: /********************************************************************* le_gap_adv_timeout **/
    	#define ED evt->data.evt_le_gap_adv_timeout
    	    if(measurement_active) adv_random_power();
    	    break;
    	#undef ED

    	  case gecko_evt_gatt_server_characteristic_status_id: /*************************************** gatt_server_characteristic_status **/
#define ED evt->data.evt_gatt_server_characteristic_status
    		  switch(ED.status_flags) {
    		  case 1:
    			  switch(ED.characteristic) {
    			  case gattdb_device_data_data:
    				  if(1 == ED.client_config_flags) {
    				  }
    				  break;
    			  case gattdb_device_data_status:
    				  if(2 == ED.client_config_flags) {
    					  start_notification(N_NONE + 1);
    				  }
    				  break;
    			  }
    			  break;
			  case 2:
    			  switch(ED.characteristic) {
    			  case gattdb_device_data_status:
    				  if(N_WAIT_DONE != state) {
    					  state++;
    					  notification_active = 1;
    				  }
    			  }
    			  break;
    		  }
    		  break;
#undef ED

	  case gecko_evt_system_external_signal_id:
		  start_notification(state);
		  break;

      default:
        break;
    }
  }
}
