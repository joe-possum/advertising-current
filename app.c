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
#include "common.h"
#include "struct.h"
#include <stdio.h>
#include "em_device.h"
#include "em_rmu.h"
#include "em_cmu.h"
#include "em_gpio.h"

uint8 adData[31];
uint8 notify = 0; /* perform notifications of power settings */

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

const uint32 GPIO_PORT_COUNT = (sizeof(GPIO->P)/sizeof(GPIO_PORT_TypeDef));
struct gpio gpio_config[12u];

struct manData {
	uint16 id;
} manData = { .id = 0xffff, };

struct read_data read_data;
struct powers power;

void set_power(void) {
	struct gecko_msg_system_set_tx_power_rsp_t *resp;
	resp = gecko_cmd_system_set_tx_power(read_data.reqTxPower);
	read_data.TxPower = resp->set_power;
}

void set_ad_data(void) {
	uint8 len = 0;
	len += ad_flags(&adData[len],6);
	if(read_data.adLen > (len+7)) {
		len += ad_name(&adData[len],"4182A");
	}
	if(read_data.adLen > (len + 4)) {
		ad_manufacturer(&adData[len],read_data.adLen-len-2,(uint8*)&manData);
	}
	gecko_cmd_le_gap_bt5_set_adv_data(0,0,read_data.adLen,adData);
}

int16 req, set, prev;
uint8 pending = 0, conn, reset_on_close, ota_on_close;
uint16 result;

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
	}
	if(0 == chksum) {
	  read_data.pa_mode = RET[1].REG & 0xff;
	  read_data.pa_input = (RET[1].REG >> 8) & 0xff;
	  read_data.sleep_clock_accuracy = (RET[1].REG >> 16) & 0xff;
	} else {
		read_data.flags |= 2;
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
 	RET[1].REG = read_data.pa_mode | (read_data.pa_input << 8) | (read_data.sleep_clock_accuracy << 16);
	for(int i = 1; i < N_RET; i++) {
		chksum += RET[i].REG;
	}
	RET[0].REG = -chksum;
	read_data.flags |= 1;
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
#ifndef SOFTWARE_RESET
#error SOFTWARE_RESET not defined
#endif
/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
  read_data.reason = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
  read_data.pa_mode = pconfig->pa.pa_mode;
  read_data.pa_input = pconfig->pa.input;
  read_data.sleep_clock_accuracy = pconfig->bluetooth.sleep_clock_accuracy;
  read_data.flags = 0;
  if(read_data.reason & SOFTWARE_RESET) { /* software reset */
	  retention_read();
  }
  pconfig->pa.pa_mode = read_data.pa_mode;
  reset_on_close = 0;
  ota_on_close = 0;
  gecko_init(pconfig);
  CMU_ClockEnable(cmuClock_GPIO,1);
  while (1) {
    struct gecko_cmd_packet* evt;
	evt = gecko_wait_event();
    switch (BGLIB_MSG_ID(evt->header)) {
      case gecko_evt_system_boot_id:
    	  read_data.reqTxPower = 0;
    	  read_data.interval = 160;
    	  read_data.adLen = 10;
    	  read_data.delay = 0;
    	  power.count = 0;
    	  for(int16 req = -300; req < 300; req++) {
    		  power.values[power.count] = gecko_cmd_system_set_tx_power(req)->set_power;
    		  if(!power.count ||(power.values[power.count] != power.values[power.count-1])) {
    			  power.count++;
    		  }
    	  }
    	  /* no break */
      case gecko_evt_le_connection_closed_id:
    	  if(ota_on_close) {
    		  gecko_cmd_dfu_reset(2);
    	  }
    	  if(reset_on_close) {
    		  gecko_cmd_system_reset(0);
    	  }
    	  set_power();
    	  set_ad_data();
          gecko_cmd_le_gap_set_advertise_timing(0, read_data.interval, read_data.interval, 0, 0);
    	  if(read_data.delay) {
    		  gecko_cmd_hardware_set_soft_timer(read_data.delay,0,1);
    		  break;
    	  }
    	  /* no break */
      case gecko_evt_hardware_soft_timer_id:
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

      case gecko_evt_gatt_server_user_write_request_id: /********************************************* gatt_server_user_write_request **/
#define ED evt->data.evt_gatt_server_user_write_request
    	  result = 0;
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
    			  read_data.flags |= 4;
    			  break;
    		  case 2:
    			  if(ED.value.data[1]) {
    				  EMU->CTRL |= EMU_CTRL_EM2DBGEN;
    			  } else {
    				  EMU->CTRL &= ~EMU_CTRL_EM2DBGEN;
    			  }
    			  break;
    		  case 4:
    			  memcpy(&read_data.delay,&ED.value.data[1],4);
    			  read_data.flags |= 8;
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
    			  retention_write();
    			  reset_on_close = 1;
    			  break;
    		  case 9:
    			  read_data.pa_input = ED.value.data[1];
    			  retention_write();
    			  reset_on_close = 1;
    			  break;
    		  case 10:
    			  read_data.sleep_clock_accuracy = ED.value.data[1] | (ED.value.data[2] << 8);
    			  retention_write();
    			  reset_on_close = 1;
    			  break;
    		  }
    	  }
		  gecko_cmd_gatt_server_send_user_write_response(ED.connection,ED.characteristic,result);
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
      		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_power_settings,0,2*power.count,(uint8*)&power.values[0]);
      		  break;
    	  case gattdb_emu_ctrl:
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_emu_ctrl,0,4,(uint8*)&EMU->CTRL);
			  break;
    	  default:
    	        gecko_cmd_gatt_server_send_user_read_response(ED.connection,ED.characteristic,1,0,0);
    	  }
        break;
#undef ED

      default:
        break;
    }
  }
}
