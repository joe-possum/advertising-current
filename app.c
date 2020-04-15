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

uint16 delay;
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

const uint32 GPIO_PORT_COUNT = (sizeof(GPIO->P)/sizeof(GPIO_P_TypeDef));
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

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
  read_data.reason = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
  CMU_ClockEnable(cmuClock_BURAM,1);
#endif
#if defined(_SILICON_LABS_32B_SERIES_1)
  if(read_data.reason & 1) {
	  RTCC->RET[0].REG = 0;
  } else {
	  read_data.pa_mode = RTCC->RET[0].REG;
	  pconfig->pa.pa_mode = read_data.pa_mode;
  }
#endif
#if defined(_SILICON_LABS_32B_SERIES_2)
  if(read_data.reason & 1) {
	  BURAM->RET[0].REG = 0;
  } else {
	  read_data.pa_mode = BURAM->RET[0].REG;
	  pconfig->pa.pa_mode = read_data.pa_mode;
  }
#endif
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
  CMU_ClockEnable(cmuClock_BURAM,0);
#endif
  reset_on_close = 0;
  ota_on_close = 0;
  gecko_init(pconfig);
  CMU_ClockEnable(cmuClock_GPIO,1);
  while (1) {
    struct gecko_cmd_packet* evt;
	evt = gecko_wait_event();
#ifdef DUMP
    dump_event(evt);
#endif
    switch (BGLIB_MSG_ID(evt->header)) {
      case gecko_evt_system_boot_id:
    	  read_data.reqTxPower = 0;
    	  read_data.interval = 160;
    	  read_data.adLen = 10;
    	  delay = 0;
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
    		  gecko_cmd_system_reset(1);
    	  }
    	  if(reset_on_close) {
    		  gecko_cmd_system_reset(0);
    	  }
    	  set_power();
    	  set_ad_data();
          gecko_cmd_le_gap_set_advertise_timing(0, read_data.interval, read_data.interval, 0, 0);
    	  if(delay) {
    		  gecko_cmd_hardware_set_soft_timer(delay,0,1);
    		  break;
    	  }
    	  /* no break */
      case gecko_evt_hardware_soft_timer_id:
        gecko_cmd_le_gap_start_advertising(0, le_gap_user_data, le_gap_connectable_scannable);
        break;

      case gecko_evt_le_connection_opened_id: /***************************************************************** le_connection_opened **/
#define ED evt->data.evt_le_connection_opened
    	  delay = 0;
    	  conn = ED.connection;
        break;
#undef ED

      case gecko_evt_gatt_server_user_write_request_id: /********************************************* gatt_server_user_write_request **/
#define ED evt->data.evt_gatt_server_user_write_request
    	  result = 0;
    	  if(gattdb_gpio_config == ED.characteristic) {
    		  if(4 == ED.value.len) {
    			  printf("before: MODEL: %08x, MODEH: %08x, DOUT: %08x\n",GPIO->P[ED.value.data[0]].MODEL,GPIO->P[ED.value.data[0]].MODEH,GPIO->P[ED.value.data[0]].DOUT);
    			  GPIO_PinModeSet(ED.value.data[0],ED.value.data[1],ED.value.data[2],ED.value.data[3]);
    			  printf(" after: MODEL: %08x, MODEH: %08x, DOUT: %08x\n",GPIO->P[ED.value.data[0]].MODEL,GPIO->P[ED.value.data[0]].MODEH,GPIO->P[ED.value.data[0]].DOUT);
    			  printf("GPIO_PinModeSet(%d,%d,%d,%d)",ED.value.data[0],ED.value.data[1],ED.value.data[2],ED.value.data[3]);
    		  } else {
    			  result = 1;
    		  }
    	  } else if(gattdb_gpio == ED.characteristic) {
    		  switch(ED.value.data[0]) {
    		  case 0 ... 3:
    			  GPIO_PinModeSet(ED.value.data[0],ED.value.data[1],ED.value.data[2],ED.value.data[3]);
    		  	  break;
    		  case 4:
    			  delay = ED.value.data[1] + (ED.value.data[2] << 8);
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
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    			  CMU_ClockEnable(cmuClock_BURAM,1);
#endif
#if defined(_SILICON_LABS_32B_SERIES_1)
    			  RTCC->RET[0].REG = ED.value.data[1];
#endif
#if defined(_SILICON_LABS_32B_SERIES_2)
    			  BURAM->RET[0].REG = ED.value.data[1];
#endif
#if defined(_SILICON_LABS_32B_SERIES_2_CONFIG_2)
    			  CMU_ClockEnable(cmuClock_BURAM,0);
#endif
    			  reset_on_close = 1;
    			  break;
    		  case 9:
    			  ota_on_close = 1;
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
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_gpio,0,sizeof(gpio_config),(uint8*)&gpio_config);
    		  break;
    	  case gattdb_read:
    		  for(int port = gpioPortA; port <= gpioPortD; port++) {
    			  read_data.gpio[port].model = GPIO->P[port].MODEL;
    			  read_data.gpio[port].modeh = GPIO->P[port].MODEH;
    			  read_data.gpio[port].dout = GPIO->P[port].DOUT;
    		  }
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_gpio,0,sizeof(read_data),(uint8*)&read_data);
    		  break;
    	  case gattdb_power:
      		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_power,0,2*power.count,(uint8*)&power.values[0]);
      		  break;
    	  default:
    	        gecko_cmd_gatt_server_send_user_read_response(ED.connection,ED.characteristic,1,0,0);
    	  }
        break;
#undef ED

      case gecko_evt_gatt_server_characteristic_status_id: /*************************************** gatt_server_characteristic_status **/
#define ED evt->data.evt_gatt_server_characteristic_status
    	  if(ED.characteristic == gattdb_notify) {
    		  if(1 == ED.status_flags) {
    			  if(2 == ED.client_config_flags)
    	    		  notify = 1;
    	    		  req = -300;
    	    		  prev = -301;
    	    		  gecko_external_signal(1);
    		  } else if(2 == ED.status_flags) {
    			  pending = 0;
	    		  gecko_external_signal(1);
    		  }
    	  }
        break;
#undef ED

      case gecko_evt_system_external_signal_id: /************************************************************* system_external_signal **/
    #define ED evt->data.evt_system_external_signal
    	  if(pending) break;
		  gecko_cmd_system_halt(1);
		  while(1) {
			  if(req > 200) {
					  break;
			  }
			  set = gecko_cmd_system_set_tx_power(req)->set_power;
			  req++;
			  if(set != prev) {
					  prev = set;
					  pending = 1;
					  break;
			  }
		  }
		  gecko_cmd_system_halt(0);
          if(pending) {
    		  gecko_cmd_gatt_server_send_characteristic_notification(conn,gattdb_notify,2,(uint8*)&set);
          }
        break;
    #undef ED

      default:
        break;
    }
  }
}
