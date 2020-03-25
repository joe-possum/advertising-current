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

uint16 delay;
uint8 adData[31];

struct manData {
	uint16 id;
} manData = { .id = 0xffff, };

struct read_data read_data;

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

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
  gecko_init(pconfig);
  while (1) {
    struct gecko_cmd_packet* evt;
    evt = gecko_wait_event();
    switch (BGLIB_MSG_ID(evt->header)) {
      case gecko_evt_system_boot_id:
    	  read_data.reqTxPower = 0;
    	  read_data.interval = 160;
    	  read_data.adLen = 0;
    	  /* no break */
      case gecko_evt_le_connection_closed_id:
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
        break;
#undef ED

      case gecko_evt_gatt_server_user_write_request_id: /********************************************* gatt_server_user_write_request **/
#define ED evt->data.evt_gatt_server_user_write_request
    	  if(gattdb_gpio == ED.characteristic) {
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
    		  }
    	  }
		  gecko_cmd_gatt_server_send_user_write_response(ED.connection,ED.characteristic,0);
        break;
#undef ED

      case gecko_evt_gatt_server_user_read_request_id: /*********************************************** gatt_server_user_read_request **/
#define ED evt->data.evt_gatt_server_user_read_request
    	  if(gattdb_read == ED.characteristic) {
    		  for(int port = gpioPortA; port <= gpioPortD; port++) {
    			  read_data.gpio[port].model = GPIO->P[port].MODEL;
    			  read_data.gpio[port].modeh = GPIO->P[port].MODEH;
    			  read_data.gpio[port].dout = GPIO->P[port].DOUT;
    		  }
    		  gecko_cmd_gatt_server_send_user_read_response(ED.connection,gattdb_gpio,0,sizeof(read_data),(uint8*)&read_data);
    	  } else {
    	        gecko_cmd_gatt_server_send_user_read_response(ED.connection,ED.characteristic,1,0,0);
    	  }
        break;
#undef ED

      default:
        break;
    }
  }
}
