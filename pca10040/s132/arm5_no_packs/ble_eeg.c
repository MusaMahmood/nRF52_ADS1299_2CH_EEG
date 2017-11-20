/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ble_eeg.h"
#include "ads1299-x.h"
#include "app_error.h"
#include "app_util.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include <string.h>

#define MAX_LEN_BLE_PACKET_BYTES 246 //20*3bytes																						 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */
                                     //20*3bytes																						 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */

static void on_write(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt) {
  ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if ((p_evt_write->handle == p_eeg->ads1299_config_char_handles.value_handle) &&
      (p_evt_write->len >= 1) && (p_eeg->eeg_config_handler != NULL)) {
    p_eeg->eeg_config_handler(p_ble_evt->evt.gap_evt.conn_handle, p_eeg, &p_evt_write->data[0]);
  }
}

void ble_eeg_on_ble_evt(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt) {
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    p_eeg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    p_eeg->conn_handle = BLE_CONN_HANDLE_INVALID;
    break;
  //    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
  //
  //      break;
  case BLE_GATTS_EVT_WRITE:
    on_write(p_eeg, p_ble_evt);
    break;
  default:
    break;
  }
}

static uint32_t eeg_ads1299_config_char_add(ble_eeg_t *p_eeg, const ble_eeg_init_t *p_eeg_init) {
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;
  ble_gatts_attr_md_t cccd_md;
  memset(&char_md, 0, sizeof(char_md));
  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.char_props.read = 1;
  char_md.char_props.write = 1;
  char_md.char_props.notify = 1;
  //  char_md.p_char_user_desc = NULL;
  //  char_md.p_char_pf = NULL;
  //  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  //  char_md.p_sccd_md = NULL;
  BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_EEG_CONFIG);
  //  ble_uuid.type = p_eeg->uuid_type;
  //  ble_uuid.uuid = BLE_UUID_EEG_CONFIG;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = 1;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = 23;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = 23;
  attr_char_value.p_value = p_eeg->ads1299_current_configuration;

  return sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->ads1299_config_char_handles);
}

static uint32_t eeg_ch1_char_add(ble_eeg_t *p_eeg) {
  uint32_t err_code = 0;
  ble_uuid_t char_uuid;
  uint8_t encoded_initial_eeg[EEG_PACKET_LENGTH];
  memset(encoded_initial_eeg, 0, EEG_PACKET_LENGTH);
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH1_CHAR);

  ble_gatts_char_md_t char_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = EEG_PACKET_LENGTH;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = EEG_PACKET_LENGTH;
  attr_char_value.p_value = encoded_initial_eeg;
  err_code = sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->eeg_ch1_handles);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

static uint32_t eeg_ch2_char_add(ble_eeg_t *p_eeg) {
  uint32_t err_code = 0;
  ble_uuid_t char_uuid;
  uint8_t encoded_initial_eeg[EEG_PACKET_LENGTH];
  memset(encoded_initial_eeg, 0, EEG_PACKET_LENGTH);
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH2_CHAR);

  ble_gatts_char_md_t char_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = EEG_PACKET_LENGTH;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = EEG_PACKET_LENGTH;
  attr_char_value.p_value = encoded_initial_eeg;
  err_code = sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->eeg_ch2_handles);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

static uint32_t eeg_ch3_char_add(ble_eeg_t *p_eeg) {
  uint32_t err_code = 0;
  ble_uuid_t char_uuid;
  uint8_t encoded_initial_eeg[EEG_PACKET_LENGTH];
  memset(encoded_initial_eeg, 0, EEG_PACKET_LENGTH);
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH3_CHAR);

  ble_gatts_char_md_t char_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = EEG_PACKET_LENGTH;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = EEG_PACKET_LENGTH;
  attr_char_value.p_value = encoded_initial_eeg;
  err_code = sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->eeg_ch3_handles);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

static uint32_t eeg_ch4_char_add(ble_eeg_t *p_eeg) {
  uint32_t err_code = 0;
  ble_uuid_t char_uuid;
  uint8_t encoded_initial_eeg[EEG_PACKET_LENGTH];
  memset(encoded_initial_eeg, 0, EEG_PACKET_LENGTH);
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH4_CHAR);

  ble_gatts_char_md_t char_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = EEG_PACKET_LENGTH;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = EEG_PACKET_LENGTH;
  attr_char_value.p_value = encoded_initial_eeg;
  err_code = sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->eeg_ch4_handles);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_eeg_service_init(ble_eeg_t *p_eeg, const ble_eeg_init_t *p_eeg_init) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions
  uint16_t service_handle;
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = {BMS_UUID_BASE};
  //Initialize service structure:
  p_eeg->eeg_config_handler = p_eeg_init->eeg_config_handler;

  err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_eeg->uuid_type));
  APP_ERROR_CHECK(err_code);

  service_uuid.type = p_eeg->uuid_type;
  service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
  APP_ERROR_CHECK(err_code);

  //Add Characteristics:
  err_code = eeg_ads1299_config_char_add(p_eeg, p_eeg_init);
  APP_ERROR_CHECK(err_code);

  eeg_ch1_char_add(p_eeg);
  eeg_ch2_char_add(p_eeg);
  eeg_ch3_char_add(p_eeg);
  eeg_ch4_char_add(p_eeg);
}

#if defined(ADS1299)

void ble_eeg_update_configuration(ble_eeg_t *p_eeg, bool notify) {
  uint32_t err_code;
  uint16_t attr_handle;
  uint16_t hvx_len = 23;
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID) {
    ble_gatts_value_t value;
    value.len = hvx_len;
    value.offset = 0;
    value.p_value = p_eeg->ads1299_current_configuration;

    err_code = sd_ble_gatts_value_set(p_eeg->conn_handle, p_eeg->ads1299_config_char_handles.value_handle, &value);
    NRF_LOG_INFO("err_code ble_eeg_update::config 0x%x \n", err_code);
    if (notify) {
      ble_gatts_hvx_params_t const hvx_params = {
          .handle = p_eeg->ads1299_config_char_handles.value_handle,
          .type = BLE_GATT_HVX_NOTIFICATION,
          .offset = 0,
          .p_len = &hvx_len,
          .p_data = p_eeg->ads1299_current_configuration,
      };
      err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
    }
  }
}

void ble_eeg_update_1ch_v2(ble_eeg_t *p_eeg) {
  uint32_t err_code;
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_eeg->eeg_ch1_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_eeg->eeg_ch1_buffer,
    };
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }

  if (err_code == NRF_ERROR_RESOURCES) {
    NRF_LOG_INFO("sd_ble_gatts_hvx() ERR/RES: 0x%x\r\n", err_code);
  }
}

void ble_eeg_update_4ch(ble_eeg_t *p_eeg) {
  //packet 1:
  uint32_t err_code;
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID && p_eeg->ads1299_current_configuration[4] != 0xE1) {
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_eeg->eeg_ch1_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_eeg->eeg_ch1_buffer,
    };
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }
  if (err_code == NRF_ERROR_RESOURCES) {
    NRF_LOG_INFO("sd_ble_gatts_hvx() ERR/RES: 0x%x\r\n", err_code);
  }
  //Packet 2
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID && p_eeg->ads1299_current_configuration[5] != 0xE1) {
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_eeg->eeg_ch2_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_eeg->eeg_ch2_buffer,
    };
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }
  if (err_code == NRF_ERROR_RESOURCES) {
    NRF_LOG_INFO("sd_ble_gatts_hvx() ERR/RES: 0x%x\r\n", err_code);
  }

  //Packet 3
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID && p_eeg->ads1299_current_configuration[6] != 0xE1) {
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_eeg->eeg_ch3_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_eeg->eeg_ch3_buffer,
    };
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }
  if (err_code == NRF_ERROR_RESOURCES) {
    NRF_LOG_INFO("sd_ble_gatts_hvx() ERR/RES: 0x%x\r\n", err_code);
  }

  //Packet 4
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID && p_eeg->ads1299_current_configuration[7] != 0xE1) {
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_eeg->eeg_ch4_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_eeg->eeg_ch4_buffer,
    };
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }
  if (err_code == NRF_ERROR_RESOURCES) {
    NRF_LOG_INFO("sd_ble_gatts_hvx() ERR/RES: 0x%x\r\n", err_code);
  }
}

#endif //(defined(ADS1299)