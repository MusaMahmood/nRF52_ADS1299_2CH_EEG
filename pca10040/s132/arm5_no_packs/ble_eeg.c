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
  default:
    break;
  }
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

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_eeg_service_init(ble_eeg_t *p_eeg) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions
  uint16_t service_handle;
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = {BMS_UUID_BASE};

  err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_eeg->uuid_type));
  APP_ERROR_CHECK(err_code);

  service_uuid.type = p_eeg->uuid_type;
  service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
  APP_ERROR_CHECK(err_code);

  //Add Characteristic:
  eeg_ch1_char_add(p_eeg);
  eeg_ch2_char_add(p_eeg);
}

#if defined(ADS1299)

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

void ble_eeg_update_2ch(ble_eeg_t *p_eeg) {
  //packet 1:
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
  //Packet 2
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID) {
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
}

#endif //(defined(ADS1299)