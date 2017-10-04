/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#if defined(APP_TIMER_SAMPLING)
#include "app_timer.h"
#endif
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "fds.h"
#include "fstorage.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_gpio.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#if defined(ADS1299)
//TODO: ADS1299 Stuff:
#include "ads1299-x.h"
#include "ble_dis.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#define DEVICE_MODEL_NUMBERSTR "Version 3.1"
#define DEVICE_FIRMWARE_STRING "Version 13.1.0"
ble_eeg_t m_eeg;
static bool m_connected = false;
#define SPI_SCLK_WRITE_REG 2
#define SPI_SCLK_SAMPLING 8
#endif

#if defined(MPU9250) || defined(MPU9255) //mpu_send_timeout_handler
#include "app_mpu.h"
#include "nrf_drv_twi.h"
APP_TIMER_DEF(m_mpu_send_timer_id);
#define TICKS_MPU_SAMPLING_INTERVAL APP_TIMER_TICKS(32)
#endif

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
#define TICKS_SAMPLING_INTERVAL APP_TIMER_TICKS(1000)
APP_TIMER_DEF(m_sampling_timer_id);
static uint16_t m_samples;
#endif
#define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 /**< Reply when unsupported features are requested. */

#define DEVICE_NAME "nRF52-EEG-4k"    //"nRF52_EEG"         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Potato Labs" /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300            /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS 180  /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(16, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define CONN_CFG_TAG 1 /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define HVN_TX_QUEUE_SIZE 12

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static nrf_ble_gatt_t m_gatt;                            /**< GATT module instance. */

static ble_uuid_t m_adv_uuids[] =
    {
        {BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static void advertising_start(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
static void m_sampling_timeout_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO("SAMPLE RATE = %dHz \r\n", m_samples);
#endif
  m_samples = 0;
#endif
}
#endif
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
static void mpu_send_timeout_handler(void *p_context) {
  //DEPENDS ON SAMPLING RATE
  mpu_read_accel_array(&m_mpu);
  mpu_read_gyro_array(&m_mpu);
  if (m_mpu.mpu_count == 240) {
    m_mpu.mpu_count = 0;
    ble_mpu_combined_update_v2(&m_mpu);
  }
}
#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  // Initialize timer module..
  //Create timers
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
  err_code = app_timer_create(&m_mpu_send_timer_id, APP_TIMER_MODE_REPEATED, mpu_send_timeout_handler);
  APP_ERROR_CHECK(err_code);
#endif

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
  err_code = app_timer_create(&m_sampling_timer_id, APP_TIMER_MODE_REPEATED, m_sampling_timeout_handler);
  APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {
  ble_eeg_service_init(&m_eeg);
  /**@Device Information Service:*/
  uint32_t err_code;
  ble_dis_init_t dis_init;
  memset(&dis_init, 0, sizeof(dis_init));
  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
  ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_MODEL_NUMBERSTR);
  ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_STRING);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
  err_code = ble_dis_init(&dis_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void) {
  /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
  ret_code_t err_code;
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
  err_code = app_timer_start(m_sampling_timer_id, TICKS_SAMPLING_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
#endif

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
  err_code = app_timer_start(m_mpu_send_timer_id, TICKS_MPU_SAMPLING_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;
  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.\r\n");
    APP_ERROR_CHECK(err_code);
//#if defined(BOARD_EXG_V3)
#if LEDS_ENABLE == 1
    nrf_gpio_pin_clear(LED_2); // Green
    nrf_gpio_pin_set(LED_1);   //Blue
#endif
    break;

  case BLE_ADV_EVT_IDLE:
    //sleep_mode_enter();
    break;

  default:
    break;
  }
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t *p_ble_evt) {
  ret_code_t err_code = NRF_SUCCESS;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.\r\n");
    m_connected = false;
#if defined(ADS1299)
    ads1299_standby();
#endif
#if LEDS_ENABLE == 1
    nrf_gpio_pin_clear(LED_2); // Green
    nrf_gpio_pin_set(LED_1);   //Blue
#endif
    advertising_start();
    break; // BLE_GAP_EVT_DISCONNECTED

  case BLE_GAP_EVT_CONNECTED:
    ads_spi_uninit();
    ads_spi_init_with_sample_freq(SPI_SCLK_SAMPLING);
#if defined(ADS1299)
    ads1299_wake();
#endif
//#if defined(BOARD_EXG_V3)
#if LEDS_ENABLE == 1
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_pin_clear(LED_1);
#endif
    NRF_LOG_INFO("Connected.\r\n");
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    m_connected = true;
    break; // BLE_GAP_EVT_CONNECTED

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    m_connected = false;
    APP_ERROR_CHECK(err_code);
    break; // BLE_GATTC_EVT_TIMEOUT

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    m_connected = false;
    break; // BLE_GATTS_EVT_TIMEOUT

  case BLE_EVT_USER_MEM_REQUEST:
    err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
    APP_ERROR_CHECK(err_code);
    break; // BLE_EVT_USER_MEM_REQUEST

  case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
    ble_gatts_evt_rw_authorize_request_t req;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    req = p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID) {
      if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
          (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
          (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
        if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
          auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
        } else {
          auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
        }
        auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
        err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
            &auth_reply);
        APP_ERROR_CHECK(err_code);
      }
    }
  } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt) {
  /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
  ble_conn_state_on_ble_evt(p_ble_evt);
  ble_conn_params_on_ble_evt(p_ble_evt);
  on_ble_evt(p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
  nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);

  ble_eeg_on_ble_evt(&m_eeg, p_ble_evt);
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
  ble_mpu_on_ble_evt(&m_mpu, p_ble_evt);
#endif
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
  // Dispatch the system event to the fstorage module, where it will be
  // dispatched to the Flash Data Storage (FDS) module.
  fs_sys_event_handler(sys_evt);

  // Dispatch to the Advertising module last, since it will check if there are any
  // pending flash operations in fstorage. Let fstorage process system events first,
  // so that it can report correctly to the Advertising module.
  ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

  // Initialize the SoftDevice handler module.
  SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = softdevice_app_ram_start_get(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Overwrite some of the default configurations for the BLE stack.
  ble_cfg_t ble_cfg;

  // Configure number of custom UUIDS.
  memset(&ble_cfg, 0, sizeof(ble_cfg));
  ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1; //2?
  err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);

  // Configure the maximum number of connections.
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.gap_cfg.role_count_cfg.periph_role_count = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT; //is 1
  ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
  ble_cfg.gap_cfg.role_count_cfg.central_sec_count = 0;
  err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);
  ///*
  // Configure the max ATT MTU?
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag = CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);
  ///*
  // Configure the maximum event length
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag = CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
  ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = BLE_GAP_CONN_COUNT_DEFAULT;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);

  // Configure the number of packets per connection event:
  memset(&ble_cfg, 0x00, sizeof(ble_cfg));
  ble_cfg.conn_cfg.conn_cfg_tag = CONN_CFG_TAG;
  ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = HVN_TX_QUEUE_SIZE;
  err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
  APP_ERROR_CHECK(err_code);

  //*/
  // Enable BLE stack.
  err_code = softdevice_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register with the SoftDevice handler module for BLE events.
  err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
  APP_ERROR_CHECK(err_code);

  // Register with the SoftDevice handler module for BLE events.
  err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
  APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code = 0;

  NRF_LOG_INFO("Erase bonds!\r\n");

  //  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advdata_t advdata;
  ble_adv_modes_config_t options;

  // Build advertising data struct to pass into @ref ble_advertising_init.
  memset(&advdata, 0, sizeof(advdata));

  advdata.name_type = BLE_ADVDATA_FULL_NAME;
  advdata.include_appearance = true;
  advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  advdata.uuids_complete.p_uuids = m_adv_uuids;

  memset(&options, 0, sizeof(options));
  options.ble_adv_fast_enabled = true;
  options.ble_adv_fast_interval = APP_ADV_INTERVAL;
  options.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS;

  err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void) {
  ret_code_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

///*
static void advertising_start(void) {
  ble_gap_adv_params_t const adv_params =
      {
          .type = BLE_GAP_ADV_TYPE_ADV_IND,
          .p_peer_addr = NULL,
          .fp = BLE_GAP_ADV_FP_ANY,
          .interval = 300,
          .timeout = 0,
      };

  NRF_LOG_INFO("Starting advertising.\r\n");

  ret_code_t err_code = sd_ble_gap_adv_start(&adv_params, CONN_CFG_TAG);
  APP_ERROR_CHECK(err_code);
}

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
///*
//MPU9250;MPU9255
void mpu_setup(void) {
  ret_code_t ret_code;
  // Initiate MPU driver
  ret_code = mpu_init();
  APP_ERROR_CHECK(ret_code); // Check for errors in return value

  // Setup and configure the MPU with intial values
  mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
  p_mpu_config.smplrt_div = 19;                     // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
  p_mpu_config.accel_config.afs_sel = AFS_16G;      // Set accelerometer full scale range to 2G
  ret_code = mpu_config(&p_mpu_config);             // Configure the MPU with above values
  APP_ERROR_CHECK(ret_code);                        // Check for errors in return value
}
//*/
#endif

#if defined(ADS1299)

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  UNUSED_PARAMETER(pin);
  UNUSED_PARAMETER(action);
  if (m_connected) {
    get_eeg_voltage_array_2ch(&m_eeg);
  }
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
  m_samples += 1;
#endif
  if (m_eeg.eeg_ch1_count == EEG_PACKET_LENGTH) {
    m_eeg.eeg_ch1_count = 0;
    ble_eeg_update_2ch(&m_eeg);
  }
}

static void ads1299_gpio_init(void) {
#if defined(BOARD_PCA10040) && LEDS_ENABLE == 1
  nrf_gpio_cfg_output(LED_1);
  nrf_gpio_cfg_output(LED_2);
  nrf_gpio_cfg_output(LED_3);
  nrf_gpio_cfg_output(LED_4);
  nrf_gpio_pin_set(LED_3);
  nrf_gpio_pin_set(LED_4);
#endif

#if defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10028) | defined(BOARD_PCA10040) | defined(BOARD_EXG_V3)
  nrf_gpio_pin_dir_set(ADS1299_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); //sets 'direction' = input/output
  nrf_gpio_pin_dir_set(ADS1299_PWDN_RST_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
#endif
#if defined(BOARD_EXG_V3) && LEDS_ENABLE == 1
  nrf_gpio_cfg_output(LED_1);
  nrf_gpio_cfg_output(LED_2);
  nrf_gpio_pin_set(LED_1);
  nrf_gpio_pin_set(LED_2);
#endif
  uint32_t err_code;
  if (!nrf_drv_gpiote_is_init()) {
    err_code = nrf_drv_gpiote_init();
  }
  NRF_LOG_RAW_INFO(" nrf_drv_gpiote_init: %d\r\n", err_code);
  NRF_LOG_FLUSH();
  APP_ERROR_CHECK(err_code);
  bool is_high_accuracy = true;
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
  in_config.is_watcher = true;
  in_config.pull = NRF_GPIO_PIN_NOPULL;
  err_code = nrf_drv_gpiote_in_init(ADS1299_DRDY_PIN, &in_config, in_pin_handler);
  NRF_LOG_RAW_INFO(" nrf_drv_gpiote_in_init: %d: \r\n", err_code);
  NRF_LOG_FLUSH();
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(ADS1299_DRDY_PIN, true);
  ads1299_powerdn();
}
#endif /* defined(ADS1299) */
static void wait_for_event(void) {
  (void)sd_app_evt_wait();
}
/**@brief Function for application main entry.
 */
int main(void) {
  bool erase_bonds = false;
  uint32_t err_code = NRF_SUCCESS;
  // Initialize.
  log_init();
  timers_init();
  ble_stack_init();
#if defined(ADS1299)
  ads1299_gpio_init();
#endif
  gap_params_init();
  gatt_init();
  advertising_init();
  services_init();
  conn_params_init();
#if defined(ADS1299)
  //  ads_spi_init();
  ads_spi_init_with_sample_freq(SPI_SCLK_WRITE_REG);
  nrf_delay_ms(5);
  ads1299_powerdn();
  ads1299_powerup();
  ads1299_stop_rdatac();
  ads1299_init_regs();
  ads1299_soft_start_conversion();
  ads1299_check_id();
  ads1299_start_rdatac();
  ads1299_standby();
  nrf_delay_ms(10);
  m_eeg.eeg_ch1_count = 0;
#endif
  // Start execution.
  application_timers_start();
  advertising_start();
  NRF_LOG_RAW_INFO(" BLE Advertising Start! \r\n");
  NRF_LOG_FLUSH();
#if LEDS_ENABLE == 1
  nrf_gpio_pin_clear(LED_2); // Green
  nrf_gpio_pin_set(LED_1);   //Blue
#endif
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
  m_samples = 0;
#endif
// Enter main loop
#if NRF_LOG_ENABLED == 1
  for (;;) {
    if (!NRF_LOG_PROCESS()) {
      wait_for_event();
    }
  }
#endif
}

/**
 * @}
 */