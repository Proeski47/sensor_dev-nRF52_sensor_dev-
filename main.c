/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "diskio_blkdev.h"
#include "ff.h"
#include "nrf_block_dev_sdc.h"

#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_hrs.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#define FILE_NAME "15_16.wav" //"NORDIC.TXT"
#define EXTN "02_03_21"
#define TEST_STRING "SD card example!"

//#define SDC_SCK_PIN 27  ///< SDC serial clock (SCK) pin.
//#define SDC_MOSI_PIN 26 ///< SDC serial data in (DI) pin.
//#define SDC_MISO_PIN 28 ///< SDC serial data out (DO) pin.
//#define SDC_CS_PIN 25   ///< SDC chip select (CS) pin.

// i2s headers
#include "adc.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_twi.h"

#include "limits.h"
#include "stdlib.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <nrf_error.h>
#include <sdk_errors.h>

#include <hardfault.h>

#define LED_OK BSP_BOARD_LED_0
#define LED_ERROR BSP_BOARD_LED_1

#define DEVICE_NAME "Nordic_MicroPhone"         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "NordicSemiconductor" /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */

#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Heart Rate Measurement packet. */
#define HANDLE_LENGTH 2                                                             /**< Length of handle inside Heart Rate Measurement packet. */
#define MAX_HRM_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Heart Rate Measurement. */
#define INITIAL_ADC_VALUE 0                                                         /**< Initial Sensor data value. */

#define SENSOR_MEAS_INTERVAL APP_TIMER_TICKS(1.8225001822500182250018225001823) /**< Sensor value detection interval (ticks). */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(10, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY 0                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 1                               /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define BLE_UUID_SENSOR_ADC_SERVICE BLE_UUID_HEART_RATE_SERVICE
#define BLE_UUID_SENSOR_ADC_CHAR BLE_UUID_HEART_RATE_MEASUREMENT_CHAR

BLE_HRS_DEF(m_hrs); /**< Sensor ADC service instance. */
//BLE_BAS_DEF(m_bas);                       /**< Structure used to identify the battery service. */

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
APP_TIMER_DEF(m_sensor_timer_id);   /**< Sensor timer. */
//APP_TIMER_DEF(m_heart_rate_timer_id);     /**< Heart rate measurement timer. */
//APP_TIMER_DEF(m_rr_interval_timer_id);    /**< RR interval timer. */
//APP_TIMER_DEF(m_sensor_contact_timer_id); /**< Sensor contact detected timer. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
//static bool m_rr_interval_enabled = true;                /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */
static uint16_t conn_handle;

static uint8_t *sending_data = NULL;
static uint16_t cnt = 1;

uint16_t count = 0;

//static uint8_t decomp_sample[3];
//static uint32_t adc_data[] = {34, 89, 450, 33, 98, 56, 27, 90, 100, 233};

//static sensorsim_cfg_t m_battery_sim_cfg;         /**< Battery Level sensor simulator configuration. */
//static sensorsim_state_t m_battery_sim_state;     /**< Battery Level sensor simulator state. */
//static sensorsim_cfg_t m_heart_rate_sim_cfg;      /**< Heart Rate sensor simulator configuration. */
//static sensorsim_state_t m_heart_rate_sim_state;  /**< Heart Rate sensor simulator state. */
//static sensorsim_cfg_t m_rr_interval_sim_cfg;     /**< RR Interval sensor simulator configuration. */
//static sensorsim_state_t m_rr_interval_sim_state; /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {{BLE_UUID_SENSOR_ADC_SERVICE, BLE_UUID_TYPE_BLE}};

//static void advertising_start(bool erase_bonds);

#define I2S_DATA_BLOCK_WORDS (81)
#define DATA_BLOCK_WORDS_IN_BYTES (81 * 3)
static uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
static uint32_t m_buffer_tx[2][I2S_DATA_BLOCK_WORDS];

static uint8_t decomp_sample[DATA_BLOCK_WORDS_IN_BYTES] = {0};
//static uint8_t decomp_sample[243] = {0};

static uint32_t const *volatile mp_block_to_check = NULL;
//static uint32_t volatile m_blocks_transferred = 0;
static bool m_error_encountered;

static bool run_time_updates = false;

static uint16_t service_handle;
static ble_gatts_char_handles_t char_handles;
static uint16_t value_handle;

static bool flag = 0;
static bool flag1 = 0;

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1
#endif

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

// RTC pins for TWI
#define RTC_SCL_S 7
#define RTC_SDA_S 8

// Address of RTC slave
#define RTC_ADDR 81
//#define RTC_ADDR 0x51

// Registers of RTC slave
#define RTC_REG_START 0x00
#define RTC_REG_TIME_START 0x04
#define EXT_RTC_MAX_REG 0x11

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

#define I2CDEV_NO_MEM_ADDR 0xFF

/** ---------------------------------------------------------------------------
* ------------------------------- I2C/TWI -------------------------------------
*------------------------------------------------------------------------------
*/

/**
 * @brief TWI for ADC initialization.
 */
void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL_PIN,
      .sda = ARDUINO_SDA_PIN,
      .frequency = NRF_DRV_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = true};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

/** ---------------------------------------------------------------------------
* ------------------------------- BLUETOOTH -----------------------------------
*------------------------------------------------------------------------------
*/

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

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  printf("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.

  } else {
    ret_code_t err_code;
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    //APP_ERROR_CHECK(err_code);
    printf("\n Advertising with err code = %d", err_code);
  }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    advertising_start(false);
    break;

  default:
    break;
  }
}

static void check_samples(void) {
  uint16_t i;
  uint32_t const *p_word = NULL;
  uint32_t dec_val;

  // First word is the discerning sample
  decomp_sample[0] = 0xF0;
  decomp_sample[1] = 0x0D;
  decomp_sample[2] = count;
  //if(count == 1
  printf("\n %2x%2x%2x", decomp_sample[0], decomp_sample[1], decomp_sample[2]);

  for (i = 1; i < I2S_DATA_BLOCK_WORDS; i++) {
    //Decompose 32 bit data to 8 bit data and store only 24 bit data discarding the msb 8bits.
    p_word = &mp_block_to_check[i];

    decomp_sample[3 * i + 0] = ((uint8_t *)p_word)[0];
    //printf("\n Word byte 1 = 0x%2x", decomp_sample[3 * i + 0]);

    decomp_sample[3 * i + 1] = ((uint8_t *)p_word)[1];
    //printf("\n Word byte 2 = 0x%2x", decomp_sample[3 * i + 1]);

    decomp_sample[3 * i + 2] = ((uint8_t *)p_word)[2];
    //printf("\n Word byte 3 = 0x%2x", decomp_sample[3 * i + 2])

    //dec_val = ((uint32_t)decomp_sample[3 * i + 2] << 16) | (decomp_sample[3 * i + 1] << 8) | decomp_sample[3 * i + 0];
    if (count != 0) {
      printf("\n %2x%2x%2x", decomp_sample[3 * i + 2], decomp_sample[3 * i + 1], decomp_sample[3 * i + 0]);
    }
    //NRF_LOG_INFO("\n %2x%2x%2x = %d",decomp_sample[3 * i + 2], decomp_sample[3 * i + 1], decomp_sample[3 * i + 0], dec_val);
  }
  sending_data = &decomp_sample[0];
  count++;
}

ret_code_t adc_measurement_send(ble_hrs_t *p_adc, uint8_t *sensor_data) {
  uint8_t encoded_adc[MAX_HRM_LEN - 1];
  //uint8_t encoded_adc[DATA_BLOCK_WORDS_IN_BYTES];
  uint16_t i;
  uint32_t err_code;
  uint16_t hvx_len;
  uint16_t len;
  ble_gatts_hvx_params_t hvx_params;

  //p_adc->conn_handle = m_conn_handle;
  //p_adc->hrm_handles.value_handle = value_handle;

  ble_gatts_value_t set_value;

  // Initialize value struct.
  memset(&set_value, 0, sizeof(set_value));

  set_value.len = MAX_HRM_LEN - 1;
  set_value.offset = 0;
  set_value.p_value = sensor_data;

  err_code = sd_ble_gatts_value_set(p_adc->conn_handle,
      p_adc->hrm_handles.value_handle,
      &set_value);
  printf("GATTS value set with code = %d", err_code);

  for (i = 0; i < MAX_HRM_LEN - 1; i++) {
    encoded_adc[i] = *(sensor_data + i);
  }

  // Send value if connected and notifying
  if (p_adc->conn_handle != BLE_CONN_HANDLE_INVALID) {

    hvx_len = MAX_HRM_LEN - 1;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_adc->hrm_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = encoded_adc;

    err_code = sd_ble_gatts_hvx(p_adc->conn_handle, &hvx_params);
    printf("\n HVX with err code = %d", err_code);
  } else {
    err_code = NRF_ERROR_INVALID_STATE;
  }
  return err_code;
}

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void adc_meas_timeout_handler(void *p_context) {
  ret_code_t err_code;
  uint8_t *buff_data = NULL;
  uint8_t i;

  UNUSED_PARAMETER(p_context);

  if (sending_data != NULL) {
    //buff_data = &sending_data[cnt * MAX_HRM_LEN];
    buff_data = &sending_data[0];
    err_code = adc_measurement_send(&m_hrs, buff_data);
    //APP_ERROR_HANDLER(err_code);
    if (err_code == NRF_SUCCESS) {
      flag1 = 1;
      bsp_board_led_off(LED_ERROR);
      bsp_board_led_invert(LED_OK);
    } else {
      bsp_board_led_off(LED_OK);
      bsp_board_led_invert(LED_ERROR);
    }
    printf("ADC measure sending with err code = %d", err_code);
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  ret_code_t err_code;

  // Initialize timer module.
  err_code = app_timer_init();
  printf("\n Timer initialization with err code = %d");

  err_code = app_timer_create(&m_sensor_timer_id,
      APP_TIMER_MODE_REPEATED,
      adc_meas_timeout_handler);
  printf("\n Timer create with err code = %d");
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

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);

  printf("\n It's a new world");
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    printf("GATT ATT MTU on connection 0x%x changed to %d.",
        p_evt->conn_handle,
        p_evt->params.att_mtu_effective);
  }
  ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  printf("\n It's a new day");
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for encoding a Heart Rate Measurement.
 *
 * @param[in]   p_hrs              Heart Rate Service structure.
 * @param[in]   heart_rate         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t hrm_encode(ble_hrs_t *p_hrs, uint32_t heart_rate, uint8_t *p_encoded_buffer) {

  uint8_t len = 0;
  len += uint24_encode(heart_rate, &p_encoded_buffer[2]);
  return len;
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void) {
  uint32_t err_code;

  ble_hrs_init_t adc_init;
  ble_hrs_t *p_adc = &m_hrs;

  // Initialize Queued Write Module.
  nrf_ble_qwr_init_t qwr_init = {0};

  ble_uuid_t ble_uuid;
  ble_add_char_params_t add_char_params;
  uint8_t encoded_initial_adc[MAX_HRM_LEN - 1];

  //uint8_t initial_adc_data[MAX_HRM_LEN] = {0x12, 0x34, 0x56, 0x78};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  printf("\n QWR init with err code = %d");

  memset(&adc_init, 0, sizeof(adc_init));

  adc_init.evt_handler = NULL;
  adc_init.hrm_cccd_wr_sec = SEC_OPEN;
  adc_init.bsl_rd_sec = SEC_OPEN;

  p_adc->evt_handler = adc_init.evt_handler;
  p_adc->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_adc->max_hrm_len = MAX_HRM_LEN - 1;

  // Initialize ADC Data Write Service.
  BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SENSOR_ADC_SERVICE);

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_adc->service_handle);
  printf("\n Service add with err code = %d");

  // Add sensor data measurement characteristic
  memset(&add_char_params, 0, sizeof(add_char_params));

  add_char_params.uuid = BLE_UUID_SENSOR_ADC_CHAR;
  add_char_params.max_len = MAX_HRM_LEN - 1;
  add_char_params.init_len = hrm_encode(p_adc, INITIAL_ADC_VALUE, encoded_initial_adc);
  add_char_params.p_init_value = encoded_initial_adc;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;
  add_char_params.char_props.notify = 1;
  add_char_params.read_access = adc_init.bsl_rd_sec;
  add_char_params.cccd_write_access = adc_init.hrm_cccd_wr_sec;

  err_code = characteristic_add(p_adc->service_handle, &add_char_params, &(p_adc->hrm_handles));

  printf("\n Char add with err code = %d", err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void) {
  ret_code_t err_code;

  err_code = app_timer_start(m_sensor_timer_id, SENSOR_MEAS_INTERVAL, NULL);
  //APP_ERROR_CHECK(err_code);
  printf("\n Timer start with err code = %d", err_code);
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
  uint16_t cccdd_handle;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = m_hrs.hrm_handles.cccd_handle;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

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
    printf("Fast advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;

  default:
    break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    printf("Connected.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_2MBPS,
            .tx_phys = BLE_GAP_PHY_2MBPS,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    printf("Disconnected, reason %d.",
        p_ble_evt->evt.gap_evt.params.disconnected.reason);
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    printf("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    printf("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    printf("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    printf("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
    break;

  case BLE_GAP_EVT_AUTH_KEY_REQUEST:
    printf("BLE_GAP_EVT_AUTH_KEY_REQUEST");
    break;

  case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
    printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
    break;

  case BLE_GAP_EVT_AUTH_STATUS:
    printf("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
        p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
        p_ble_evt->evt.gap_evt.params.auth_status.bonded,
        p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  printf("\n Here I am");
  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  //APP_ERROR_CHECK(err_code);
  printf("\n BLE stack config with err code = %d", err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  //APP_ERROR_CHECK(err_code);
  printf("\n BLE stack enable with  err code = %d", err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
  ret_code_t err_code;

  switch (event) {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE) {
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  default:
    break;
  }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;
  printf("\n This is me");

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  //APP_ERROR_CHECK(err_code);
  printf("\n Advert init with err = %d", err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool *p_erase_bonds) {
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  ret_code_t err_code;

  err_code = nrf_ble_lesc_request_handler();
  APP_ERROR_CHECK(err_code);

  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/** ---------------------------------------------------------------------------
* --------------------------- Microphone & ADC --------------------------------
*------------------------------------------------------------------------------
*/

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in i2cdev_readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t i2cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {

  if (regAddr != I2CDEV_NO_MEM_ADDR) {
    nrf_drv_twi_tx(&m_twi, devAddr, &regAddr, 1, true);
  }

  ret_code_t r = nrf_drv_twi_rx(&m_twi, devAddr, data, length);

  return (r == NRF_SUCCESS);
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool i2cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
  uint8_t w2_data[2];
  uint8_t length = 2;
  uint8_t test;

  if (regAddr != I2CDEV_NO_MEM_ADDR) {
    w2_data[0] = regAddr;
    w2_data[1] = data;
  } else {
    w2_data[0] = data;
    w2_data[1] = data;
    length = 1;
  }
  if (NRF_SUCCESS == nrf_drv_twi_tx(&m_twi, devAddr, w2_data, length, false)) {
    if (regAddr == PAGE_CTL_REGISTER) {
      printf("\r\n Accessing Page no : %02x ", data);
    }

    i2cdev_readBytes(devAddr, regAddr, 1, &test);
    if (test != data) {
      printf("\r\n Error in register %d. Write = %02x but Read = %02x", regAddr, data, test);
    }
  }
  return NRF_SUCCESS == nrf_drv_twi_tx(&m_twi, devAddr, w2_data, length, false);
}

// Delay time between consecutive I2S transfers performed in the main loop
// (in milliseconds).
//#define PAUSE_TIME          500
// Number of blocks of data to be contained in each transfer.

unsigned char mode;

static void button_handler(bsp_event_t evt) {
  switch (evt) {
  case BSP_EVENT_KEY_0:
    nrf_gpio_pin_toggle(LED_1);
    break;
  case BSP_EVENT_KEY_1:
    nrf_gpio_pin_toggle(LED_2);
    mode = 0x02;
    break;
  default:
    break;
  }
}

/**@brief Function for initializing bsp module.
 */
void bsp_configuration() {
  uint32_t err_code;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, button_handler);
  APP_ERROR_CHECK(err_code);
}

//bool status_write = true;
//status_write = write_the_block(p_decomp);

/*Data structure to hold a single frame with two channels*/
typedef struct PCM32_stereo_s {
  uint32_t left;
  uint32_t right;
} PCM32_stereo_t;

//static void check_rx_data(uint32_t const *p_block) {
//++m_blocks_transferred;

//if (m_error_encountered) {
//  bsp_board_led_off(LED_OK);
//  bsp_board_led_invert(LED_ERROR);
//} else {
//  bsp_board_led_off(LED_ERROR);
//  bsp_board_led_invert(LED_OK);
//}
//}

static void i2s_data_handler(nrf_drv_i2s_buffers_t const *p_released, uint32_t status) {
  // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
  // each time next buffers are requested, so data corruption is not
  // expected.
  ASSERT(p_released);

  // When the handler is called after the transfer has been stopped
  // (no next buffers are needed, only the used buffers are to be
  // released), there is nothing to do.
  if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)) {
    return;
  }

  // First call of this handler occurs right after the transfer is started.
  // No data has been transferred yet at this point, so there is nothing to
  // check. Only the buffers for the next part of the transfer should be
  // provided.
  if (!p_released->p_rx_buffer) {
    nrf_drv_i2s_buffers_t const next_buffers = {
        .p_rx_buffer = m_buffer_rx[1],
        .p_tx_buffer = NULL,
    };
    APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));

    //mp_block_to_fill = m_buffer_tx[1];

  } else {
    mp_block_to_check = p_released->p_rx_buffer;
    // The driver has just finished accessing the buffers pointed by
    // 'p_released'. They can be used for the next part of the transfer
    // that will be scheduled now.
    APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));

    // The pointer needs to be typecasted here, so that it is possible to
    // modify the content it is pointing to (it is marked in the structure
    // as pointing to constant data because the driver is not supposed to
    // modify the provided data).
    //mp_block_to_fill = (uint32_t *)p_released->p_tx_buffer;
  }
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  bsp_board_leds_on();
  app_error_save_and_stop(id, pc, info);
}

void twi_adc_configuration(void) {
  ret_code_t err_code;
  uint8_t sample_data;
  uint8_t address = 24;
  bool detected_device = false;

  // Hardware Reset
  nrf_gpio_cfg_output(ARDUINO_9_PIN);
  nrf_gpio_pin_clear(ARDUINO_9_PIN);
  nrf_delay_ms(100);
  nrf_gpio_pin_set(ARDUINO_9_PIN);

  twi_init();
  err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
  if (err_code == NRF_SUCCESS) {
    detected_device = true;
    printf("TWI device detected at address 0x%x.", address);
  }

  if (!detected_device) {
    printf("No device was found.");
    return;
  }
  //Page select 0
  i2cdev_writeByte(address, PAGE_CTL_REGISTER, PAGE_0);

  i2cdev_writeByte(address, SW_Reset, 0x01);
  i2cdev_writeByte(address, Clock_Gen_Multiplexing, 0x07); //0b00000111 BCLK
  i2cdev_writeByte(address, PLL_PR_VAL, 0x11);             // P = 1 R = 1
  i2cdev_writeByte(address, PLL_J_VAL, 0x30);              // J = 48
  i2cdev_writeByte(address, PLL_D_VAL_MSB, 0x00);          // D = 0000
  i2cdev_writeByte(address, PLL_D_VAL_LSB, 0x00);
  i2cdev_writeByte(address, ADC_NADC, 0x89);       //NADC = 9
  i2cdev_writeByte(address, ADC_MADC, 0x82);       //MADC = 2
  i2cdev_writeByte(address, ADC_AOSR, 0x80);       //ASOR = 128
  i2cdev_writeByte(address, ADC_IADC, 0xBC);       //IADC = 94*2 = 188
  i2cdev_writeByte(address, ADC_AUDIO_IC_1, 0x20); // I2S, 24 bits, without tristate
  i2cdev_writeByte(address, ADC_Processing_Block_Selection, 0x01);
  i2cdev_writeByte(address, L_ADC_Volume_Control, 0x00);
  i2cdev_writeByte(address, PLL_PR_VAL, 0x91);
  //nrf_delay_ms(10);
  //Page select 1
  i2cdev_writeByte(address, PAGE_CTL_REGISTER, PAGE_1);
  //Read current page
  i2cdev_readBytes(address, PAGE_CTL_REGISTER, 1, &sample_data);
  if (sample_data == PAGE_1) {
    i2cdev_writeByte(address, MICBIAS_Control, 0x00);
    i2cdev_writeByte(address, L_ADC_INPUT_Selection_L_PGA, 0xF3); // Change to differential => 0x7F, 0b01111111 (enable IN2L(P) as plus and IN3L(M) as minus)
    i2cdev_writeByte(address, L_Analog_PGA_Setting, 0x00);        // Change may not needed.. could edit 0b00000111
  }

  //Page select 0
  i2cdev_writeByte(address, PAGE_CTL_REGISTER, PAGE_0);
  //Read current page
  i2cdev_readBytes(address, PAGE_CTL_REGISTER, 1, &sample_data);
  if (sample_data == PAGE_0) {
    i2cdev_writeByte(address, ADC_Digital, 0x80);
    i2cdev_writeByte(address, ADC_Fine_Volume_Control, 0x08);
  }

  i2cdev_readBytes(address, 36, 1, &sample_data);
  //printf("\r\n The ADC FLAG Register values is %02x",sample_data);

  nrf_drv_twi_disable(&m_twi);
}

/** ---------------------------------------------------------------------------
* ------------------------------EXTERNAL RTC ----------------------------------
*------------------------------------------------------------------------------
*/

/**
 * @brief Write into external RTC module (RV-8263-C7)
 *
 * Write time data from the external RTC module (RV-8263-C7) over a TWI session
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
ret_code_t ext_rtc_write(uint8_t rtc_addr, uint8_t reg_data, uint8_t *write_data, uint8_t length, bool no_stop) {
  uint8_t rtc_write_data[length + 1];
  if (reg_data <= EXT_RTC_MAX_REG) {
    rtc_write_data[0] = reg_data;
    for (int i = 1; i <= length; i++) {
      rtc_write_data[i] = write_data[i - 1];
    }
    return nrf_drv_twi_tx(&m_twi, rtc_addr, rtc_write_data, length, no_stop);
  } else
    return NRF_ERROR_INVALID_ADDR;
}

/**
 * @brief Read into external RTC module (RV-8263-C7)
 *
 * Read time data from the external RTC module (RV-8263-C7) over a TWI session
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
ret_code_t ext_rtc_read(uint8_t rtc_addr, uint8_t reg_data, uint8_t *read_data, uint8_t length, bool no_stop) {
  ret_code_t err_code;
  if (reg_data <= EXT_RTC_MAX_REG) {
    err_code = nrf_drv_twi_tx(&m_twi, rtc_addr, &reg_data, sizeof(reg_data), no_stop);
    if (err_code == NRF_SUCCESS) {
      return nrf_drv_twi_rx(&m_twi, rtc_addr, read_data, length);
    } else
      return err_code;
  } else
    return NRF_ERROR_INVALID_ADDR;
}

#ifdef BSP_BUTTON_1
#define PIN_IN BSP_BUTTON_1
#endif
#ifndef PIN_IN
#error "Please indicate input pin"
#error "Please indicate input pin"
#endif

#ifdef BSP_LED_1
#define PIN_OUT BSP_LED_1
#endif
#ifndef PIN_OUT
#error "Please indicate output pin"
#endif

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  nrf_drv_gpiote_out_set(PIN_OUT);

  if (mode == 0x00) {
    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = NULL,
        .p_rx_buffer = m_buffer_rx[0],
    };

    ret_code_t err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
    //APP_ERROR_CHECK(err_code);
    printf("I2S driver start with err code = %d", err_code);
    mode = 0x01; // Recording mode
  } else if (mode == 0x01) {
    mode = 0x00; // Stop recording
    nrf_drv_gpiote_out_set(PIN_OUT);
    nrf_drv_i2s_stop();
    bsp_board_leds_off();
  }
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

  err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

static uint8_t decToBCD(int data) {
  uint8_t byte = data % 10;
  data = data / 10;
  byte = byte + (data << 4);
  return byte;
}

static char *time_string(uint8_t *data) {
  struct tm time_fetched;
  int temp;

  static char rtc_time[50] = "Time: ";

  //data[6] = data[6] + 2000;

  temp = ((data[0] & 0xF0) >> 4) * 10 + (data[0] & 0x0F);
  time_fetched.tm_sec = temp;

  temp = ((data[1] & 0xF0) >> 4) * 10 + (data[1] & 0x0F);
  time_fetched.tm_min = temp;

  temp = ((data[2] & 0xF0) >> 4) * 10 + (data[2] & 0x0F);
  time_fetched.tm_hour = temp;

  temp = ((data[3] & 0xF0) >> 4) * 10 + (data[3] & 0x0F);
  time_fetched.tm_mday = temp;

  temp = ((data[4] & 0xF0) >> 4) * 10 + (data[4] & 0x0F);
  time_fetched.tm_wday = temp;
  printf("Weekday= %d", temp);

  temp = ((data[5] & 0xF0) >> 4) * 10 + (data[5] & 0x0F);
  temp = temp - 1;
  time_fetched.tm_mon = temp; // Range in 00 - 11
  printf("Month= %d", temp);

  temp = ((data[6] & 0xF0) >> 4) * 10 + (data[6] & 0x0F);
  temp = temp + 100;
  time_fetched.tm_year = temp; // Years since 1900

  strcat(rtc_time, asctime(&time_fetched));

  //strcat(rtc_time, (char)data[0]);
  //strcat(rtc_time, (char)data[1]);
  //strcat(rtc_time, (char)data[2]);
  //strcat(rtc_time, (char)data[3]);
  //strcat(rtc_time, (char)data[4]);
  //strcat(rtc_time, (char)data[5]);
  //strcat(rtc_time, (char)data[6]);

  return rtc_time;
}

/** ---------------------------------------------------------------------------
* ---------------------------MAIN FUNCTION MODULE------------------------------
*------------------------------------------------------------------------------
*/

int main(void) {

  bool erase_bonds;

  uint32_t err_code = NRF_SUCCESS;
  uint8_t sample_data;
  uint8_t response;
  bool detected_device = false;

  static struct tm time_keep;

  static struct tm time_fetched;

  uint8_t write_data[10] = {0x58}; // Data array to be written to the RTC slave
                                   // First register address in RTC slave to write to/read from

  uint8_t read_data[10]; // Data array to be read from the RTC slave

  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // Asking the user's choice to reset external RTC after each nRF reset
  //printf("Do you want to initialize real time to external RTC (RV-8263-C7)? [Y:N]: \n");
  //scanf("%c", &response);

  //switch (response) {
  //case 'Y':

  //  printf("TWI scanner started \n");

  //  twi_init();

  //  err_code = nrf_drv_twi_rx(&m_twi, RTC_ADDR, &sample_data, sizeof(sample_data));
  //  if (err_code == NRF_SUCCESS) {
  //    detected_device = true;
  //    printf("TWI device detected at address 0x%02x \n", RTC_ADDR);
  //  }

  //  if (!detected_device) {
  //    printf("No device was found \n");
  //  }

  //  err_code = ext_rtc_write(RTC_ADDR, RTC_REG_START, write_data, 1, false);
  //  APP_ERROR_CHECK(err_code);
  //  if (NRF_SUCCESS == err_code) {
  //    printf("Time registers of 0x%02x successfully reset \n", RTC_ADDR);
  //  }

  //  // Set the current date from the terminal manually by user
  //  printf("Enter current date in DD-MM-YYYY-WD: \n");
  //  scanf("%d-%d-%d-%d", &time_keep.tm_mday, &time_keep.tm_mon, &time_keep.tm_year, &time_keep.tm_wday);

  //  // Set the current time from the terminal manually by user
  //  printf("Enter current time in HH-MM-SS: \n");
  //  scanf("%d-%d-%d", &time_keep.tm_hour, &time_keep.tm_min, &time_keep.tm_sec);

  //  time_keep.tm_wday = time_keep.tm_wday - 1;
  //  time_keep.tm_year = time_keep.tm_year - 2000;

  //  write_data[0] = decToBCD(time_keep.tm_sec);
  //  write_data[1] = decToBCD(time_keep.tm_min);
  //  write_data[2] = decToBCD(time_keep.tm_hour);
  //  write_data[3] = decToBCD(time_keep.tm_mday);
  //  write_data[4] = decToBCD(time_keep.tm_wday);
  //  write_data[5] = decToBCD(time_keep.tm_mon);
  //  write_data[6] = decToBCD(time_keep.tm_year);

  //  // Set the ext. RTC time just once at the beginning
  //  err_code = ext_rtc_write(RTC_ADDR, RTC_REG_TIME_START, write_data, 7, false);
  //  APP_ERROR_CHECK(err_code);
  //  if (NRF_SUCCESS == err_code) {
  //    printf("Time data written into registers starting from 0x%02x \n", RTC_REG_TIME_START);
  //  }
  //  nrf_drv_twi_disable(&m_twi);
  //  break;

  //case 'N':
  //  break;
  //}

  twi_adc_configuration();
  gpio_init();

  // Fs = 44.5kHz if 32DIV15
  // Fs = 31.75 if 32DIV21
  // Fs = 15.87KHz if 32DIV42

  mode = 0x00; // standby/stop mode

  // I2S Sampling Rate = 2.1333333 MHz/48 = 44444.44 Hz
  nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
  config.sdin_pin = I2S_SDIN_PIN;
  config.sdout_pin = NRFX_I2S_PIN_NOT_USED;
  config.sdout_pin = NRFX_I2S_PIN_NOT_USED;
  config.mck_pin = NRFX_I2S_PIN_NOT_USED;
  //config.mck_setup = NRF_I2S_MCK_32MDIV15FL;
  config.mck_setup = NRF_I2S_MCK_32MDIV15;
  config.ratio = NRF_I2S_RATIO_48X;
  config.format = NRF_I2S_FORMAT_I2S;
  config.sample_width = NRF_I2S_SWIDTH_24BIT;
  config.channels = NRF_I2S_CHANNELS_LEFT;

  err_code = nrf_drv_i2s_init(&config, i2s_data_handler);
  printf("I2S driver initialized with err code = %d", err_code);
  //APP_ERROR_CHECK(err_code);

  //uint32_t year, month, day, hour, minute, second;

  //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  //NRF_CLOCK->TASKS_HFCLKSTART = 1;
  //while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

  //nrf_cal_init();
  //nrf_cal_set_callback(calendar_updated, 4);

  //if (mp_block_to_check) {
  //    check_rx_data(mp_block_to_check);
  //    mp_block_to_check = NULL;
  //  }
  //uint32_t data[3] = {90, 76, 89};
  //uint16_t i;

  // Initialize.
  log_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  advertising_init();
  services_init();
  conn_params_init();
  peer_manager_init();

  // Start execution.
  printf("\n Connected to BLE central");
  advertising_start(erase_bonds);

  for (;;) {

    mp_block_to_check = NULL;

    //Wait for an event.
    __WFE();
    // Clear the event register.
    __SEV();
    __WFE();

    while (mode == 0x01) {

      // Fetch real time-stamp from external RTC
      //nrf_drv_twi_enable(&m_twi);
      //err_code = ext_rtc_read(RTC_ADDR, RTC_REG_TIME_START, read_data, 7, true);
      //APP_ERROR_CHECK(err_code);
      //if (NRF_SUCCESS == err_code) {
      //  printf("Time data read from 0x%02x \n", RTC_ADDR);
      //}
      //nrf_drv_twi_disable(&m_twi);

      //printf("%s", time_string(read_data));

      if (mp_block_to_check) {
        check_samples();
        if (flag == 0) {
          application_timers_start();
          flag = 1;
        }
      }
     
      //if (cnt != 21) {
      //  if (mp_block_to_check) {
      //    check_samples();
      //    if (flag == 0) {
      //      application_timers_start();
      //      flag = 1;
      //    }
      //  }

      //  if (flag1 == 1) {
      //    cnt++;
      //    flag1 = 0;
      //  }
      //}

      //if (cnt == 21) {
      //  mode = 0x00;
      //  app_timer_stop(m_sensor_timer_id);
      //  break;
      //}

      if (mode != 0x01) {
        if (flag == 1) {
          app_timer_stop(m_sensor_timer_id);
          flag = 0;
        }
        sending_data = NULL;
        idle_state_handle();
        break;
      }
    }
  }
}
