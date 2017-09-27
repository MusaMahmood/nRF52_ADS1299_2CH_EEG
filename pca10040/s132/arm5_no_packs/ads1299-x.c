#include "ads1299-x.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
/**headers for µs delay:*/
#include "compiler_abstraction.h"
#include "nrf.h"
#include <stdio.h>

//NOTE: ADS1299 Default Registers
uint8_t ads1299_default_registers[] = {
    ADS1299_REGDEFAULT_CONFIG1,
    ADS1299_REGDEFAULT_CONFIG2,
    ADS1299_REGDEFAULT_CONFIG3,
    ADS1299_REGDEFAULT_LOFF,
    ADS1299_REGDEFAULT_CH1SET,
    ADS1299_REGDEFAULT_CH2SET,
    ADS1299_REGDEFAULT_CH3SET,
    ADS1299_REGDEFAULT_CH4SET,
    ADS1299_REGDEFAULT_CH5SET,
    ADS1299_REGDEFAULT_CH6SET,
    ADS1299_REGDEFAULT_CH7SET,
    ADS1299_REGDEFAULT_CH8SET,
    ADS1299_REGDEFAULT_BIAS_SENSP,
    ADS1299_REGDEFAULT_BIAS_SENSN,
    ADS1299_REGDEFAULT_LOFF_SENSP,
    ADS1299_REGDEFAULT_LOFF_SENSN,
    ADS1299_REGDEFAULT_LOFF_FLIP,
    ADS1299_REGDEFAULT_LOFF_STATP,
    ADS1299_REGDEFAULT_LOFF_STATN,
    ADS1299_REGDEFAULT_GPIO,
    ADS1299_REGDEFAULT_MISC1,
    ADS1299_REGDEFAULT_MISC2,
    ADS1299_REGDEFAULT_CONFIG4};

/**@SPI HANDLERS:
 * @brief SPI user event handler.
 * @param event
 */

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
#define RX_DATA_LEN 12
static uint8_t rx_data[RX_DATA_LEN];
static volatile bool spi_xfer_done;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
    void *p_context) {
  spi_xfer_done = true;
}

void ads_spi_init(void) {
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  //SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
  spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST; //APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_1;               //CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1299_SPI_MISO_PIN;
  spi_config.sck_pin = ADS1299_SPI_SCLK_PIN;
  spi_config.mosi_pin = ADS1299_SPI_MOSI_PIN;
  spi_config.ss_pin = ADS1299_SPI_CS_PIN;
  spi_config.orc = 0x55;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" SPI Initialized..\r\n");
#endif
}

void ads_spi_uninit(void) {
  nrf_drv_spi_uninit(&spi);
  NRF_LOG_INFO(" SPI UNinitialized \r\n");
}

void ads_spi_init_with_sample_freq(uint8_t spi_sclk) {
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  switch (spi_sclk) {
  case 2:
    spi_config.frequency = NRF_DRV_SPI_FREQ_2M;
    break;
  case 4:
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    break;
  case 8:
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    break;
  default:
    break;
  }
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST; //APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_1;               //CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1299_SPI_MISO_PIN;
  spi_config.sck_pin = ADS1299_SPI_SCLK_PIN;
  spi_config.mosi_pin = ADS1299_SPI_MOSI_PIN;
  spi_config.ss_pin = ADS1299_SPI_CS_PIN;
  spi_config.orc = 0x55;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  NRF_LOG_INFO(" SPI Initialized @ %d MHz\r\n",spi_sclk);
}

void ads1299_powerdn(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040) | defined(BOARD_EXG_V3)
  nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_clear(ADS1299_RESET_PIN);
  nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
#endif
  nrf_delay_us(20);
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1299-x POWERED DOWN..\r\n");
#endif
}

void ads1299_powerup_reset(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040) | defined(BOARD_EXG_V3)
  nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_clear(ADS1299_RESET_PIN);
  nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
#endif
  nrf_delay_ms(50);
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1299-x POWERED UP AND RESET..\r\n");
#endif
}

void ads1299_powerup(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040) | defined(BOARD_EXG_V3)
  nrf_gpio_pin_set(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_set(ADS1299_RESET_PIN);
  nrf_gpio_pin_set(ADS1299_PWDN_PIN);
#endif
  nrf_delay_ms(1000); // Allow time for power-on reset
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1299-x POWERED UP...\r\n");
#endif
}

void ads1299_standby(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_STANDBY;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1299-x placed in standby mode...\r\n");
#endif
}

void ads1299_wake(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_WAKEUP;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
  nrf_delay_ms(10); // Allow time to wake up - 10ms
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1299-x Wakeup..\r\n");
#endif
}

void ads1299_soft_start_conversion(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_START;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Start ADC conversion..\r\n");
#endif
}

void ads1299_stop_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_SDATAC;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Disabled..\r\n");
#endif
}

void ads1299_start_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_RDATAC;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Enabled..\r\n");
#endif
}

void ads1299_check_id(void) {
  uint8_t device_id_reg_value;
  uint8_t tx_data_spi[3];
  uint8_t rx_data_spi[7];
  memset(rx_data_spi, 0, 7);
  tx_data_spi[0] = 0x20; // First command byte = 001r rrrr (r rrrr = register start address)
  tx_data_spi[1] = 0x00; // Intend to read 1 byte: (Bytes to read)-1 = 0
  tx_data_spi[2] = 0x00; //This will be replaced by Reg Data
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 3, rx_data_spi, 7));
  while (!spi_xfer_done) {
    __WFE();
  }
//NOTE: CHANGES FROM [2] to [3] for EASY DMA
#if SPI0_USE_EASY_DMA == 1
  device_id_reg_value = rx_data_spi[3];
#else
  device_id_reg_value = rx_data_spi[2];
#endif
  bool is_ads_1299_4 = (device_id_reg_value & 0x1F) == (ADS1299_4_DEVICE_ID);
  bool is_ads_1299_6 = (device_id_reg_value & 0x1F) == (ADS1299_6_DEVICE_ID);
  bool is_ads_1299 = (device_id_reg_value & 0x1F) == (ADS1299_DEVICE_ID);
  uint8_t revisionVersion = (device_id_reg_value & 0xE0) >> 5;
#if LOG_LOW_DETAIL == 1
  if (is_ads_1299 || is_ads_1299_6 || is_ads_1299_4) {
    NRF_LOG_INFO("Device Address Matches!\r\n");
  } else {
    NRF_LOG_INFO("********SPI I/O Error, Device Not Detected! *********** \r\n");
    NRF_LOG_INFO("SPI Transfer Dump: \r\n");
    NRF_LOG_INFO("ID[b0->3]: [0x%x | 0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[0], rx_data_spi[1], rx_data_spi[2], rx_data_spi[3]);
    NRF_LOG_INFO("ID[b4->6]: [0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[4], rx_data_spi[5], rx_data_spi[6]);
  }
  if (is_ads_1299) {
    NRF_LOG_INFO("Device Name: ADS1299 \r\n");
  } else if (is_ads_1299_6) {
    NRF_LOG_INFO("Device Name: ADS1299-6 \r\n");
  } else if (is_ads_1299_4) {
    NRF_LOG_INFO("Device Name: ADS1299-4 \r\n");
  }
  if (is_ads_1299 || is_ads_1299_6 || is_ads_1299_4) {
    NRF_LOG_INFO("Device Revision #%d\r\n", revisionVersion);
    NRF_LOG_INFO("Device ID: 0x%x \r\n", device_id_reg_value);
  }
#endif
}

void ads1299_init_regs(void) {
  uint8_t err_code;
  uint8_t num_registers = 23;
  uint8_t txrx_size = num_registers + 2;
  uint8_t tx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t rx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t wreg_init_opcode = 0x41;
  for (int i = 0; i < txrx_size; ++i) {
    tx_data_spi[i] = 0;
    rx_data_spi[i] = 0;
  }
  tx_data_spi[0] = wreg_init_opcode;
  tx_data_spi[1] = num_registers - 1;
  for (int j = 0; j < num_registers; ++j) {
    tx_data_spi[j + 2] = ads1299_default_registers[j];
  }
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, num_registers, rx_data_spi, num_registers));
  while (!spi_xfer_done) {
    __WFE();
  }
  nrf_delay_ms(150);
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Power-on reset and initialization procedure.. EC: %d \r\n", err_code);
#endif
}

//NOTE: DATA RETRIEVAL
/**@brief Function for acquiring a EEG Voltage Measurement samples.
 *
 * @details Uses SPI
 *          
 */
void get_eeg_voltage_array(ble_eeg_t *p_eeg) {
  memset(rx_data, 0, RX_DATA_LEN);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, RX_DATA_LEN));
  while (!spi_xfer_done)
    __WFE();
  if (((rx_data[0] + rx_data[1] + rx_data[2]) == 0xC0) && ((rx_data[6] + rx_data[7] + rx_data[8]) == 0x00)) {
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count++] = rx_data[3];
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count++] = rx_data[4];
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count++] = rx_data[5];
  }
}

void get_eeg_voltage_array_2ch(ble_eeg_t *p_eeg) {
  memset(rx_data, 0, RX_DATA_LEN);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, RX_DATA_LEN));
  while (!spi_xfer_done)
    __WFE();
  if (((rx_data[0] + rx_data[1] + rx_data[2]) == 0xC0) && ((rx_data[9] + rx_data[10] + rx_data[11]) == 0x00)) {
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count] = rx_data[3];
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count+1] = rx_data[4];
    p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count+2] = rx_data[5];
    p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[6];
    p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[7];
    p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count++] = rx_data[8];
  }
}