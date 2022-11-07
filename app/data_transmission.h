#ifndef __DATA_TRANSMISSION_H_
#define __DATA_TRANSMISSION_H_
#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_log.h"

#define TOUCH_SPI_CS_ON             0
#define TOUCH_SPI_CS_OFF            1

#define SPI_CS_HIGH()               nrf_gpio_pin_write(TOUCH_SPI0_CSN_IO, 1)
#define SPI_CS_LOW()                nrf_gpio_pin_write(TOUCH_SPI0_CSN_IO, 0)

#define TOUCH_SPI2_MOSI_IO          16
#define TOUCH_SPI2_MISO_IO          17
#define TOUCH_SPI2_CSN_IO           18
#define TOUCH_SPI2_CLK_IO           19

#define SLAVE_SPI_RSP_IO            13

#define SPI_INSTANCE                2

extern bool spi_dir_out;
extern bool spi_send_done;

void usr_spim_init(void);

void spi_cs_set(bool pinState);

void usr_spi_write(uint8_t *p_buffer, uint32_t size);

void usr_spi_read(uint8_t *p_buffer, uint32_t size);

void usr_spi_enable(void);

void usr_spi_disable(void);

#endif
