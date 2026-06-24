/*
 * dw1000_stm32.h
 * Low-level SPI wrapper for STM32
 */

#ifndef DW1000_STM32_H_
#define DW1000_STM32_H_

#include "dw1000.h"

/* Master SPI Transfer function handling dynamic 1, 2, and 3-byte headers */
static inline void dw1000_SPI_Transfer(dw1000_HandleTypeDef *dw1000, uint8_t *header, uint8_t header_len, uint8_t *data, uint16_t data_len, uint8_t is_write) {
    /* Pull CS Low */
    HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
    
    /* Send the dynamic 1, 2, or 3 byte header */
    HAL_SPI_Transmit(dw1000->spi, header, header_len, HAL_MAX_DELAY);
    
    /* Send or receive the payload */
    if (data_len > 0) {
        if (is_write) {
            HAL_SPI_Transmit(dw1000->spi, data, data_len, HAL_MAX_DELAY);
        } else {
            HAL_SPI_Receive(dw1000->spi, data, data_len, HAL_MAX_DELAY);
        }
    }
    
    /* Pull CS High */
    HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

static inline void dw1000_WriteData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t header[1];
    header[0] = 0x80 | (reg & 0x3F); // Bit 7 = 1 (Write), Bit 6 = 0 (No Sub-index)
    dw1000_SPI_Transfer(dw1000, header, 1, data, len, 1);
}

static inline void dw1000_ReadData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t header[1];
    header[0] = 0x00 | (reg & 0x3F); // Bit 7 = 0 (Read), Bit 6 = 0 (No Sub-index)
    dw1000_SPI_Transfer(dw1000, header, 1, data, len, 0);
}

static inline void dw1000_WriteSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t sub_idx, uint8_t *data, uint16_t len) {
    uint8_t header[3];
    uint8_t header_len = 1;

    header[0] = 0x80 | 0x40 | (reg & 0x3F); // Bit 7=1 (Write), Bit 6=1 (Sub-index)

    if (sub_idx < 128) {
        header[1] = (uint8_t)sub_idx;
        header_len = 2;
    } else {
        header[1] = 0x80 | (sub_idx & 0x7F); // Bit 7=1 (Extended index present)
        header[2] = (uint8_t)(sub_idx >> 7);
        header_len = 3;
    }
    dw1000_SPI_Transfer(dw1000, header, header_len, data, len, 1);
}

static inline void dw1000_ReadSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t sub_idx, uint8_t *data, uint16_t len) {
    uint8_t header[3];
    uint8_t header_len = 1;

    header[0] = 0x00 | 0x40 | (reg & 0x3F); // Bit 7=0 (Read), Bit 6=1 (Sub-index)

    if (sub_idx < 128) {
        header[1] = (uint8_t)sub_idx;
        header_len = 2;
    } else {
        header[1] = 0x80 | (sub_idx & 0x7F);
        header[2] = (uint8_t)(sub_idx >> 7);
        header_len = 3;
    }
    dw1000_SPI_Transfer(dw1000, header, header_len, data, len, 0);
}

static inline void dw1000_SSBlink(dw1000_HandleTypeDef *dw1000) {
    HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

#endif /* DW1000_STM32_H_ */
