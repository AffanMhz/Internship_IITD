/*
 * dw1000_stm32.h
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 */

#ifndef DW1000_STM32_H_
#define DW1000_STM32_H_

#include "dw1000.h"

void dw1000_WriteData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t header[] = {(1 << 7) | reg};
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

void dw1000_ReadData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t header[] = {(0 << 7) | reg};
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

/*
 * Extended (sub-register) addressing access functions.
 *
 * Per DW1000 User Manual section "SPI Interface": every transaction header
 * starts with a 1-byte header containing the R/W bit (bit7), a sub-index
 * present flag (bit6) and the 6-bit register file ID (bits 5:0).
 *
 * If a sub-address is present, a second header byte follows containing the
 * low 7 bits of the sub-address (bits 6:0) and a flag (bit7) indicating a
 * third header byte is present for sub-addresses > 0x7F. The third header
 * byte (if present) contains bits 14:7 of the sub-address.
 *
 * header[0] = (RW << 7) | (1 << 6) | (reg & 0x3F)
 * header[1] = (more << 7) | (subaddr & 0x7F)
 * header[2] = (subaddr >> 7) & 0xFF   (only if subaddr > 0x7F)
 */
void dw1000_WriteSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t *data, uint16_t len) {
  uint8_t header[3];
  uint8_t header_len;

  header[0] = (uint8_t)(0x80 | 0x40 | (reg & 0x3F)); /* write, sub-index present */

  if (subaddr <= 0x7F) {
    header[1] = (uint8_t)(subaddr & 0x7F);
    header_len = 2;
  } else {
    header[1] = (uint8_t)(0x80 | (subaddr & 0x7F));
    header[2] = (uint8_t)((subaddr >> 7) & 0xFF);
    header_len = 3;
  }

  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, header_len, HAL_MAX_DELAY);
  HAL_SPI_Transmit(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

void dw1000_ReadSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t *data, uint16_t len) {
  uint8_t header[3];
  uint8_t header_len;

  header[0] = (uint8_t)(0x00 | 0x40 | (reg & 0x3F)); /* read, sub-index present */

  if (subaddr <= 0x7F) {
    header[1] = (uint8_t)(subaddr & 0x7F);
    header_len = 2;
  } else {
    header[1] = (uint8_t)(0x80 | (subaddr & 0x7F));
    header[2] = (uint8_t)((subaddr >> 7) & 0xFF);
    header_len = 3;
  }

  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, header_len, HAL_MAX_DELAY);
  HAL_SPI_Receive(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

/* Convenience helpers for common widths, little-endian (DW1000 native order) */
void dw1000_WriteSubReg8(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t value) {
  dw1000_WriteSubData(dw1000, reg, subaddr, &value, 1);
}

uint8_t dw1000_ReadSubReg8(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr) {
  uint8_t value;
  dw1000_ReadSubData(dw1000, reg, subaddr, &value, 1);
  return value;
}

void dw1000_WriteSubReg16(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint16_t value) {
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(value & 0xFF);
  buffer[1] = (uint8_t)(value >> 8);
  dw1000_WriteSubData(dw1000, reg, subaddr, buffer, 2);
}

uint16_t dw1000_ReadSubReg16(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr) {
  uint8_t buffer[2];
  dw1000_ReadSubData(dw1000, reg, subaddr, buffer, 2);
  return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

void dw1000_WriteSubReg32(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint32_t value) {
  uint8_t buffer[4];
  buffer[0] = (uint8_t)(value & 0xFF);
  buffer[1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[2] = (uint8_t)((value >> 16) & 0xFF);
  buffer[3] = (uint8_t)((value >> 24) & 0xFF);
  dw1000_WriteSubData(dw1000, reg, subaddr, buffer, 4);
}

uint32_t dw1000_ReadSubReg32(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr) {
  uint8_t buffer[4];
  dw1000_ReadSubData(dw1000, reg, subaddr, buffer, 4);
  return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

void dw1000_SSBlink(dw1000_HandleTypeDef *dw1000) {
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

#endif /* DW1000_STM32_H_ */
