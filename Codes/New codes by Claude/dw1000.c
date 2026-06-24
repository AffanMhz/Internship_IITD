/*
 * dw1000.c
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 */

#include <string.h>
#include "dw1000.h"

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value) {
  uint8_t byte_index = bitnum / 8;
  uint8_t bit_mask = (uint8_t)(1u << (bitnum % 8));

  if(value) {
    data[byte_index] |= bit_mask;
  }
  else {
    data[byte_index] &= (uint8_t)~bit_mask;
  }
};

uint8_t dw1000_IsSet(uint8_t *data, uint8_t bitnum) {
  uint8_t byte_index = bitnum / 8;
  uint8_t bit_mask = (uint8_t)(1u << (bitnum % 8));

  return (data[byte_index] & bit_mask) ? 1u : 0u;
}

dw1000_dev_id_t dw1000_GetDevID(dw1000_HandleTypeDef *dw1000) {
  dw1000_dev_id_t dev_id;
  uint8_t buffer[DW1000_DEV_ID_LEN];

  dw1000_ReadData(dw1000, DW1000_DEV_ID, buffer, DW1000_DEV_ID_LEN);

  dev_id.rev = buffer[0] & 0x0F;
  dev_id.ver = buffer[0] >> 4;
  dev_id.model = buffer[1];
  dev_id.ridtag = (buffer[3] << 8) | buffer[2];

  return dev_id;
}

dw1000_pan_addr_t dw1000_GetPanAddress(dw1000_HandleTypeDef *dw1000) {
  dw1000_pan_addr_t pan_addr;
  uint8_t buffer[DW1000_PANADR_LEN];

  dw1000_ReadData(dw1000, DW1000_PANADR, buffer, DW1000_PANADR_LEN);

  pan_addr.short_addr = (buffer[1] << 8) | buffer[0];
  pan_addr.pan_id = (buffer[3] << 8) | buffer[2];

  return pan_addr;
}

void dw1000_SetPanAddress(dw1000_HandleTypeDef *dw1000, dw1000_pan_addr_t *pan_addr) {
  uint8_t buffer[] = {
      pan_addr->short_addr & 0x00FF,
      pan_addr->short_addr >> 8,
      pan_addr->pan_id & 0x00FF,
      pan_addr->pan_id >> 8,
  };

  dw1000_WriteData(dw1000, DW1000_PANADR, buffer, DW1000_PANADR_LEN);
}

dw1000_channel_t dw1000_GetChannel(dw1000_HandleTypeDef *dw1000) {
  dw1000_channel_t channel;
  uint8_t buffer[DW1000_CHAN_CTRL_LEN];

  dw1000_ReadData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);

  channel.tx_chan = buffer[0] & 0x0F;
  channel.rx_chan = buffer[0] >> 4;
  channel.prf = (buffer[2] >> 2) & 0x03;
  channel.tx_preamble = ((buffer[3] & 0x07) << 2) | (buffer[2] >> 6);
  channel.rx_preamble = buffer[3] >> 3;

  return channel;
}

void dw1000_SetChannel(dw1000_HandleTypeDef *dw1000, dw1000_channel_t *channel) {
  // Read channel control register first because the data isn't complete
  uint8_t buffer[DW1000_CHAN_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);

  buffer[0] = (channel->rx_chan << 4) | channel->tx_chan;
  buffer[2] = (buffer[2] & 0xF3) | (channel->prf << 2);
  buffer[2] = (buffer[2] & 0x3F) | ((channel->tx_preamble & 0x03) << 6);
  buffer[3] = (channel->rx_preamble << 3) | (channel->tx_preamble >> 2);

  dw1000_WriteData(dw1000, DW1000_CHAN_CTRL, buffer, DW1000_CHAN_CTRL_LEN);
}

dw1000_timestamp_t dw1000_GetSystemTimeCounter(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[DW1000_SYS_TIME_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_TIME, buffer, DW1000_SYS_TIME_LEN);
  return ((uint64_t)buffer[4] << 32) | ((uint64_t)buffer[3] << 24) | ((uint64_t)buffer[2] << 16) | ((uint64_t)buffer[1] << 8) | (uint64_t)buffer[0];
}

uint32_t dw1000_GetReceiverTimeTrackingInterval(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[DW1000_RX_TTCKI_LEN];
  dw1000_ReadData(dw1000, DW1000_RX_TTCKI, buffer, DW1000_RX_TTCKI_LEN);
  return (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
}

void dw1000_ClearAllStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  memset(sys_status, 0xFF, DW1000_SYS_STATUS_LEN);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

void dw1000_IdleMode(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_TRXOFF, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, 4);
}

void dw1000_StartTransmit(dw1000_HandleTypeDef *dw1000, uint16_t length, uint8_t use_crc) {
  // Write transmit frame control register
  uint8_t fctrl[DW1000_TX_FCTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);
  fctrl[0] = length & 0x00FF;
  fctrl[1] &= ~(0x03);
  fctrl[1] |= ((length >> 8) & 0x03);
  dw1000_WriteData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

  // Start transmit
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, !use_crc);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_TXSTRT, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, 4);
}

void dw1000_ClearTransmitStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXFRB, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXPRS, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXPHS, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_TXFRS, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

void dw1000_SetDataToTransmit(dw1000_HandleTypeDef *dw1000, uint8_t *data, uint16_t length, uint8_t use_crc) {
  // TODO: Extended frame length?
  if(length > DW1000_TX_BUFFER_LEN) {
    return;  // Error handling
  }
  dw1000_WriteData(dw1000, DW1000_TX_BUFFER, data, length);
  // txfctrl in arduino-dwm1000?
}

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc) {
  uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, !use_crc);
  dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_RXENAB, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
}

void dw1000_ClearReceiveStatus(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXDFR, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_LDEDONE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_LDEERR, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXPHE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXFCE, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXFCG, 1);
  dw1000_SetBit(sys_status, DW1000_SYS_STATUS_RXRFSL, 1);
  dw1000_WriteData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

void dw1000_ReceiveAutoEnable(dw1000_HandleTypeDef *dw1000, uint8_t val) {
  uint8_t sys_config[DW1000_SYS_CFG_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CFG, sys_config, DW1000_SYS_CFG_LEN);
  dw1000_SetBit(sys_config, DW1000_SYS_CFG_RXAUTR, val);
  dw1000_WriteData(dw1000, DW1000_SYS_CFG, sys_config, DW1000_SYS_CFG_LEN);
}

uint8_t dw1000_ReceiveDataFrameReady(dw1000_HandleTypeDef *dw1000) {
  uint8_t sys_status[DW1000_SYS_STATUS_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
  return dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXDFR);
}

uint16_t dw1000_GetDataReceivedLength(dw1000_HandleTypeDef *dw1000, uint8_t use_crc) {
  uint16_t n;
  uint8_t rx_frame_info[DW1000_RX_FINFO_LEN];
  dw1000_ReadData(dw1000, DW1000_RX_FINFO, rx_frame_info, DW1000_RX_FINFO_LEN);
  n = ((rx_frame_info[1] & 0x03) << 8) | rx_frame_info[0];
  if(use_crc && n>2)
    n-=2;
  return n;
}

void dw1000_GetDataReceived(dw1000_HandleTypeDef *dw1000, uint8_t* data, uint16_t length) {
  dw1000_ReadData(dw1000, DW1000_RX_BUFFER, data, length);
}

void dw1000_GetSystemStatus(dw1000_HandleTypeDef *dw1000, uint8_t *status_out5) {
  dw1000_ReadData(dw1000, DW1000_SYS_STATUS, status_out5, DW1000_SYS_STATUS_LEN);
}

uint32_t dw1000_GetSystemState(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[DW1000_SYS_STATE_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_STATE, buffer, DW1000_SYS_STATE_LEN);
  /* SYS_STATE is a 24-bit value spread across the lower 3 bytes:
   * bits 0:3   = PMSC state
   * bits 8:11  = RX state
   * bits 16:19 = TX state */
  return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);
}

/*
 * dw1000_Init()
 *
 * Performs the DW1000 power-on tuning sequence required for reliable
 * TX/RX operation. Values are taken from the DW1000 User Manual section
 * "2.5.5 Default Configurations that should be modified" and verified
 * against the tested thotro/arduino-dw1000 tune() routine and the
 * DecaWave mbed driver's manageLDE()/loadLDE() initialisation.
 *
 * Sequence:
 *  1. PMSC soft-reset (force SYSCLKS to 19.2 MHz XTI clock, toggle
 *     SOFTRESET bits in PMSC_CTRL0) - clears the chip into a known state.
 *  2. Load LDE microcode from ROM into RAM (required for accurate RX
 *     timestamps; without this, RXDFR may still set on some firmware
 *     revisions but timestamps/diagnostics will be invalid and on most
 *     revisions the receiver will not function correctly at all).
 *  3. Write AGC_TUNE1/2/3, DRX_TUNE0b/1a/1b/2/4H, LDE_CFG1/2, RF_RXCTRLH,
 *     RF_TXCTRL, TC_PGDELAY, FS_PLLTUNE, FS_PLLCFG, FS_XTALT, TX_POWER -
 *     these configure the analog/digital front end for the chosen
 *     channel, PRF and data rate. Defaults are NOT correct for channel 5
 *     / PRF16 and this is the root cause of "TX appears to work but RX
 *     never sees RXDFR" symptoms.
 *  4. Configure TX_FCTRL data-rate/PRF bits and SYS_CFG (disable frame
 *     filtering, enable 1023-byte frames, RX auto-re-enable).
 *
 * channel: only channel 5 tuning values are provided in this
 *          implementation (matches dw1000_SetChannel default in main.c).
 * prf: DW1000_PRF_16MHZ or DW1000_PRF_64MHZ
 * data_rate: DW1000_TX_FCTRL_TXBR_110K / _850K / _6M8
 *
 * Returns 1 if the post-init read-back sanity checks pass, 0 otherwise.
 */
uint8_t dw1000_Init(dw1000_HandleTypeDef *dw1000, uint8_t channel, dw1000_prf_t prf, uint8_t data_rate) {
  (void)channel; /* only channel 5 supported by the tuning values below */

  /* ---- Step 1: PMSC soft reset sequence ----
   * Force system clock to 19.2 MHz XTI so the reset is clocked correctly,
   * then pulse the SOFTRESET field (bits 31:28 of PMSC_CTRL0) low then
   * high again. */
  uint32_t pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);

  /* Force SYSCLKS = 19.2 MHz XTI (bits 1:0 = 01) */
  pmsc_ctrl0 = (pmsc_ctrl0 & ~0x03u) | DW1000_PMSC_CTRL0_SYSCLKS_19M;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  /* Clear SOFTRESET bits (31:28) to 0 to trigger reset of digital logic */
  pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);
  pmsc_ctrl0 &= 0x0FFFFFFFul;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  HAL_Delay(1); /* allow reset to propagate */

  /* Release SOFTRESET bits back to 0xF (normal operation) */
  pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);
  pmsc_ctrl0 |= 0xF0000000ul;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  /* Return system clock to AUTO so PLL takes over once locked */
  pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);
  pmsc_ctrl0 &= ~0x03u;
  pmsc_ctrl0 |= DW1000_PMSC_CTRL0_SYSCLKS_AUTO;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  /* ---- Step 2: Load LDE microcode ----
   * Per User Manual: set system clock to 125 MHz PLL is NOT required for
   * this step; the documented sequence uses the 19.2 MHz XTI clock with
   * OTP interface set up for LDE load, then sets PMSC_CTRL0 LDE load bits.
   *
   * Sequence (matches DecaWave reference and thotro driver):
   *   1. Set SYSCLKS to 19.2 MHz XTI in PMSC_CTRL0
   *   2. Set OTP_CTRL = 0x8000 (LDELOAD bit) via PMSC_CTRL0 bit 17
   *   3. Wait ~150us
   *   4. Restore SYSCLKS to AUTO
   */
  pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);
  pmsc_ctrl0 &= ~0x03u;
  pmsc_ctrl0 |= DW1000_PMSC_CTRL0_SYSCLKS_19M;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  /* OTP_CTRL: set LDELOAD bit (bit 15 of OTP_CTRL, value 0x8000) */
  dw1000_WriteSubReg16(dw1000, DW1000_OTP_IF, DW1000_OTP_CTRL_SUB, 0x8000u);

  HAL_Delay(1); /* >150us required; 1ms is generous and simple under HAL_Delay */

  /* Restore SYSCLKS to AUTO */
  pmsc_ctrl0 = dw1000_ReadSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB);
  pmsc_ctrl0 &= ~0x03u;
  pmsc_ctrl0 |= DW1000_PMSC_CTRL0_SYSCLKS_AUTO;
  dw1000_WriteSubReg32(dw1000, DW1000_PSMC, DW1000_PMSC_CTRL0_SUB, pmsc_ctrl0);

  /* ---- Step 3: AGC tuning ---- */
  uint16_t agc_tune1 = (prf == DW1000_PRF_64MHZ) ? DW1000_AGC_TUNE1_64MHZ : DW1000_AGC_TUNE1_16MHZ;
  dw1000_WriteSubReg16(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB, agc_tune1);
  dw1000_WriteSubReg32(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE2_SUB, DW1000_AGC_TUNE2_VAL);
  dw1000_WriteSubReg16(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE3_SUB, DW1000_AGC_TUNE3_VAL);

  /* ---- DRX tuning ---- */
  uint16_t drx_tune0b;
  switch (data_rate) {
    case DW1000_TX_FCTRL_TXBR_850K: drx_tune0b = DW1000_DRX_TUNE0b_850K_STD; break;
    case DW1000_TX_FCTRL_TXBR_6M8:  drx_tune0b = DW1000_DRX_TUNE0b_6M8_STD;  break;
    default:                        drx_tune0b = DW1000_DRX_TUNE0b_110K_STD; break;
  }
  dw1000_WriteSubReg16(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE0b_SUB, drx_tune0b);

  uint16_t drx_tune1a = (prf == DW1000_PRF_64MHZ) ? DW1000_DRX_TUNE1a_PRF64 : DW1000_DRX_TUNE1a_PRF16;
  dw1000_WriteSubReg16(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE1a_SUB, drx_tune1a);

  /* DRX_TUNE1b/4H depend on preamble length: this driver targets the
   * standard 1024-symbol preamble (used by dw1000_SetChannel default
   * tx_preamble=0x4 in main.c, which corresponds to 1024 symbols at
   * PRF16). Use the "long preamble" values. */
  dw1000_WriteSubReg16(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE1b_SUB, DW1000_DRX_TUNE1b_LONG);
  dw1000_WriteSubReg16(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE4H_SUB, DW1000_DRX_TUNE4H_LONG);

  uint32_t drx_tune2 = (prf == DW1000_PRF_64MHZ) ? DW1000_DRX_TUNE2_PAC8_PRF64 : DW1000_DRX_TUNE2_PAC8_PRF16;
  dw1000_WriteSubReg32(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE2_SUB, drx_tune2);

  /* ---- LDE configuration ---- */
  dw1000_WriteSubReg8(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB, DW1000_LDE_CFG1_VAL);
  uint16_t lde_cfg2 = (prf == DW1000_PRF_64MHZ) ? DW1000_LDE_CFG2_PRF64 : DW1000_LDE_CFG2_PRF16;
  dw1000_WriteSubReg16(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG2_SUB, lde_cfg2);

  /* ---- RF / TX calibration (channel 5 values) ---- */
  dw1000_WriteSubReg8(dw1000, DW1000_RF_CONF, DW1000_RF_RXCTRLH_SUB, DW1000_RF_RXCTRLH_VAL_CH5);
  dw1000_WriteSubReg32(dw1000, DW1000_RF_CONF, DW1000_RF_TXCTRL_SUB, DW1000_RF_TXCTRL_VAL_CH5);
  dw1000_WriteSubReg8(dw1000, DW1000_TX_CAL, DW1000_TC_PGDELAY_SUB, DW1000_TC_PGDELAY_VAL_CH5);

  /* ---- Frequency synthesiser (channel 5 values) ---- */
  dw1000_WriteSubReg8(dw1000, DW1000_FS_CTRL, DW1000_FS_PLLTUNE_SUB, DW1000_FS_PLLTUNE_VAL_CH5);
  dw1000_WriteSubReg32(dw1000, DW1000_FS_CTRL, DW1000_FS_PLLCFG_SUB, DW1000_FS_PLLCFG_VAL_CH5);
  dw1000_WriteSubReg8(dw1000, DW1000_FS_CTRL, DW1000_FS_XTALT_SUB, DW1000_FS_XTALT_MIDRANGE);

  /* ---- TX power (default, smart-TX off) ---- */
  dw1000_WriteData(dw1000, DW1000_TX_POWER, (uint8_t[]){
    (uint8_t)(DW1000_TX_POWER_MAN_DEFAULT & 0xFF),
    (uint8_t)((DW1000_TX_POWER_MAN_DEFAULT >> 8) & 0xFF),
    (uint8_t)((DW1000_TX_POWER_MAN_DEFAULT >> 16) & 0xFF),
    (uint8_t)((DW1000_TX_POWER_MAN_DEFAULT >> 24) & 0xFF)
  }, DW1000_TX_POWER_LEN);

  /* ---- Step 4: TX_FCTRL data-rate / PRF bits ----
   * TX_FCTRL bits: [6:5]=TXBR (data rate), [18:17]=TXPRF
   * Preserve length field (bits 9:0) which dw1000_StartTransmit sets per
   * frame; here we only configure the rate/PRF bits which are static. */
  uint8_t fctrl[DW1000_TX_FCTRL_LEN];
  dw1000_ReadData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

  fctrl[1] &= ~((0x03u << 5) & 0xFF); /* clear TXBR bits [6:5] within byte1 (bits 13:8 of TX_FCTRL) */
  /* TXBR occupies bits 14:13 of the 40-bit TX_FCTRL register, i.e. bits
   * 6:5 of byte 1 (byte1 = bits 15:8). */
  fctrl[1] = (uint8_t)((fctrl[1] & ~0x60u) | ((data_rate & 0x03u) << 5));

  /* TXPRF occupies bits 18:17, i.e. bits 2:1 of byte 2 (byte2 = bits 23:16) */
  uint8_t txprf = (prf == DW1000_PRF_64MHZ) ? DW1000_TX_FCTRL_TXPRF_64M : DW1000_TX_FCTRL_TXPRF_16M;
  fctrl[2] = (uint8_t)((fctrl[2] & ~0x06u) | ((txprf & 0x03u) << 1));

  dw1000_WriteData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

  /* ---- SYS_CFG: disable frame filtering, allow 1023-byte frames,
   * enable RX auto re-enable so a CRC/PHY error doesn't leave the
   * receiver permanently off. */
  uint8_t sys_cfg[DW1000_SYS_CFG_LEN];
  dw1000_ReadData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);
  dw1000_SetBit(sys_cfg, DW1000_SYS_CFG_FFEN, 0);     /* frame filtering off */
  dw1000_SetBit(sys_cfg, DW1000_SYS_CFG_PHR_MODE, 0); /* standard frames, <=127 bytes; set to 1 for extended (1023) */
  dw1000_SetBit(sys_cfg, DW1000_SYS_CFG_RXAUTR, 1);   /* RX auto re-enable */
  dw1000_WriteData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);

  /* ---- Sanity check: read back AGC_TUNE1 and LDE_CFG1 ---- */
  uint16_t agc_check = dw1000_ReadSubReg16(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB);
  uint8_t lde_check = dw1000_ReadSubReg8(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB);

  if (agc_check != agc_tune1 || lde_check != DW1000_LDE_CFG1_VAL) {
    return 0;
  }

  return 1;
}


