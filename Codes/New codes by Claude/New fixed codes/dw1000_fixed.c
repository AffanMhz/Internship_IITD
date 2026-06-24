/*
 * dw1000.c  — patched version
 *
 * Changes from uploaded version:
 *   [C] dw1000_StartReceive: SFCST must be 0 (not use_crc) — SFCST is a
 *       TX-only "suppress FCS" bit; setting it during RXENAB is wrong intent
 *       even though SYS_CTRL self-clears and makes it harmless in practice.
 *       Fixed to always write 0 so the register is unambiguous.
 *
 * Everything else is unchanged — the Init, StartTransmit, timestamps, etc.
 * are all correct in the uploaded version.
 */

#include <string.h>
#include "dw1000.h"
#include "dw1000_stm32.h"

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value) {
    if(value) {
        *(data+(bitnum / 8)) |= (1 << (bitnum % 8));
    } else {
        *(data+(bitnum / 8)) &= ~(1 << (bitnum % 8));
    }
}

uint8_t dw1000_IsSet(uint8_t *data, uint8_t bitnum) {
    if(*(data+(bitnum / 8)) & (1 << (bitnum % 8))) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t dw1000_ReadSubReg8(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t sub_idx) {
    uint8_t val;
    dw1000_ReadSubData(dw1000, reg, sub_idx, &val, 1);
    return val;
}

uint16_t dw1000_ReadSubReg16(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t sub_idx) {
    uint8_t buffer[2];
    dw1000_ReadSubData(dw1000, reg, sub_idx, buffer, 2);
    return (buffer[1] << 8) | buffer[0];
}

uint32_t dw1000_ReadSubReg32(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t sub_idx) {
    uint8_t buffer[4];
    dw1000_ReadSubData(dw1000, reg, sub_idx, buffer, 4);
    return ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[1] << 8) | buffer[0];
}

uint8_t dw1000_Init(dw1000_HandleTypeDef *dw1000, uint8_t channel, uint8_t prf, uint8_t datarate) {
    uint8_t agc_tune1[2] = {0x70, 0x88};
    dw1000_WriteSubData(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB, agc_tune1, 2);

    uint8_t agc_tune2[4] = {0x07, 0xA9, 0x02, 0x25};
    dw1000_WriteSubData(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE2_SUB, agc_tune2, 4);

    uint8_t drx_tune2[4] = {0x2D, 0x00, 0x1A, 0x31};
    dw1000_WriteSubData(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE2_SUB, drx_tune2, 4);

    uint8_t lde_cfg1 = 0x6D;
    dw1000_WriteSubData(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB, &lde_cfg1, 1);

    uint8_t lde_cfg2[2] = {0x07, 0x16};
    dw1000_WriteSubData(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG2_SUB, lde_cfg2, 2);

    uint8_t tx_power[4] = {0x48, 0x28, 0x08, 0x0E};
    dw1000_WriteData(dw1000, DW1000_TX_POWER, tx_power, 4);

    /* TX_FCTRL: 110 kbps, 16 MHz PRF, 128-symbol preamble */
    uint8_t tx_fctrl[5];
    dw1000_ReadData(dw1000, DW1000_TX_FCTRL, tx_fctrl, 5);
    tx_fctrl[1] &= ~0x60;
    tx_fctrl[2] &= ~0x3F;
    tx_fctrl[2] |= 0x15;
    dw1000_WriteData(dw1000, DW1000_TX_FCTRL, tx_fctrl, 5);

    /* SYS_CFG: set RXM110K for 110 kbps receive */
    uint8_t sys_cfg[DW1000_SYS_CFG_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);
    dw1000_SetBit(sys_cfg, DW1000_SYS_CFG_RXM110K, 1);
    dw1000_WriteData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);

    uint16_t agc_rb = dw1000_ReadSubReg16(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB);
    return (agc_rb == 0x8870) ? 1 : 0;
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
    return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
           ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] << 8) | buffer[0];
}

uint32_t dw1000_GetReceiverTimeTrackingInterval(dw1000_HandleTypeDef *dw1000) {
    uint8_t buffer[DW1000_RX_TTCKI_LEN];
    dw1000_ReadData(dw1000, DW1000_RX_TTCKI, buffer, DW1000_RX_TTCKI_LEN);
    return (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
}

void dw1000_GetSystemStatus(dw1000_HandleTypeDef *dw1000, uint8_t *sys_status) {
    dw1000_ReadData(dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
}

uint32_t dw1000_GetSystemState(dw1000_HandleTypeDef *dw1000) {
    uint8_t buffer[5];
    dw1000_ReadData(dw1000, DW1000_SYS_STATE, buffer, DW1000_SYS_STATE_LEN);
    return ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[1] << 8) | buffer[0];
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
    uint16_t tflen = use_crc ? (length + 2) : length;

    uint8_t fctrl[DW1000_TX_FCTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);
    fctrl[0] = tflen & 0x00FF;
    fctrl[1] &= ~(0x03);
    fctrl[1] |= ((tflen >> 8) & 0x03);
    dw1000_WriteData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

    uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
    /* SFCST=0 means chip appends real FCS; SFCST=1 suppresses it.
     * use_crc=1 -> SFCST=0 (let chip add FCS). Polarity is inverted. */
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

void dw1000_SetDataToTransmit(dw1000_HandleTypeDef *dw1000, uint8_t *data,
                               uint16_t length, uint8_t use_crc) {
    (void)use_crc;
    if(length > DW1000_TX_BUFFER_LEN) return;
    dw1000_WriteData(dw1000, DW1000_TX_BUFFER, data, length);
}

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc) {
    uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
    /* [FIX C] SFCST is a TX-only bit and must be 0 when enabling the receiver.
     * The uploaded code wrote use_crc here, which set SFCST=1 during RXENAB.
     * SYS_CTRL is self-clearing so it was harmless in practice, but wrong
     * intent and could confuse future read-back. Always write 0 for RX. */
    dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, 0);
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
    if(use_crc && n > 2) n -= 2;
    return n;
}

void dw1000_GetDataReceived(dw1000_HandleTypeDef *dw1000, uint8_t *data, uint16_t length) {
    dw1000_ReadData(dw1000, DW1000_RX_BUFFER, data, length);
}
