/*
 * dw1000.c
 */

#include <string.h>
#include "dw1000.h"
#include "dw1000_stm32.h"

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value) {
    if(value) {
        *(data+(bitnum / 8)) |= (1 << (bitnum % 8)); // Note: Fix bit shift modulo
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

/* Sub Register Functions */
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
    return ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];
}

/* Required Tuning Parameters for DWM1000 (16MHz PRF, Ch 5) */
uint8_t dw1000_Init(dw1000_HandleTypeDef *dw1000, uint8_t channel, uint8_t prf, uint8_t datarate) {
    // 1. AGC_TUNE1
    uint8_t agc_tune1[2] = {0x70, 0x88}; // 0x8870
    dw1000_WriteSubData(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB, agc_tune1, 2);

    // 2. AGC_TUNE2
    uint8_t agc_tune2[4] = {0x07, 0xA9, 0x02, 0x25}; // 0x2502A907
    dw1000_WriteSubData(dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE2_SUB, agc_tune2, 4);

    // 3. DRX_TUNE2 (16 MHz PRF, 110 kbps)
    uint8_t drx_tune2[4] = {0x2D, 0x00, 0x1A, 0x31}; // 0x311A002D
    dw1000_WriteSubData(dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE2_SUB, drx_tune2, 4);

    // 4. LDE_CFG1
    uint8_t lde_cfg1 = 0x6D;
    dw1000_WriteSubData(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB, &lde_cfg1, 1);

    // 5. LDE_CFG2
    uint8_t lde_cfg2[2] = {0x07, 0x16}; // 0x1607
    dw1000_WriteSubData(dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG2_SUB, lde_cfg2, 2);

    // TX Power configuration (Required for DWM1000 module antenna)
    uint8_t tx_power[4] = {0x48, 0x28, 0x08, 0x0E}; // 0x0E082848 (Ch 5)
    dw1000_WriteData(dw1000, DW1000_TX_POWER, tx_power, 4);

    /* ----------------------------------------------------------------------
     * CRITICAL FIX: actually program the PHY for 110 kbps / 16 MHz PRF.
     *
     * On power-up the DW1000 defaults to 6.8 Mbps TX, 128-symbol preamble,
     * channel 5 (see DW1000 User Manual, "Default Configuration on Power
     * Up"). This function only ever tuned AGC/DRX_TUNE2/LDE for 110 kbps —
     * it never touched TX_FCTRL or SYS_CFG, so the radio was *transmitting*
     * at the chip's default 6.8 Mbps while the *receiver* front-end was
     * tuned for 110 kbps. That mismatch is exactly what produces endless
     * RXPREJ / RXPHE / RXSFDTO floods even at close range: the receiver's
     * digital tuning never matches what's actually arriving over the air.
     * ------------------------------------------------------------------- */

    // a) TX_FCTRL: TXBR=00 (110k), TXPRF=01 (16 MHz), TXPSR/PE=01/01 (128-symbol
    //    preamble, matching the PAC8 DRX_TUNE2 value already programmed above)
    uint8_t tx_fctrl[5];
    dw1000_ReadData(dw1000, DW1000_TX_FCTRL, tx_fctrl, 5);
    tx_fctrl[1] &= ~0x60;   // clear TXBR (bits 13-14) -> 110 kbps
    tx_fctrl[2] &= ~0x3F;   // clear TXPRF/TXPSR/PE (bits 16-21)
    tx_fctrl[2] |= 0x15;    // TXPRF=01 (16MHz) | TXPSR=01,PE=01 (128 symbols)
    dw1000_WriteData(dw1000, DW1000_TX_FCTRL, tx_fctrl, 5);

    // b) SYS_CFG: RXM110K MUST be set whenever you intend to receive at
    //    110 kbps — per the User Manual this is not automatic.
    uint8_t sys_cfg[DW1000_SYS_CFG_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);
    dw1000_SetBit(sys_cfg, DW1000_SYS_CFG_RXM110K, 1);
    dw1000_WriteData(dw1000, DW1000_SYS_CFG, sys_cfg, DW1000_SYS_CFG_LEN);

    // Verify written parameters to ensure sub-indexing works
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
    return ((uint32_t)buffer[3] << 24) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0];
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
    /* TFLEN must include the 2 auto-generated FCS octets, per the User
     * Manual: "this length value should include space for the FCS too". */
    uint16_t tflen = use_crc ? (length + 2) : length;

    uint8_t fctrl[DW1000_TX_FCTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);
    fctrl[0] = tflen & 0x00FF;
    fctrl[1] &= ~(0x03);
    fctrl[1] |= ((tflen >> 8) & 0x03);
    dw1000_WriteData(dw1000, DW1000_TX_FCTRL, fctrl, DW1000_TX_FCTRL_LEN);

    uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
    /* BUG FIX: SFCST means "Suppress auto-FCS Transmission". The old code
     * set SFCST = use_crc, so asking for a CRC (use_crc=1) actually told
     * the chip to SUPPRESS the auto-generated FCS on every frame. The two
     * extra trailing bytes that went out were then just whatever stale
     * stack data happened to sit in tx_buf[], which essentially never
     * matches the CRC the receiver computes -> guaranteed RXFCE/FCS
     * failure even on a frame that syncs perfectly. Polarity must be
     * inverted: use_crc=1 should mean SFCST=0 (let the chip append a
     * real FCS automatically). */
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
    /* NOTE: the auto-generated FCS is computed and appended by the chip
     * itself — it is never read out of TX_BUFFER — so only the real
     * payload bytes need to be written here. TFLEN (in dw1000_StartTransmit)
     * is what tells the hardware to add 2 more octets of FCS on the wire. */
    (void)use_crc;
    if(length > DW1000_TX_BUFFER_LEN) {
        return;
    }
    dw1000_WriteData(dw1000, DW1000_TX_BUFFER, data, length);
}

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc) {
    uint8_t sys_ctrl[DW1000_SYS_CTRL_LEN];
    dw1000_ReadData(dw1000, DW1000_SYS_CTRL, sys_ctrl, DW1000_SYS_CTRL_LEN);
    dw1000_SetBit(sys_ctrl, DW1000_SYS_CTRL_SFCST, use_crc);
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
