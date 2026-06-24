/*
 * dw1000.h
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 */

#ifndef DW1000_H_
#define DW1000_H_

#include <stdint.h>
#include "main.h"
#include "dw1000_time.h"

// Register map and length
#define DW1000_DEV_ID 0x00
#define DW1000_DEV_ID_LEN 4
#define DW1000_EUI 0x01
#define DW1000_EUI_LEN 8
#define DW1000_PANADR 0x03
#define DW1000_PANADR_LEN 4
#define DW1000_SYS_CFG 0x04
#define DW1000_SYS_CFG_LEN 4
#define DW1000_SYS_TIME 0x06
#define DW1000_SYS_TIME_LEN 5
#define DW1000_TX_FCTRL 0x08
#define DW1000_TX_FCTRL_LEN 5
#define DW1000_TX_BUFFER 0x09
#define DW1000_TX_BUFFER_LEN 1024
#define DW1000_DX_TIME 0x0A
#define DW1000_DX_TIME_LEN 5
#define DW1000_RX_FWTO 0x0C
#define DW1000_RX_FWTO_LEN 2
#define DW1000_SYS_CTRL 0x0D
#define DW1000_SYS_CTRL_LEN 4
#define DW1000_SYS_MASK 0x0E
#define DW1000_SYS_MASK_LEN 4
#define DW1000_SYS_STATUS 0x0F
#define DW1000_SYS_STATUS_LEN 5
#define DW1000_RX_FINFO 0x10
#define DW1000_RX_FINFO_LEN 4
#define DW1000_RX_BUFFER 0x11
#define DW1000_RX_BUFFER_LEN 1024
#define DW1000_RX_FQUAL 0x12
#define DW1000_RX_FQUAL_LEN 8
#define DW1000_RX_TTCKI 0x13
#define DW1000_RX_TTCKI_LEN 4
#define DW1000_RX_TTCKO 0x14
#define DW1000_RX_TTCKO_LEN 5
#define DW1000_RX_TIME 0x15
#define DW1000_RX_TIME_LEN 14
#define DW1000_TX_TIME 0x17
#define DW1000_TX_TIME_LEN 10
#define DW1000_TX_ANTD 0x18
#define DW1000_TX_ANTD_LEN 2
#define DW1000_SYS_STATE 0x19
#define DW1000_SYS_STATE_LEN 5
#define DW1000_ACK_RESP_T 0x1A
#define DW1000_ACK_RESP_T_LEN 4
#define DW1000_RX_SNIFF 0x1D
#define DW1000_RX_SNIFF_LEN 4
#define DW1000_TX_POWER 0x1E
#define DW1000_TX_POWER_LEN 4
#define DW1000_CHAN_CTRL 0x1F
#define DW1000_CHAN_CTRL_LEN 4
#define DW1000_USR_SFD 0x21
#define DW1000_USR_SFD_LEN 41
#define DW1000_AGC_CTRL 0x23
#define DW1000_AGC_CTRL_LEN 33
#define DW1000_EXT_SYNC 0x24
#define DW1000_EXT_SYNC_LEN 12
#define DW1000_ACC_MEM 0x25
#define DW1000_ACC_MEM_LEN 4064
#define DW1000_GPIO_CTRL 0x26
#define DW1000_GPIO_CTRL_LEN 44
#define DW1000_DRX_CONF 0x27
#define DW1000_DRX_CONF_LEN 44
#define DW1000_RF_CONF 0x28
#define DW1000_RF_CONF_LEN 58
#define DW1000_TX_CAL 0x2A
#define DW1000_TX_CAL_LEN 52
#define DW1000_FS_CTRL 0x2B
#define DW1000_FS_CTRL_LEN 21
#define DW1000_AON 0x2C
#define DW1000_AON_LEN 12
#define DW1000_OTP_IF 0x2D
#define DW1000_OTP_IF_LEN 18
#define DW1000_LDE_CTRL 0x2E
#define DW1000_DIG_DIAG 0x2F
#define DW1000_DIG_DIAG_LEN 41
#define DW1000_PSMC 0x36
#define DW1000_PSMC_LEN 48

/* ----------------------------------------------------------------------
 * Sub-register offsets and tuning constants used by dw1000_Init()/tune().
 * Values taken from the DW1000 User Manual "Default configuration that
 * should be modified" table and cross-checked against the tested
 * thotro/arduino-dw1000 and DecaWave mbed driver tune() sequences.
 * ---------------------------------------------------------------------- */

/* Register file 0x23 - AGC_CTRL */
#define DW1000_AGC_TUNE1_SUB   0x04   /* 16-bit */
#define DW1000_AGC_TUNE2_SUB   0x0C   /* 32-bit */
#define DW1000_AGC_TUNE3_SUB   0x12   /* 16-bit */
#define DW1000_AGC_TUNE1_16MHZ 0x8870u
#define DW1000_AGC_TUNE1_64MHZ 0x889Bu
#define DW1000_AGC_TUNE2_VAL   0x2502A907ul
#define DW1000_AGC_TUNE3_VAL   0x0035u

/* Register file 0x27 - DRX_CONF */
#define DW1000_DRX_TUNE0b_SUB  0x02   /* 16-bit */
#define DW1000_DRX_TUNE1a_SUB  0x04   /* 16-bit */
#define DW1000_DRX_TUNE1b_SUB  0x06   /* 16-bit */
#define DW1000_DRX_TUNE2_SUB   0x08   /* 32-bit */
#define DW1000_DRX_TUNE4H_SUB  0x26   /* 16-bit */
#define DW1000_DRX_SFDTOC_SUB  0x20   /* 16-bit, SFD detection timeout */

/* DRX_TUNE0b: data-rate dependent (here: 110 kbps with standard SFD) */
#define DW1000_DRX_TUNE0b_110K_STD 0x000Au
#define DW1000_DRX_TUNE0b_850K_STD 0x0001u
#define DW1000_DRX_TUNE0b_6M8_STD  0x0001u

/* DRX_TUNE1a: PRF dependent */
#define DW1000_DRX_TUNE1a_PRF16 0x0087u
#define DW1000_DRX_TUNE1a_PRF64 0x008Du

/* DRX_TUNE1b: preamble-length / data-rate dependent */
#define DW1000_DRX_TUNE1b_LONG   0x0064u /* preamble >= 1024 symbols, 110 kbps */
#define DW1000_DRX_TUNE1b_MED    0x0020u /* 128 <= preamble < 1024 */
#define DW1000_DRX_TUNE1b_SHORT  0x0010u /* preamble == 64, 6.8 Mbps only */

/* DRX_TUNE2: PRF and PAC size dependent (PAC=8, PRF16 shown; this is the
 * commonly used value for preamble 1024/2048 PAC=8) */
#define DW1000_DRX_TUNE2_PAC8_PRF16  0x311A002Dul
#define DW1000_DRX_TUNE2_PAC8_PRF64  0x313B006Bul

/* DRX_TUNE4H: preamble length dependent */
#define DW1000_DRX_TUNE4H_LONG  0x0028u /* preamble > 64 */
#define DW1000_DRX_TUNE4H_SHORT 0x0010u /* preamble == 64 */

/* Register file 0x2E - LDE_CTRL (sub-registers use 2-byte extended sub-addr) */
#define DW1000_LDE_CFG1_SUB    0x0806 /* 8-bit */
#define DW1000_LDE_CFG2_SUB    0x1806 /* 16-bit */
#define DW1000_LDE_REPC_SUB    0x2804 /* 16-bit */
#define DW1000_LDE_CFG1_VAL    0x6Du  /* NTM=13(0xD), PMULT=0b011<<5 -> 0x6D */
#define DW1000_LDE_CFG2_PRF16  0x1607u
#define DW1000_LDE_CFG2_PRF64  0x0607u

/* Register file 0x28 - RF_CONF */
#define DW1000_RF_RXCTRLH_SUB  0x0B   /* 8-bit */
#define DW1000_RF_TXCTRL_SUB   0x0C   /* 32-bit */
#define DW1000_RF_RXCTRLH_VAL_CH5 0xD8u
#define DW1000_RF_TXCTRL_VAL_CH5  0x001E3FE0ul

/* Register file 0x2A - TX_CAL */
#define DW1000_TC_PGDELAY_SUB  0x0B   /* 8-bit */
#define DW1000_TC_PGDELAY_VAL_CH5 0xC0u
#define DW1000_TC_PGCCTRL_SUB  0x07   /* 8-bit, kick off PG count */

/* Register file 0x2B - FS_CTRL */
#define DW1000_FS_PLLTUNE_SUB  0x0B   /* 8-bit */
#define DW1000_FS_PLLCFG_SUB   0x07   /* 32-bit */
#define DW1000_FS_XTALT_SUB    0x0E   /* 8-bit */
#define DW1000_FS_PLLTUNE_VAL_CH5 0xA6u
#define DW1000_FS_PLLCFG_VAL_CH5  0x0800041Dul
#define DW1000_FS_XTALT_MIDRANGE  0x10u /* default 0b10000, trim value */

/* Register file 0x2D - OTP_IF */
#define DW1000_OTP_ADDR_SUB    0x04   /* 16-bit */
#define DW1000_OTP_CTRL_SUB    0x06   /* 16-bit */
#define DW1000_OTP_RDAT_SUB    0x0A   /* 32-bit */
#define DW1000_OTP_CTRL_OTPREAD 0x0002u
#define DW1000_OTP_CTRL_OTPRDEN 0x0001u

/* Register file 0x36 - PMSC (Power management/system control) */
#define DW1000_PMSC_CTRL0_SUB  0x00   /* 32-bit */
#define DW1000_PMSC_CTRL1_SUB  0x04   /* 32-bit */
#define DW1000_PMSC_CTRL0_SYSCLKS_19M  0x01u
#define DW1000_PMSC_CTRL0_SYSCLKS_125M 0x02u
#define DW1000_PMSC_CTRL0_SYSCLKS_AUTO 0x00u
#define DW1000_PMSC_CTRL0_FACE         (1u << 6)
#define DW1000_PMSC_CTRL1_LDE_BIT      17 /* PMSC_CTRL1.LDERUNE */

/* Register file 0x1E - TX_POWER */
#define DW1000_TX_POWER_MAN_DEFAULT 0x0E082848ul /* smart-TX off, ch5 */

/* PHR mode / data rate fields within TX_FCTRL (register 0x08) */
#define DW1000_TX_FCTRL_TXBR_110K  0x00u
#define DW1000_TX_FCTRL_TXBR_850K  0x01u
#define DW1000_TX_FCTRL_TXBR_6M8   0x02u
#define DW1000_TX_FCTRL_TXPRF_16M  0x01u
#define DW1000_TX_FCTRL_TXPRF_64M  0x02u

typedef enum {
  DW1000_SYS_CTRL_SFCST = 0,
  DW1000_SYS_CTRL_TXSTRT = 1,
  DW1000_SYS_CTRL_TXDLYS = 2,
  DW1000_SYS_CTRL_CANSFCS = 3,
  DW1000_SYS_CTRL_TRXOFF = 6,
  DW1000_SYS_CTRL_WAIT4RESP = 7,
  DW1000_SYS_CTRL_RXENAB = 8,
  DW1000_SYS_CTRL_RXDLYE = 9,
  DW1000_SYS_CTRL_HRBPT = 24
} dw1000_sys_ctrl_bit_t;

typedef enum {
  DW1000_SYS_STATUS_IRQS = 0,
  DW1000_SYS_STATUS_CPLOCK,
  DW1000_SYS_STATUS_ESYNCR,
  DW1000_SYS_STATUS_AAT,
  DW1000_SYS_STATUS_TXFRB,
  DW1000_SYS_STATUS_TXPRS,
  DW1000_SYS_STATUS_TXPHS,
  DW1000_SYS_STATUS_TXFRS,
  DW1000_SYS_STATUS_RXPRD,	// 8
  DW1000_SYS_STATUS_RXSFDD,
  DW1000_SYS_STATUS_LDEDONE,
  DW1000_SYS_STATUS_RXPHD,
  DW1000_SYS_STATUS_RXPHE,
  DW1000_SYS_STATUS_RXDFR,
  DW1000_SYS_STATUS_RXFCG,
  DW1000_SYS_STATUS_RXFCE,
  DW1000_SYS_STATUS_RXRFSL,	// 16
  DW1000_SYS_STATUS_PXRFTO,
  DW1000_SYS_STATUS_LDEERR,
  DW1000_SYS_STATUS_RXOVRR = 20,
  DW1000_SYS_STATUS_RXPTO,
  DW1000_SYS_STATUS_GPIOIRQ,
  DW1000_SYS_STATUS_SLP2INIT,
  DW1000_SYS_STATUS_RFPLL_LL,	// 24
  DW1000_SYS_STATUS_CLKPLL_LL,
  DW1000_SYS_STATUS_RXSFDTO,
  DW1000_SYS_STATUS_HPDWARN,
  DW1000_SYS_STATUS_TXBERR,
  DW1000_SYS_STATUS_AFFREJ,
  DW1000_SYS_STATUS_HSRBP,
  DW1000_SYS_STATUS_ICRBP,
  DW1000_SYS_STATUS_RXRSCS,	// 32
  DW1000_SYS_STATUS_RXPREJ,
  DW1000_SYS_STATUS_TXPUTE
} dw1000_sys_status_bit_t;

typedef enum {
  DW1000_SYS_CFG_FFEN = 0,
  DW1000_SYS_CFG_FFBC,
  DW1000_SYS_CFG_FFAB,
  DW1000_SYS_CFG_FFAD,
  DW1000_SYS_CFG_FFAA,
  DW1000_SYS_CFG_FFAM,
  DW1000_SYS_CFG_FFAR,
  DW1000_SYS_CFG_FFA4,
  DW1000_SYS_CFG_FFA5,  // 8
  DW1000_SYS_CFG_HIRQ_POL,
  DW1000_SYS_CFG_SPI_EDGE,
  DW1000_SYS_CFG_DIS_FCE,
  DW1000_SYS_CFG_DIS_DRXB,
  DW1000_SYS_CFG_DIS_PHE,
  DW1000_SYS_CFG_DIS_RSDE,
  DW1000_SYS_CFG_INIT2F,
  DW1000_SYS_CFG_PHR_MODE,  // 16
  DW1000_SYS_CFG_DIS_STXP = 18,
  DW1000_SYS_CFG_RXM110K = 22,
  DW1000_SYS_CFG_RXWTOE = 28,
  DW1000_SYS_CFG_RXAUTR,
  DW1000_SYS_CFG_AUTOACK,
  DW1000_SYS_CFG_AACKPEND
} dw1000_sys_cfg_bit_t;

typedef enum {
  DW1000_PRF_16MHZ = 0b01,
  DW1000_PRF_64MHZ = 0b10
} dw1000_prf_t;

typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *ss_port;
  uint16_t ss_pin;
} dw1000_HandleTypeDef;

typedef struct {
  uint8_t rev;
  uint8_t ver;
  uint8_t model;
  uint16_t ridtag;
} dw1000_dev_id_t;

typedef struct {
  uint16_t pan_id;
  uint16_t short_addr;
} dw1000_pan_addr_t;

typedef struct {
  uint8_t tx_chan;
  uint8_t rx_chan;
  dw1000_prf_t prf;
  uint8_t tx_preamble;
  uint8_t rx_preamble;
} dw1000_channel_t;

void dw1000_SetBit(uint8_t *data, uint8_t bitnum, uint8_t value);
uint8_t dw1000_IsSet(uint8_t *data, uint8_t bitnum);

void dw1000_WriteData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len);
void dw1000_ReadData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len);

dw1000_dev_id_t dw1000_GetDevID(dw1000_HandleTypeDef *dw1000);
dw1000_pan_addr_t dw1000_GetPanAddress(dw1000_HandleTypeDef *dw1000);
void dw1000_SetPanAddress(dw1000_HandleTypeDef *dw1000, dw1000_pan_addr_t *pan_addr);
dw1000_channel_t dw1000_GetChannel(dw1000_HandleTypeDef *dw1000);
void dw1000_SetChannel(dw1000_HandleTypeDef *dw1000, dw1000_channel_t *channel);
dw1000_timestamp_t dw1000_GetSystemTimeCounter(dw1000_HandleTypeDef *dw1000);
uint32_t dw1000_GetReceiverTimeTrackingInterval(dw1000_HandleTypeDef *dw1000);
void dw1000_ClearAllStatus(dw1000_HandleTypeDef *dw1000);

void dw1000_IdleMode(dw1000_HandleTypeDef *dw1000);

void dw1000_StartTransmit(dw1000_HandleTypeDef *dw1000, uint16_t length, uint8_t use_crc);
void dw1000_ClearTransmitStatus(dw1000_HandleTypeDef *dw1000);

void dw1000_StartReceive(dw1000_HandleTypeDef *dw1000, uint8_t use_crc);
void dw1000_ClearReceiveStatus(dw1000_HandleTypeDef *dw1000);
void dw1000_ReceiveAutoEnable(dw1000_HandleTypeDef *dw1000, uint8_t val);
uint8_t dw1000_ReceiveDataFrameReady(dw1000_HandleTypeDef *dw1000);

void dw1000_SetDataToTransmit(dw1000_HandleTypeDef *dw1000, uint8_t *data, uint16_t length, uint8_t use_crc);
uint16_t dw1000_GetDataReceivedLength(dw1000_HandleTypeDef *dw1000, uint8_t use_crc);
void dw1000_GetDataReceived(dw1000_HandleTypeDef *dw1000, uint8_t* data, uint16_t length);

/* ---- Extended (sub-register) addressing ---- */
void dw1000_WriteSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t *data, uint16_t len);
void dw1000_ReadSubData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t *data, uint16_t len);
void dw1000_WriteSubReg8(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint8_t value);
uint8_t dw1000_ReadSubReg8(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr);
void dw1000_WriteSubReg16(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint16_t value);
uint16_t dw1000_ReadSubReg16(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr);
void dw1000_WriteSubReg32(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr, uint32_t value);
uint32_t dw1000_ReadSubReg32(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint16_t subaddr);

/* ---- Full initialization / tuning sequence ----
 * Performs: PMSC soft reset, LDE microcode load, AGC/DRX/RF/TX_CAL/FS_CTRL
 * tuning register writes for the requested channel/PRF/data-rate, and
 * configures TX_FCTRL data-rate/PRF bits and SYS_CFG for the chosen frame
 * length mode. Must be called once after dw1000_GetDevID() succeeds and
 * before any TX/RX activity. Returns 1 on success, 0 if a sanity read-back
 * check fails (useful for diagnostics).
 */
uint8_t dw1000_Init(dw1000_HandleTypeDef *dw1000, uint8_t channel, dw1000_prf_t prf, uint8_t data_rate);

/* ---- Diagnostics ---- */
/* Reads SYS_STATUS (40-bit) into a 5-byte buffer */
void dw1000_GetSystemStatus(dw1000_HandleTypeDef *dw1000, uint8_t *status_out5);
/* Reads SYS_STATE (24-bit, in 5-byte register) - reports current TX/RX/PMSC FSM state */
uint32_t dw1000_GetSystemState(dw1000_HandleTypeDef *dw1000);

#endif /* DW1000_H_ */
