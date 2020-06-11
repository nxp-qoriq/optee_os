// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * DSPI Controller
 */

#include <spi.h>

/* SPI register offset */
#define DSPI_MCR		0x0	//Module Configuration Register
#define DSPI_TCR		0x8	//Transfer Count Register
#define DSPI_CTAR0		0xC	//Clock and Transfer Attributes Register (in Master mode)
#define DSPI_CTAR1		0x10	//Clock and Transfer Attributes Register (in Master mode)
#define DSPI_SR			0x2C	//Status Register
#define DSPI_RSER		0x30	//DMA/Interrupt Request Select and Enable Register
#define DSPI_PUSHR		0x34	//PUSH TX FIFO Register In Master Mode
#define DSPI_POPR		0x38	//POP RX FIFO Register
#define DSPI_TXFR0		0x3C	//Transmit FIFO Registers
#define DSPI_TXFR1		0x40	//Transmit FIFO Registers
#define DSPI_TXFR2		0x44	//Transmit FIFO Registers
#define DSPI_TXFR3		0x48	//Transmit FIFO Registers
#define DSPI_RXFR0		0x7C	//Receive FIFO Registers
#define DSPI_RXFR1		0x80	//Receive FIFO Registers
#define DSPI_RXFR2		0x84	//Receive FIFO Registers
#define DSPI_RXFR3		0x88	//Receive FIFO Registers
#define DSPI_CTARE0		0x11C	//Clock and Transfer Attributes Register Extended
#define DSPI_CTARE1		0x120	//Clock and Transfer Attributes Register Extended
#define DSPI_SREX		0x13C	//Status Register Extended

/* Module configuration */
#define DSPI_MCR_MSTR		0x80000000	//Master/Slave Mode Select [0]
#define DSPI_MCR_CSCK		0x40000000	//Continuous SCK Enable [1]
#define DSPI_MCR_DCONF(x)	(((x) & 0x03) << 28)	//SPI Configuration [2-3]
#define DSPI_MCR_ROOE		0x01000000	//Receive FIFO Overflow Overwrite Enable [7]
#define DSPI_MCR_PCSIS(x)	(1 << (16 + (x)))	//Peripheral Chip Select x Inactive State [12-15]
#define DSPI_MCR_PCSIS_MASK	(0xff << 16)
#define DSPI_MCR_MDIS		0x00004000	//Module Disable [17]
#define DSPI_MCR_DTXF		0x00002000	//Disable Transmit FIFO [18]
#define DSPI_MCR_DRXF		0x00001000	//Disable Receive FIFO [19]
#define DSPI_MCR_CTXF		0x00000800	//Clear TX FIFO [20]
#define DSPI_MCR_CRXF		0x00000400	//Clear RX FIFO [21]
#define DPSI_XSPI		0x00000008	//Extended SPI Mode [28]
#define DSPI_MCR_PES		0x00000002	//Parity Error Stop [30]
#define DSPI_MCR_HALT		0x00000001	//Halt [31]
#define DPSI_ENABLE		0x0
#define DSPI_DISABLE		0x1

/* Transfer count */
#define DSPI_TCR_SPI_TCNT(x)	(((x) & 0x0000FFFF) << 16)

/* Clock and transfer attributes */
#define DSPI_CTAR_DBR		0x80000000		//Double Baud Rate [0]
#define DSPI_CTAR_FMSZ(x)	(((x) & 0x0F) << 27)	//Frame Size [1-4]
#define DSPI_CTAR_CPOL		0x04000000		//Clock Polarity [5]
#define DSPI_CTAR_CPHA		0x02000000		//Clock Phase [6]
#define DSPI_CTAR_LSBFE		0x01000000		//LSB First [7]
#define DSPI_CTAR_PCSSCK(x)	(((x) & 0x03) << 22)	//PCS to SCK Delay Prescaler [8-9]
#define DSPI_CTAR_PCSSCK_7CLK	0x00A00000
#define DSPI_CTAR_PCSSCK_5CLK	0x00800000
#define DSPI_CTAR_PCSSCK_3CLK	0x00400000
#define DSPI_CTAR_PCSSCK_1CLK	0x00000000
#define DSPI_CTAR_PASC(x)	(((x) & 0x03) << 20)	//After SCK Delay Prescaler [10-11]
#define DSPI_CTAR_PASC_7CLK	0x00300000
#define DSPI_CTAR_PASC_5CLK	0x00200000
#define DSPI_CTAR_PASC_3CLK	0x00100000
#define DSPI_CTAR_PASC_1CLK	0x00000000
#define DSPI_CTAR_PDT(x)	(((x) & 0x03) << 18)	//Delay after Transfer Prescaler [12-13]
#define DSPI_CTAR_PDT_7CLK	0x000A0000
#define DSPI_CTAR_PDT_5CLK	0x00080000
#define DSPI_CTAR_PDT_3CLK	0x00040000
#define DSPI_CTAR_PDT_1CLK	0x00000000
#define DSPI_CTAR_PBR(x)	(((x) & 0x03) << 16)	//Baud Rate Prescaler [14-15]
#define DSPI_CTAR_PBR_7CLK	0x00030000
#define DSPI_CTAR_PBR_5CLK	0x00020000
#define DSPI_CTAR_PBR_3CLK	0x00010000
#define DSPI_CTAR_PBR_1CLK	0x00000000
#define DSPI_CTAR_CSSCK(x)	(((x) & 0x0F) << 12)	//PCS to SCK Delay Scaler [16-19]
#define DSPI_CTAR_ASC(x)	(((x) & 0x0F) << 8)	//After SCK Delay Scaler [20-23]
#define DSPI_CTAR_DT(x)		(((x) & 0x0F) << 4)	//Delay After Transfer Scaler [24-27]
#define DSPI_CTAR_BR(x)		((x) & 0x0F)		//Baud Rate Scaler [28-31]

/* Status */
#define DSPI_SR_TCF		0x80000000		//Transfer Complete Flag [0]
#define DSPI_SR_TXRXS		0x40000000		//TX and RX Status [1]
#define DSPI_SR_EOQF		0x10000000		//End of Queue Flag [3]
#define DSPI_SR_TFFF		0x02000000		//Transmit FIFO Fill Flag [6]
#define DSPI_SR_BSYF		0x01000000		//Busy Flag [7]
#define DSPI_SR_CMDTCF		0x00800000		//Command Transfer Complete Flag [8]
#define DSPI_SR_SPEF		0x00200000		//SPI Parity Error Flag [10]
#define DSPI_SR_RFOF		0x00080000		//Receive FIFO Overflow Flag [12]
#define DSPI_SR_TFIWF		0x00040000		//Transmit FIFO Invalid Write Flag [13]
#define DSPI_SR_RFDF		0x00020000		//Receive FIFO Drain Flag [14]
#define DSPI_SR_CMDFFF		0x00010000		//Command FIFO Fill Flag [15]
#define DSPI_SR_TXCTR(x)	(((x) & 0x0000F000) >> 12)		//TX FIFO Counter [16-19]
#define DSPI_SR_TXNXTPTR(x)	(((x) & 0x00000F00) >> 8)		//Transmit Next Pointer [20-23]
#define DSPI_SR_RXCTR(x)	(((x) & 0x000000F0) >> 4)		//RX FIFO Counter [24-27]
#define DSPI_SR_POPNXTPTR(x)	((x) & 0x0000000F)			//Pop Next Pointer [28-31]

/* DMA/interrupt request select and enable */
#define DSPI_RSER_TCF_RE	0x80000000		//Transmission Complete Request Enable [0]
#define DSPI_RSER_CMDFFF_RE	0x40000000		//Command FIFO Fill Flag Request Enable [1]
#define DSPI_RSER_EOQF_RE	0x10000000		//Finished Request Enable [3]
#define DSPI_RSER_TFFF_RE	0x02000000		//Transmit FIFO Fill Request Enable [6]
#define DSPI_RSER_TFFF_DIRS	0x01000000		//Transmit FIFO Fill DMA or Interrupt Request Select[7]
#define DSPI_RSER_RFOF_RE	0x00080000		//Receive FIFO Overflow Request Enable [12]
#define DSPI_RSER_RFDF_RE	0x00020000		//Receive FIFO Drain Request Enable [14]
#define DSPI_RSER_RFDF_DIRS	0x00010000		//Receive FIFO Drain DMA or Interrupt Request Select [15]

#define DSPI_DATA_8BIT		SHIFT_U32(8, 0)
#define DSPI_DATA_16BIT		SHIFT_U32(0xF, 0)

#define DSPI_TFR_CONT		(0x80000000)
#define DSPI_TFR_CTAS(x)	(((x)&0x07)<<12)
#define DSPI_TFR_PCS(x)		(((1 << x) & 0x0000003f) << 16)

/* tx/rx data wait timeout value, unit: us */
#define DSPI_TXRX_WAIT_TIMEOUT	1000000

/* Transfer Fifo */
#define DSPI_TFR_TXDATA(x)	(((x)&0xFFFF))

/* Bit definitions and macros for DRFR */
#define DSPI_RFR_RXDATA(x)	(((x)&0xFFFF))

/* CTAR register pre-configure value */
#define DSPI_CTAR_DEFAULT_VALUE		(DSPI_CTAR_FMSZ(7) | \
					DSPI_CTAR_PCSSCK_1CLK | \
					DSPI_CTAR_PASC(0) | \
					DSPI_CTAR_PDT(0) | \
					DSPI_CTAR_CSSCK(0) | \
					DSPI_CTAR_ASC(0) | \
					DSPI_CTAR_DT(0))

/* CTAR register pre-configure mask */
#define DSPI_CTAR_SET_MODE_MASK		(DSPI_CTAR_FMSZ(15) | \
					DSPI_CTAR_PCSSCK(3) | \
					DSPI_CTAR_PASC(3) | \
					DSPI_CTAR_PDT(3) | \
					DSPI_CTAR_CSSCK(15) | \
					DSPI_CTAR_ASC(15) | \
					DSPI_CTAR_DT(15))

#define CONFIG_SYS_DSPI_CTAR0	1

/* SPI mode flags */
#define SPI_CPHA		BIT(0)	/* clock phase */
#define SPI_CPOL		BIT(1)	/* clock polarity */
#define SPI_LSB_FIRST		BIT(3)	/* per-word bits-on-wire */
#define SPI_CS_HIGH		BIT(2)	/* CS active high */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


/* max chipselect signals number */
#define FSL_DSPI_MAX_CHIPSELECT			6

/* default SCK frequency, unit: HZ */
#define PLATFORM_CLK		650000000
#define DSPI_DEFAULT_SCK_FREQ	10000000
#define DSPI_CLK_DIV		2	/* prescaler divisor*/
#define DSPI_CLK		(PLATFORM_CLK/DSPI_CLK_DIV)	/* DSPI clock*/
#define CS_SPEED_MAX_HZ		1000000	/* Slave max speed*/

struct nxp_dspi_data {
	struct spi_chip		chip;
	vaddr_t			base;			/* DSPI controller base address */
	enum spi_mode		mode;			/* SPI mode to use for slave device */
	unsigned int		bus_clk_hz;		/* DSPI input clk frequency */
	unsigned int		speed_hz;		/* Default SCK frequency=1mhz */
	unsigned int		num_chipselect;		/* chip/slave selection */
	unsigned int		num_bus;		/* bus selection */
	unsigned int		slave_bus;
	unsigned int		slave_cs;
	unsigned int		slave_speed_max_hz;
	unsigned int		slave_mode;
	unsigned int		slave_data_size_bits;	/* Data size to be transferred (8 or 16 bits) */
	unsigned int		ctar_val[FSL_DSPI_MAX_CHIPSELECT];
};

void dspi_test(void);
