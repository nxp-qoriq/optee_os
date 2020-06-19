// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * Helper Code for DSPI Controller driver
 */

#include <spi.h>
#include <mm/core_memprot.h>

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
#define DSPI_CTAR_BRD		0x80000000		//Double Baud Rate [0]
#define DSPI_CTAR_FMSZ(x)	(((x) & 0x0F) << 27)	//Frame Size [1-4]
#define DSPI_CTAR_CPOL		0x04000000		//Clock Polarity [5]
#define DSPI_CTAR_CPHA		0x02000000		//Clock Phase [6]
#define DSPI_CTAR_LSBFE		0x01000000		//LSB First [7]
#define DSPI_CTAR_BRP(x)	(((x) & 0x03) << 16)	//Baud Rate Prescaler [14-15]
#define DSPI_CTAR_BR(x)		((x) & 0x0F)		//Baud Rate Scaler [28-31]
#define DSPI_CTAR_PCS_SCK(x)	(((x) & 0x03) << 22)
#define DSPI_CTAR_PA_SCK(x)	(((x) & 0x03) << 20)
#define DSPI_CTAR_P_DT(x)	(((x) & 0x03) << 18)
#define DSPI_CTAR_CS_SCK(x)	(((x) & 0x0F) << 12)
#define DSPI_CTAR_A_DT(x)	(((x) & 0x0F) << 4)
#define DSPI_CTAR_A_SCK(x)	(((x) & 0x0F) << 8)
#define DSPI_CTAR_PCS_SCK_1CLK	0x00000000

/* Status */
#define DSPI_SR_TXRXS		0x40000000		//TX and RX Status [1]
#define DSPI_SR_TXCTR(x)	(((x) & 0x0000F000) >> 12)		//TX FIFO Counter [16-19]
#define DSPI_SR_RXCTR(x)	(((x) & 0x000000F0) >> 4)		//RX FIFO Counter [24-27]

#define DSPI_DATA_8BIT		SHIFT_U32(8, 0)
#define DSPI_DATA_16BIT		SHIFT_U32(0xF, 0)

#define DSPI_TFR_CONT		(0x80000000)
#define DSPI_TFR_CTAS(x)	(((x)&0x07)<<12)
#define DSPI_TFR_PCS(x)		(((1 << x) & 0x0000003f) << 16)
#define DSPI_IDLE_DATA		0x0

/* tx/rx data wait timeout value, unit: us */
#define DSPI_TXRX_WAIT_TIMEOUT	1000000

/* Transfer Fifo */
#define DSPI_TFR_TXDATA(x)	(((x) & 0x0000FFFF))

/* Bit definitions and macros for DRFR */
#define DSPI_RFR_RXDATA(x)	(((x) & 0x0000FFFF))

/* CTAR register pre-configure value */
#define DSPI_CTAR_DEFAULT_VALUE		(DSPI_CTAR_FMSZ(7) | \
					DSPI_CTAR_PCS_SCK_1CLK | \
					DSPI_CTAR_PA_SCK(0) | \
					DSPI_CTAR_P_DT(0) | \
					DSPI_CTAR_CS_SCK(0) | \
					DSPI_CTAR_A_SCK(0) | \
					DSPI_CTAR_A_DT(0))

/* CTAR register pre-configure mask */
#define DSPI_CTAR_SET_MODE_MASK		(DSPI_CTAR_FMSZ(15) | \
					DSPI_CTAR_PCS_SCK(3) | \
					DSPI_CTAR_PA_SCK(3) | \
					DSPI_CTAR_P_DT(3) | \
					DSPI_CTAR_CS_SCK(15) | \
					DSPI_CTAR_A_SCK(15) | \
					DSPI_CTAR_A_DT(15))

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

TEE_Result nxp_dspi_init(struct nxp_dspi_data *dspi_data);
void dspi_test(void);
