/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * Copyright 2019 NXP
 *
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <mm/generic_ram_layout.h>

#define STACK_ALIGNMENT			64

/* console uart define */
#define CONSOLE_UART_BASE		UART0_BASE

/* Platform specific defines */
#if defined(PLATFORM_FLAVOR_ls1021aqds) || defined(PLATFORM_FLAVOR_ls1021atwr)
/*  DUART 1 */
#define UART0_BASE			0x021C0500
#define DCFG_BASE			0x01EE0000
#define DCFG_CCSR_BRR			0xE4
#define DCFG_SCRATCHRW1			0x200

#define CSU_BASE			0x01510000
#define CSU_CSL_START			0x0
#define CSU_CSL_END			0xE8
#define CSU_CSL30			0x78
#define CSU_CSL37			0x94

/* Central Security Unit register values */
#define	CSU_ACCESS_ALL			0x00FF00FF
#define	CSU_ACCESS_SEC_ONLY		0x003F003F
#define CSU_SETTING_LOCK		0x01000100

#define GIC_BASE			0x01400000
#define GICC_OFFSET			0x2000
#define GICD_OFFSET			0x1000
#endif

#if defined(PLATFORM_FLAVOR_ls1043ardb)
 /*  DUART 1 */
 #define UART0_BASE			0x021C0500
 #define GIC_BASE			0x01400000
#define GICC_4K_ADDR_OFFSET		0x2000
#define GICD_4K_ADDR_OFFSET		0x1000
#define GICC_64K_ADDR_OFFSET		0x20000
#define GICD_64K_ADDR_OFFSET		0x10000
#endif

#if defined(PLATFORM_FLAVOR_ls1046ardb) || defined(PLATFORM_FLAVOR_ls1012ardb)\
|| defined(PLATFORM_FLAVOR_ls1012afrwy)

/*  DUART 1 */
#define UART0_BASE			0x021C0500
#define GIC_BASE			0x01400000
#define GICC_OFFSET			0x20000
#define GICD_OFFSET			0x10000
#endif

#if defined(PLATFORM_FLAVOR_ls1088ardb)
/*  DUART 1 */
#define UART0_BASE			0x021C0500
#define GIC_BASE			0x06000000
#define GICD_OFFSET			0x0
#endif

#if defined(PLATFORM_FLAVOR_ls2088ardb)
/*  DUART 1 */
#define UART0_BASE			0x021C0600
#define GIC_BASE			0x06000000
#define GICD_OFFSET			0x0
#endif

#if defined(PLATFORM_FLAVOR_ls1028ardb)
/*  DUART 1 */
#define UART0_BASE			0x021C0600
#define GIC_BASE			0x06000000
#define GICC_OFFSET			0x0
#define GICD_OFFSET			0x0
#endif

#if defined(PLATFORM_FLAVOR_lx2160ardb)
/*  DUART 1 */
#define UART0_BASE			0x021C0000
#define CONSOLE_BAUDRATE		0x1C200
/* As per LX2 Clock tree, UART clock is 1/4th of the platform clock.
 * Currently hard-coding for Platform clock = 700MHz.
 */
#define CONSOLE_UART_CLK_IN_HZ		0xA6E49C0
#define UART1_BASE			0x021D0000
#define GIC_BASE			0x06000000
#define GICD_OFFSET			0x0
#endif

#endif /*PLATFORM_CONFIG_H*/
