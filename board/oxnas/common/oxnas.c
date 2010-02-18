/*
 * (C) Copyright 2005
 * Oxford Semiconductor Ltd
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/reset.h>

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

#define FLASH_WORD_SIZE unsigned short

int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_arch_number = MACH_TYPE_OXNAS;
	gd->bd->bi_boot_params = PHYS_SDRAM_1_PA + 0x100;
	gd->flags = 0;

	icache_enable();

	/* Block reset Static core */
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_STATIC_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_STATIC_BIT);

	/* Enable clock to Static core */
	*(volatile u32 *)SYS_CTRL_CKEN_SET_CTRL =
	    (1UL << SYS_CTRL_CKEN_STATIC_BIT);

#ifdef CONFIG_OXNAS_USE_PCI
	/* Block reset PCI core */
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_PCI_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_PCI_BIT);

	/* Enable clock to PCI core */
	*(volatile u32 *)SYS_CTRL_CKEN_SET_CTRL =
	    (1UL << SYS_CTRL_CKEN_PCI_BIT);
#endif // CONFIG_OXNAS_USE_PCI

#ifdef CONFIG_OXNAS_MANUAL_STATIC_ARBITRATION
	/* Assert manual static bus PCI arbitration request */
	*(volatile u32 *)SYS_CTRL_PCI_CTRL1 |=
	    (1UL << SYS_CTRL_PCI_CTRL1_PCI_STATIC_RQ_BIT);
#endif // CONFIG_OXNAS_MANUAL_STATIC_ARBITRATION

#ifdef CONFIG_OXNAS_FEEDBACK_PCI_CLKS
	/* Set PCI feedback clk GPIO pin as an output */
	*(volatile u32 *)GPIO_1_SET_OE |= 0x800;

	/* Enable PCI feedback clk onto GPIO pin */
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 |= 0x00000800;
#endif // CONFIG_OXNAS_FEEDBACK_PCI_CLKS

#ifndef CONFIG_SYS_NO_FLASH
	/* Enable static bus onto GPIOs, only CS0 as CS1 conflicts with UART2 */
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 |= 0x002FF000;

	/* Setup the static bus CS0 to access FLASH */
	*(volatile u32 *)STATIC_CONTROL_BANK0 = STATIC_BUS_FLASH_CONFIG;

	/* Put flash into read array mode */
	*(FLASH_WORD_SIZE *) CFG_FLASH_BASE = (FLASH_WORD_SIZE) 0x00ff00ff;
#endif // !CONFIG_SYS_NO_FLASH

	/* Set 33MHz PCI clock */
	*(volatile u32 *)SYS_CTRL_CKCTRL_CTRL_ADDR = 5;
	/* Enable full speed RPS clock */
	*(volatile u32 *)SYS_CTRL_CKCTRL_CTRL_ADDR &=
	    ~(1UL << SYS_CTRL_CKCTRL_SLOW_BIT);

#ifndef EXTERNAL_UART
#ifdef CONFIG_OXNAS_UART1
	/* Block reset UART1 */
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART1_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART1_BIT);

	/* Setup pin mux'ing for first internal UART */
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x80000000;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_0 &= ~0x80000000;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |= 0x80000000;	// Route UART1 SOUT onto external pins

	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_1 &= ~0x00000001;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_1 &= ~0x00000001;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_1 |= 0x00000001;	// Route UART1 SIN onto external pins

	*(volatile u32 *)GPIO_1_SET_OE |= 0x80000000;	// Make UART1 SOUT an o/p
	*(volatile u32 *)GPIO_2_CLR_OE |= 0x00000001;	// Make UART1 SIN an i/p
#endif // CONFIG_OXNAS_UART1

#ifdef CONFIG_OXNAS_UART2
	// Block reset UART2
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART2_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART2_BIT);

	/* Setup pin mux'ing for second internal UART */
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x00500000;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_0 &= ~0x00500000;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |= 0x00500000;	// Route UART2 SOUT and SIN onto external pins

	*(volatile u32 *)GPIO_1_SET_OE |= 0x00100000;	// Make UART2 SOUT an o/p
	*(volatile u32 *)GPIO_1_CLR_OE |= 0x00400000;	// Make UART2 SIN an i/p
#endif // CONFIG_OXNAS_UART2

#ifdef CONFIG_OXNAS_UART3
	// Block reset UART3
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART3_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART3_BIT);

	// Route UART3 SIN/SOUT onto external pin
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 &= ~0x000000C0;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_0 &= ~0x000000C0;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_0 |= 0x000000C0;

	// Setup GPIO line directions for UART3 SIN/SOUT
	*(volatile u32 *)GPIO_1_SET_OE |= 0x00000080;
	*(volatile u32 *)GPIO_1_CLR_OE |= 0x00000040;
#endif // CONFIG_ARCH_OXNAS_UART3

#ifdef CONFIG_OXNAS_UART4
	// Block reset UART4
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART4_BIT);
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_UART4_BIT);

	// Enable UART4 to override PCI functions onto GPIOs
	*(volatile u32 *)SYS_CTRL_UART_CTRL |=
	    (1UL << SYS_CTRL_UART4_NOT_PCI_MODE);
#endif // CONFIG_OXNAS_UART4
#endif // !EXTERNAL_UART

	return 0;
}

int board_late_init()
{
#ifdef CONFIG_OXNAS_USE_SATA
	/* Start U-Boot's IDE support */
	ide_init();
#endif // CONFIG_OXNAS_USE_SATA

	return 0;
}

int misc_init_r(void)
{
	return 0;
}

int dram_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1_PA;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}
