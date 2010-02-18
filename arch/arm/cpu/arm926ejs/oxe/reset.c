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

void reset_cpu(ulong ignoredval)
{
	printf("Resetting Oxsemi NAS...");

	/* Assert reset to cores as per power on defaults */
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_COPRO_BIT) |
	    (1UL << SYS_CTRL_RSTEN_USBHS_BIT) |
	    (1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT) |
	    (1UL << SYS_CTRL_RSTEN_MAC_BIT) |
	    (1UL << SYS_CTRL_RSTEN_PCI_BIT) |
	    (1UL << SYS_CTRL_RSTEN_DMA_BIT) |
	    (1UL << SYS_CTRL_RSTEN_DPE_BIT) |
	    (1UL << SYS_CTRL_RSTEN_SATA_BIT) |
	    (1UL << SYS_CTRL_RSTEN_SATA_PHY_BIT) |
	    (1UL << SYS_CTRL_RSTEN_STATIC_BIT) |
	    (1UL << SYS_CTRL_RSTEN_UART1_BIT) |
	    (1UL << SYS_CTRL_RSTEN_UART2_BIT) |
	    (1UL << SYS_CTRL_RSTEN_MISC_BIT) |
	    (1UL << SYS_CTRL_RSTEN_I2S_BIT) |
	    (1UL << SYS_CTRL_RSTEN_AHB_MON_BIT) |
	    (1UL << SYS_CTRL_RSTEN_UART3_BIT) |
	    (1UL << SYS_CTRL_RSTEN_UART4_BIT) |
	    (1UL << SYS_CTRL_RSTEN_SGDMA_BIT);

	/* Release reset to cores as per power on defaults */
	*(volatile u32 *)SYS_CTRL_RSTEN_CLR_CTRL =
	    (1UL << SYS_CTRL_RSTEN_GPIO_BIT);

	/* Disable clocks to cores as per power-on defaults */
	*(volatile u32 *)SYS_CTRL_CKEN_CLR_CTRL =
	    (1UL << SYS_CTRL_CKEN_COPRO_BIT) |
	    (1UL << SYS_CTRL_CKEN_DMA_BIT) |
	    (1UL << SYS_CTRL_CKEN_DPE_BIT) |
	    (1UL << SYS_CTRL_CKEN_SATA_BIT) |
	    (1UL << SYS_CTRL_CKEN_I2S_BIT) |
	    (1UL << SYS_CTRL_CKEN_USBHS_BIT) |
	    (1UL << SYS_CTRL_CKEN_MAC_BIT) | (1UL << SYS_CTRL_CKEN_STATIC_BIT);

	/* Enable clocks to cores as per power-on defaults */
	*(volatile u32 *)SYS_CTRL_CKEN_SET_CTRL =
	    (1UL << SYS_CTRL_CKEN_PCI_BIT);

	/* Set sys-control pin mux'ing as per power-on defaults */
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_0 = 0x800UL;
	*(volatile u32 *)SYS_CTRL_GPIO_PRIMSEL_CTRL_1 = 0x0UL;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_0 = 0x0UL;
	*(volatile u32 *)SYS_CTRL_GPIO_SECSEL_CTRL_1 = 0x0UL;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_0 = 0x0UL;
	*(volatile u32 *)SYS_CTRL_GPIO_TERTSEL_CTRL_1 = 0x0UL;

	/* No need to save any state, as the ROM loader can determine whether reset
	 * is due to power cycling or programatic action, just hit the (self-clearing)
	 * CPU reset bit of the block reset register
	 */
	*(volatile u32 *)SYS_CTRL_RSTEN_SET_CTRL =
	    (1UL << SYS_CTRL_RSTEN_ARM_BIT);
}
