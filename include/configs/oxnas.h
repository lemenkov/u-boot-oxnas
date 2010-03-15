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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef OXNAS_CONFIG_H
#define OXNAS_CONFIG_H

#define readb(p)  (*(volatile u8 *)(p))
#define readl(p)  (*(volatile u32 *)(p))
#define writeb(v, p) (*(volatile u8 *)(p)= (v))
#define writel(v, p) (*(volatile u32*)(p)=(v))

/**
 * Architecture
 */
#define CONFIG_ARM926EJS    1
#define CONFIG_OXNAS        1
//#define CONFIG_OXNAS_RESET_DELAY
//#define CONFIG_OXNAS_USE_PCI            /* Enables PCI clock and takes out of reset - needed if require access to static bus */
//#define CONFIG_OXNAS_FEEDBACK_PCI_CLKS  /* Feedback PCI clock out 3 to drive PCI core clock - needed if require access to static bus */
#define CONFIG_OXNAS_MANUAL_STATIC_ARBITRATION
#define CONFIG_OXNAS_USE_SATA	/* Define to include support for SATA disks */

#define CONFIG_SYS_NO_FLASH	/* Define to NOT include flash support on static bus */

/* Shall we overclock the CPU? */
#define PLL_400MHZ   0x00003014
#define PLL_425MHZ   0x00002b10
#define PLL_437_5MHZ 0x00003e18
#define PLL_466MHZ   0x00003010
#define PLL_500MHZ   0x00003410
#define PLL_533MHZ   0x00003810
//#define OXNAS_OVERCLOCK (PLL_533MHZ)

/* Won't be using any interrupts */
#undef CONFIG_USE_IRQ

/* Everything, incl board info, in Hz */
#undef CONFIG_SYS_CLKS_IN_HZ

#define CONFIG_SYS_HUSH_PARSER		1
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP	/* undef to save memory         */
#ifdef CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"$ "	/* Monitor Command Prompt */
#else
#define CONFIG_SYS_PROMPT		"# "	/* Monitor Command Prompt */
#endif
#define CONFIG_SYS_CBSIZE      256	/* Console I/O Buffer Size  */

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE      ((CONFIG_SYS_CBSIZE)+sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS     16	/* max number of command args   */
#define CONFIG_SYS_BARGSIZE    (CONFIG_SYS_CBSIZE)	/* Boot Argument Buffer Size    */

#define CONFIG_CMDLINE_TAG          1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS    1
#define CONFIG_MISC_INIT_R          1	/* call misc_init_r during start up */
#define CONFIG_INITRD_TAG           1	/* allow initrd tag to be generated */

/* May want to do some setup prior to relocation */
#define CONFIG_SKIP_LOWLEVEL_INIT

/* ARM specific late initialisation */
#define BOARD_LATE_INIT

/**
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE        (128*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ    (4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ    (4*1024)	/* FIQ stack */
#endif

/**
 * RAM
 */
#define CONFIG_NR_DRAM_BANKS    1	/* We have 1 bank of SDRAM */
#define PHYS_SDRAM_1_PA         0x48000000	/* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE       0x02000000	/* 32 MB */
#define SRAM_BASE_PA            0x4C000000
#define INITIALISE_SDRAM

/* Default location from which bootm etc will load */
#define CONFIG_SYS_LOAD_ADDR   (PHYS_SDRAM_1_PA)

#define CONFIG_SYS_MEMTEST_START   (PHYS_SDRAM_1_PA)
#define CONFIG_SYS_MEMTEST_END     ((PHYS_SDRAM_1_PA) + (PHYS_SDRAM_1_SIZE))

/**
 * Core addresses
 */
#define MAC_BASE_PA             0x40400000
#define STATIC_CS0_BASE_PA      0x41000000
#define STATIC_CS1_BASE_PA      0x41400000
#define STATIC_CS2_BASE_PA      0x41800000
#define STATIC_CONTROL_BASE_PA  0x41C00000
#define SATA_DATA_BASE_PA       0x42000000

#define APB_BRIDGE_A_BASE_PA    0x44000000
#define APB_BRIDGE_B_BASE_PA    0x45000000

#define GPIO_1_PA               ((APB_BRIDGE_A_BASE_PA) + 0x0)
#define GPIO_2_PA               ((APB_BRIDGE_A_BASE_PA) + 0x100000)

#define SYS_CONTROL_BASE_PA     ((APB_BRIDGE_B_BASE_PA) + 0x0)
#define DMA_BASE_PA             ((APB_BRIDGE_B_BASE_PA) + 0x600000)
#define RPS_BASE                ((APB_BRIDGE_B_BASE_PA) + 0x300000)

/* Static bus registers */
#define STATIC_CONTROL_VERSION  ((STATIC_CONTROL_BASE_PA) + 0x0)
#define STATIC_CONTROL_BANK0    ((STATIC_CONTROL_BASE_PA) + 0x4)
#define STATIC_CONTROL_BANK1    ((STATIC_CONTROL_BASE_PA) + 0x8)
#define STATIC_CONTROL_BANK2    ((STATIC_CONTROL_BASE_PA) + 0xC)

/* Define the master clock frequencies of the NAS chip */
#define NOMINAL_PLL400_FREQ 400000000
#define NOMINAL_RPSCLK_FREQ 25000000

/* Clock to the ARM/DDR */
#define NOMINAL_ARMCLK  ((NOMINAL_PLL400_FREQ) / 2)
#define NOMINAL_SYSCLK  ((NOMINAL_PLL400_FREQ) / 4)
#define NOMINAL_PCICLK  ((NOMINAL_PLL400_FREQ) / 12)

/**
 * Timer
 */
#define CONFIG_SYS_TIMERBASE            ((RPS_BASE) + 0x200)
#define TIMER_PRESCALE_BIT       2
#define TIMER_PRESCALE_1_ENUM    0
#define TIMER_PRESCALE_16_ENUM   1
#define TIMER_PRESCALE_256_ENUM  2
#define TIMER_MODE_BIT           6
#define TIMER_MODE_FREE_RUNNING  0
#define TIMER_MODE_PERIODIC      1
#define TIMER_ENABLE_BIT         7
#define TIMER_ENABLE_DISABLE     0
#define TIMER_ENABLE_ENABLE      1

#define TIMER_PRESCALE_ENUM      (TIMER_PRESCALE_256_ENUM)
#define CONFIG_SYS_HZ                   ((NOMINAL_RPSCLK_FREQ) / 256)

/**
 * GPIO
 */
#define GPIO_1_OE       ((GPIO_1_PA) + 0x4)
#define GPIO_1_SET_OE   ((GPIO_1_PA) + 0x1C)
#define GPIO_1_CLR_OE   ((GPIO_1_PA) + 0x20)

#define GPIO_2_OE       ((GPIO_2_PA) + 0x4)
#define GPIO_2_SET_OE   ((GPIO_2_PA) + 0x1C)
#define GPIO_2_CLR_OE   ((GPIO_2_PA) + 0x20)

/**
 * Serial Configuration
 */
//#define EXTERNAL_UART
#define EXT_UART_BASE       0x28000000

#define UART_1_BASE (APB_BRIDGE_A_BASE_PA + 0x200000)
#define UART_2_BASE (APB_BRIDGE_A_BASE_PA + 0x300000)
#define UART_3_BASE (APB_BRIDGE_A_BASE_PA + 0x900000)
#define UART_4_BASE (APB_BRIDGE_A_BASE_PA + 0xA00000)

#define CONFIG_SYS_NS16550          1
#define CONFIG_SYS_NS16550_SERIAL   1
#define CONFIG_SYS_NS16550_REG_SIZE 1

#ifdef EXTERNAL_UART
#define CONFIG_SYS_NS16550_CLK      16000000
#define CONFIG_SYS_NS16550_COM1     (EXT_UART_BASE)
#else // EXTERNAL_UART
#define CONFIG_SYS_NS16550_CLK      (NOMINAL_SYSCLK)
#define USE_UART_FRACTIONAL_DIVIDER
//#define CONFIG_OXNAS_UART1
#define CONFIG_OXNAS_UART2
//#define CONFIG_OXNAS_UART3
//#define CONFIG_OXNAS_UART4
#define CONFIG_SYS_NS16550_COM1     (UART_2_BASE)
//#define CONFIG_SYS_NS16550_COM2     (UART_2_BASE)
//#define CONFIG_SYS_NS16550_COM3     (UART_3_BASE)
//#define CONFIG_SYS_NS16550_COM4     (UART_4_BASE)
#endif // EXTERNAL_UART

#define CONFIG_CONS_INDEX    1
#define CONFIG_BAUDRATE      115200
#define CONFIG_SYS_BAUDRATE_TABLE   { 9600, 19200, 38400, 57600, 115200 }

/**
 * Monitor commands
 */

#define CONFIG_CMD_BOOTD	/* bootd                        */
#define CONFIG_CMD_ECHO		/* echo arguments               */
#define CONFIG_CMD_EDITENV	/* editenv                      */
#define CONFIG_CMD_IMI		/* iminfo                       */
#define CONFIG_CMD_ITEST	/* Integer (and string) test    */
#define CONFIG_CMD_LOADB	/* loadb                        */
#define CONFIG_CMD_LOADS	/* loads                        */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc */
#define CONFIG_CMD_NET		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_DHCP		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_PING		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_RUN		/* run command in env variable  */
#define CONFIG_CMD_SAVEENV	/* saveenv                      */
#define CONFIG_CMD_SOURCE	/* "source" command support     */

#define CONFIG_CMD_ENV
#define CONFIG_CMD_IDE
#define CONFIG_CMD_EXT2

/**
 * Booting
 */
#define CONFIG_BOOTDELAY            2
#define CONFIG_BOOTARGS             "mem=32M console=ttyS0,115200 root=/dev/sda1 netdev=0,0,0x0030e000,0x0001,eth0 elevator=cfq"
#define CONFIG_BOOTCOMMAND          "ext2load ide 0:1 0x48500000 /boot/uImage ; bootm 0x48500000"

/**
 * Networking
 */
#define CONFIG_GMAC 1

#define CONFIG_ETHADDR      00:30:e0:00:00:01
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_IPADDR		192.168.0.5
#define CONFIG_SERVERIP		192.168.0.2
#define CFG_AUTOLOAD        "n"
#define CONFIG_NET_RETRY_COUNT 30

/**
 * Environment organization
 */

/* Environment is read-only */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_ADDR        (SRAM_BASE_PA + 0x5c00)
#define CONFIG_ENV_SIZE        (8*1024)
#define CONFIG_ENV_DISK_SECTOR 47

#define CONFIG_ENV_OVERWRITE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN      (CONFIG_ENV_SIZE + 128*1024)
#define CONFIG_SYS_GBL_DATA_SIZE   128	/* size in bytes reserved for initial data */

/**
 * ASM startup control
 */
/* Start of address within SRAM of loader's exception table. */
/* ROM-based exception table will redirect to here */
#define EXCEPTION_BASE  (SRAM_BASE_PA)

/**
 * Disk related stuff
 */
#define CONFIG_LBA48
#define CONFIG_DOS_PARTITION
#define CONFIG_SYS_IDE_MAXDEVICE   2
#define CONFIG_SYS_IDE_MAXBUS      1
#define CONFIG_IDE_PREINIT
#undef CONFIG_IDE_RESET
#undef CONFIG_IDE_LED
#define CONFIG_SYS_ATA_DATA_OFFSET 0
#define CONFIG_SYS_ATA_REG_OFFSET  0
#define CONFIG_SYS_ATA_ALT_OFFSET  0

/**
 * SATA related definitions
 */
#define ATA_PORT_CTL        0
#define ATA_PORT_FEATURE    1
#define ATA_PORT_NSECT      2
#define ATA_PORT_LBAL       3
#define ATA_PORT_LBAM       4
#define ATA_PORT_LBAH       5
#define ATA_PORT_DEVICE     6
#define ATA_PORT_COMMAND    7

#define SATA_0_REGS_BASE (APB_BRIDGE_B_BASE_PA + 0x900000)
#define SATA_0_LINK_BASE (APB_BRIDGE_B_BASE_PA + 0x940000)
#define SATA_1_REGS_BASE (APB_BRIDGE_B_BASE_PA + 0x980000)
#define SATA_1_LINK_BASE (APB_BRIDGE_B_BASE_PA + 0x9c0000)

/* The offsets to the SATA registers */
#define SATA_ORB1_OFF           0
#define SATA_ORB2_OFF           1
#define SATA_ORB3_OFF           2
#define SATA_ORB4_OFF           3
#define SATA_ORB5_OFF           4

#define SATA_FIS_ACCESS         11
#define SATA_INT_STATUS_OFF     12	/* Read only */
#define SATA_INT_CLR_OFF        12	/* Write only */
#define SATA_INT_ENABLE_OFF     13	/* Read only */
#define SATA_INT_ENABLE_SET_OFF 13	/* Write only */
#define SATA_INT_ENABLE_CLR_OFF 14	/* Write only */
#define SATA_VERSION_OFF        15
#define SATA_BURST_BUF_CTRL_OFF 18
#define SATA_CONTROL_OFF        23
#define SATA_COMMAND_OFF        24
#define SATA_DEVICE_SELECT_OFF  25
#define SATA_DEVICE_CONTROL_OFF 26
#define SATA_DRIVE_CONTROL_OFF  27

/* The offsets to the link registers that are access in an asynchronous manner */
#define OX800SATA_LINK_DATA     (0x00000000)
#define OX800SATA_LINK_RD_ADDR  (0x00000001)
#define OX800SATA_LINK_WR_ADDR  (0x00000002)
#define OX800SATA_LINK_CONTROL  (0x00000003)

/* SATA interrupt status register fields */
#define SATA_INT_STATUS_EOC_RAW_BIT     8
#define SATA_INT_STATUS_EODC_RAW_BIT    9
#define SATA_INT_STATUS_ERROR_BIT       10
#define SATA_INT_STATUS_EOADT_RAW_BIT   13

/* ATA status (7) register field definitions */
#define ATA_STATUS_BSY_BIT     7
#define ATA_STATUS_DRDY_BIT    6
#define ATA_STATUS_DF_BIT      5
#define ATA_STATUS_DRQ_BIT     3
#define ATA_STATUS_ERR_BIT     0

/* ATA device (6) register field definitions */
#define ATA_DEVICE_FIXED_MASK 0xA0
#define ATA_DEVICE_DRV_BIT 4
#define ATA_DEVICE_DRV_NUM_BITS 1
#define ATA_DEVICE_LBA_BIT 6

/* ATA control (0) register field definitions */
#define ATA_CTL_SRST_BIT   2

/* ATA Command register initiated commands */
#define ATA_CMD_INIT    0x91
#define ATA_CMD_IDENT   0xEC

/* SATA core command register commands */
#define SATA_CMD_WRITE_TO_ORB_REGS_NO_COMMAND   1
#define SATA_CMD_WRITE_TO_ORB_REGS              2
#define SATA_CMD_READ_ALL_REGS                  3
#define SATA_CMD_READ_STATUS_REG                4

#define SATA_CMD_BUSY_BIT 7

#define SATA_OPCODE_MASK 0x3

#define SATA_LBAL_BIT    0
#define SATA_LBAM_BIT    8
#define SATA_LBAH_BIT    16
#define SATA_DEVICE_BIT  24
#define SATA_NSECT_BIT   0
#define SATA_FEATURE_BIT 16
#define SATA_COMMAND_BIT 24
#define SATA_CTL_BIT     24

#define SATA_STD_ASYNC_REGS_OFF 0x20
#define SATA_SCR_STATUS      0
#define SATA_SCR_ERROR       1
#define SATA_SCR_CONTROL     2
#define SATA_SCR_ACTIVE      3
#define SATA_SCR_NOTIFICAION 4

#define SATA_BURST_BUF_FORCE_EOT_BIT        0
#define SATA_BURST_BUF_DATA_INJ_ENABLE_BIT  1
#define SATA_BURST_BUF_DIR_BIT              2
#define SATA_BURST_BUF_DATA_INJ_END_BIT     3
#define SATA_BURST_BUF_FIFO_DIS_BIT         4
#define SATA_BURST_BUF_DIS_DREQ_BIT         5
#define SATA_BURST_BUF_DREQ_BIT             6

#endif // OXNAS_CONFIG_H
