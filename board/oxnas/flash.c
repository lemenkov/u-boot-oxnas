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
#include <linux/byteorder/swab.h>

#ifndef CFG_NO_FLASH

#ifndef CFG_FLASH_BANKS_LIST
#define CFG_FLASH_BANKS_LIST { CFG_FLASH_BASE }
#endif

#define ADDR0           0x5555
#define ADDR1           0x2aaa
#define FLASH_WORD_SIZE unsigned short

static ulong bank_base[CFG_MAX_FLASH_BANKS] = CFG_FLASH_BANKS_LIST;

flash_info_t flash_info[CFG_MAX_FLASH_BANKS];

static void inline spin_wheel(void)
{
	static int p = 0;
	static char w[] = "\\/-";

	printf ("\010%c", w[p]);
	(++p == 3) ? (p = 0) : 0;
}

static ulong flash_get_size(unsigned long base, flash_info_t *info)
{
	short i;
	FLASH_WORD_SIZE value;
	volatile FLASH_WORD_SIZE *addr2 = (FLASH_WORD_SIZE *)base;

	/* Write auto select command: read Manufacturer ID */
	addr2[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
	addr2[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
	addr2[ADDR0] = (FLASH_WORD_SIZE)0x00900090;

	value = addr2[0];
printf("flash_get_size value: 0x%04x\n",value);
	switch (value) {
	case (FLASH_WORD_SIZE)AMD_MANUFACT:
		info->flash_id = FLASH_MAN_AMD;
		break;
	case (FLASH_WORD_SIZE)FUJ_MANUFACT:
		info->flash_id = FLASH_MAN_FUJ;
		break;
	case (FLASH_WORD_SIZE)INTEL_MANUFACT:
		info->flash_id = FLASH_MAN_INTEL;
		break;
	case (FLASH_WORD_SIZE)SST_MANUFACT:
		info->flash_id = FLASH_MAN_SST;
		break;
	case (FLASH_WORD_SIZE)ATM_MANUFACT:
		info->flash_id = FLASH_MAN_ATM; 
		break;
	default:
		info->flash_id = FLASH_UNKNOWN;
		info->sector_count = 0;
		info->size = 0;
		return (0);			/* no or unknown flash	*/
	}
printf(" -> info->flash_id = 0x%08x\n", info->flash_id);

	value = addr2[1];			/* device ID		*/
printf("Device value 0x%04x\n",value);
	switch (value) {
	case (FLASH_WORD_SIZE)AMD_ID_F040B:
		info->flash_id += FLASH_AM040;
		info->sector_count = 8;
		info->size = 0x0080000; /* => 512 ko */
		break;
	case (FLASH_WORD_SIZE)AMD_ID_LV400T:
		info->flash_id += FLASH_AM400T;
		info->sector_count = 11;
		info->size = 0x00080000;
		break;				/* => 0.5 MB		*/

	case (FLASH_WORD_SIZE)AMD_ID_LV400B:
		info->flash_id += FLASH_AM400B;
		info->sector_count = 11;
		info->size = 0x00080000;
		break;				/* => 0.5 MB		*/

	case (FLASH_WORD_SIZE)AMD_ID_LV800T:
		info->flash_id += FLASH_AM800T;
		info->sector_count = 19;
		info->size = 0x00100000;
		break;				/* => 1 MB		*/

	case (FLASH_WORD_SIZE)AMD_ID_LV800B:
		info->flash_id += FLASH_AM800B;
		info->sector_count = 19;
		info->size = 0x00100000;
		break;				/* => 1 MB		*/

	case (FLASH_WORD_SIZE)AMD_ID_LV160T:
		info->flash_id += FLASH_AM160T;
		info->sector_count = 35;
		info->size = 0x00200000;
		break;				/* => 2 MB		*/

	case (FLASH_WORD_SIZE)AMD_ID_LV160B:
		info->flash_id += FLASH_AM160B;
		info->sector_count = 35;
		info->size = 0x00200000;
		break;				/* => 2 MB		*/
	case (FLASH_WORD_SIZE)AMD_ID_LV320T:
		info->flash_id += FLASH_AM320T;
		info->sector_count = 67;
		info->size = 0x00400000;
		break;				/* => 4 MB		*/
	case (FLASH_WORD_SIZE)AMD_ID_LV640U:
		info->flash_id += FLASH_AM640U;
		info->sector_count = 128;
		info->size = 0x00800000;
		break;				/* => 8 MB		*/
#if 0	/* enable when device IDs are available */

	case (FLASH_WORD_SIZE)AMD_ID_LV320B:
		info->flash_id += FLASH_AM320B;
		info->sector_count = 67;
		info->size = 0x00400000;
		break;				/* => 4 MB		*/
#endif
	case (FLASH_WORD_SIZE)SST_ID_xF800A:
		info->flash_id += FLASH_SST800A;
		info->sector_count = 16;
		info->size = 0x00100000;
		break;				/* => 1 MB		*/
	case (FLASH_WORD_SIZE)INTEL_ID_28F320C3T:
		info->flash_id += FLASH_INTEL320T;
		info->sector_count = 71;
		info->size = 0x00400000;
		break;				/* => 4 MB		*/

	case (FLASH_WORD_SIZE)INTEL_ID_28F320B3T:
		info->flash_id += FLASH_INTEL320T;
		info->sector_count = 71;
		info->size = 0x00400000;
		break;				/* => 4 MB		*/


	case (FLASH_WORD_SIZE)SST_ID_xF160A:
		info->flash_id += FLASH_SST160A;
		info->sector_count = 32;
		info->size = 0x00200000;
		break;				/* => 2 MB		*/

	case (FLASH_WORD_SIZE)ATM_ID_BV322:
		info->flash_id += FLASH_AMDL322T;
		info->sector_count = 71;
		info->size = 0x00400000;
		break;				/* => 4 MB		*/

	default:
		info->flash_id = FLASH_UNKNOWN;
		return (0);			/* => no or unknown flash */

	}
printf(" -> info->flash_id = 0x%08x\n", info->flash_id);

	/* set up sector start address table */
	if (((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_SST) ||
	     (info->flash_id  == FLASH_AM040) ||
	     (info->flash_id  == FLASH_AM640U)){
		for (i = 0; i < info->sector_count; i++)
			info->start[i] = base + (i * 0x00010000);
	}
	else {
		if (info->flash_id & FLASH_BTYPE) {
			/* set sector offsets for bottom boot block type	*/
			info->start[0] = base + 0x00000000;
			info->start[1] = base + 0x00004000;
			info->start[2] = base + 0x00006000;
			info->start[3] = base + 0x00008000;
			for (i = 4; i < info->sector_count; i++)
				info->start[i] = base + (i * 0x00010000) - 0x00030000;
		}
		else {
			/* set sector offsets for top boot block type		*/
			i = info->sector_count - 1;
			if(info->sector_count==71) {

				info->start[i--] = base + info->size - 0x00002000;
				info->start[i--] = base + info->size - 0x00004000;
				info->start[i--] = base + info->size - 0x00006000;
				info->start[i--] = base + info->size - 0x00008000;
				info->start[i--] = base + info->size - 0x0000A000;
				info->start[i--] = base + info->size - 0x0000C000;
				info->start[i--] = base + info->size - 0x0000E000;
				for (; i >= 0; i--)
					info->start[i] = base + i * 0x000010000;
			}
			else {
				info->start[i--] = base + info->size - 0x00004000;
				info->start[i--] = base + info->size - 0x00006000;
				info->start[i--] = base + info->size - 0x00008000;
				for (; i >= 0; i--)
					info->start[i] = base + i * 0x00010000;
			}
		}
	}

	/* check for protected sectors */
	for (i = 0; i < info->sector_count; i++) {
		/* read sector protection at sector address, (A7 .. A0) = 0x02 */
		/* D0 = 1 if protected */
		addr2 = (volatile FLASH_WORD_SIZE *)(info->start[i]);
		if ((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_INTEL)
			info->protect[i] = 0;
		else
			info->protect[i] = addr2[2] & 1;
	}

	/*
	 * Prevent writes to uninitialized FLASH.
	 */
	if (info->flash_id != FLASH_UNKNOWN) {
		addr2 = (FLASH_WORD_SIZE *)info->start[0];
		*addr2 = (FLASH_WORD_SIZE)0x00F000F0;	/* reset bank */
	}
	return (info->size);
}

/** Copied from cfi_flash.c */
unsigned long flash_init(void)
{
    unsigned long size = 0;
    int i;

    /* Probe all flash banks */
    for (i = 0; i < CFG_MAX_FLASH_BANKS; ++i) {
        flash_info[i].flash_id = FLASH_UNKNOWN;
        size += flash_info[i].size = flash_get_size(bank_base[i], &flash_info[i]);
        if (flash_info[i].flash_id == FLASH_UNKNOWN) {
            printf("## Unknown FLASH on Bank %d - Size = 0x%08lx = %ld MB\n",
                   i, flash_info[i].size, flash_info[i].size << 20);
        }
    }

    /* Put into read array mode */
    volatile FLASH_WORD_SIZE *addr = (FLASH_WORD_SIZE *)(flash_info[0].start[0]);
    addr[0] = (FLASH_WORD_SIZE)0x00ff00ff;

    /* Monitor protection ON by default */
#if (CFG_MONITOR_BASE >= CFG_FLASH_BASE)
    flash_protect(FLAG_PROTECT_SET,
                  CFG_MONITOR_BASE,
                  CFG_MONITOR_BASE + CFG_MONITOR_LEN - 1,
                  &flash_info[0]);
#endif

    /* Environment protection ON by default */
#ifdef CFG_ENV_IS_IN_FLASH
    flash_protect(FLAG_PROTECT_SET,
                  CFG_ENV_ADDR,
/* May be supposed to use CFG_ENV_SECT_SIZE rather than CFG_ENV_SIZE here, but
 * I'm hoping it'll work OK as is and allow env. over multiple parameter sectors */
                  CFG_ENV_ADDR + CFG_ENV_SIZE - 1,
                  &flash_info[0]);
#endif

    /* Redundant environment protection ON by default */
#ifdef CFG_ENV_ADDR_REDUND
    flash_protect(FLAG_PROTECT_SET,
                  CFG_ENV_ADDR_REDUND,
                  CFG_ENV_ADDR_REDUND + CFG_ENV_SIZE_REDUND - 1,
                  &flash_info[0]);
#endif
    return (size);
}

static int wait_for_DQ7(flash_info_t *info, int sect)
{
	ulong start, now, last;
	volatile FLASH_WORD_SIZE *addr = (FLASH_WORD_SIZE *)(info->start[sect]);

	start = get_timer (0);
	last  = start;
	while ((addr[0] & (FLASH_WORD_SIZE)0x00800080) != (FLASH_WORD_SIZE)0x00800080) {
		if ((now = get_timer(start)) > CFG_FLASH_ERASE_TOUT) {
			printf ("Timeout\n");
			return ERR_TIMOUT;
		}
		/* show that we're waiting */
		if ((now - last) > 1000) {  /* every second */
			putc ('.');
			last = now;
		}
	}
	return ERR_OK;
}

int flash_erase(flash_info_t* info, int s_first, int s_last)
{
	volatile FLASH_WORD_SIZE *addr = (FLASH_WORD_SIZE *)(info->start[0]);
	volatile FLASH_WORD_SIZE *addr2;
	int flag, prot, sect, l_sect;
	int i, rcode = 0;


	if ((s_first < 0) || (s_first > s_last)) {
		if (info->flash_id == FLASH_UNKNOWN) {
			printf ("- missing\n");
		} else {
			printf ("- no sectors to erase\n");
		}
		return 1;
	}

	if (info->flash_id == FLASH_UNKNOWN) {
		printf ("Can't erase unknown flash type - aborted\n");
		return 1;
	}

	prot = 0;
	for (sect=s_first; sect<=s_last; ++sect) {
		if (info->protect[sect]) {
			prot++;
		}
	}

	if (prot) {
		printf ("- Warning: %d protected sectors will not be erased!\rcoden",
			prot);
	} else {
		printf ("\n");
	}

	l_sect = -1;

	/* Disable interrupts which might cause a timeout here */
	flag = disable_interrupts();

	/* Start erase on unprotected sectors */
	for (sect = s_first; sect<=s_last; sect++) {
		if (info->protect[sect] == 0) {	/* not protected */
			addr2 = (FLASH_WORD_SIZE *)(info->start[sect]);
			/*  printf("Erasing sector %p\n", addr2); */ /* CLH */
			if ((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_SST) {
				addr[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
				addr[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
				addr[ADDR0] = (FLASH_WORD_SIZE)0x00800080;
				addr[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
				addr[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
				addr2[0] = (FLASH_WORD_SIZE)0x00500050;  /* block erase */
				for (i=0; i<50; i++)
					udelay(1000);  /* wait 1 ms */
				rcode |= wait_for_DQ7(info, sect);
			}
			else {
                if ((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_INTEL) {
                    /* Clear the status register */
                    addr[0] = (FLASH_WORD_SIZE)0x00500050;

                    /* Erase setup, with adr lines indicating required sector */
                    addr2[0] = (FLASH_WORD_SIZE)0x00200020;
                    /* Erase confirm, with adr lines indicating required sector */
                    addr2[0] = (FLASH_WORD_SIZE)0x00D000D0;

                    /* Place in read status mode */
                    addr[0] = (FLASH_WORD_SIZE)0x00700070;

                    /* Read status until indicates that erasure has finished,
                       sampling every 10uS */
                    unsigned wait_limit = (CFG_FLASH_ERASE_TOUT / CFG_HZ) * 100000;
                    while (wait_limit-- && !(addr[0] & (1UL << 7))) {
                        spin_wheel();
                        udelay(10);
                    }
                    if (!wait_limit) {
                        printf("Timed out\n");
                    }

                    /* Read the erasure result */
                    FLASH_WORD_SIZE erase_status = addr[0];

                    /* Clear the status register */
                    addr[0] = (FLASH_WORD_SIZE)0x00500050;

                    /* Put back in read array mode */
                    addr[0] = (FLASH_WORD_SIZE)0x00ff00ff;

                    /* Accumulate any errors for all sectors */
                    rcode |= (erase_status & ((1UL << 5) | (1UL << 3)));
                    if (rcode) {
                        printf("Failed, so aborting\n");
                        break; // No point continuing if had a failure
                    }

                    /* Clear status */
                    addr[0] = (FLASH_WORD_SIZE)0x00500050;
				}
				else {
					addr[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
					addr[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
					addr[ADDR0] = (FLASH_WORD_SIZE)0x00800080;
					addr[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
					addr[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
					addr2[0] = (FLASH_WORD_SIZE)0x00300030;  /* sector erase */
					rcode |= wait_for_DQ7(info, sect);
				}
			}
			l_sect = sect;
			/*
			 * Wait for each sector to complete, it's more
			 * reliable.  According to AMD Spec, you must
			 * issue all erase commands within a specified
			 * timeout.  This has been seen to fail, especially
			 * if printf()s are included (for debug)!!
			 */
			/*   wait_for_DQ7(info, sect); */
		}
	}

	/* re-enable interrupts if necessary */
	if (flag)
		enable_interrupts();

	/* wait at least 80us - let's wait 1 ms */
	udelay (1000);

#if 0
	/*
	 * We wait for the last triggered sector
	 */
	if (l_sect < 0)
		goto DONE;
	wait_for_DQ7(info, l_sect);

DONE:
#endif
    /* Put back in read array mode */
	addr = (FLASH_WORD_SIZE *)info->start[0];
	addr[0] = (FLASH_WORD_SIZE)0x00ff00ff;

	if (!rcode)
	    printf (" done\n");

	return rcode;
}

/**
 * Write a word to Flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 */
//static FLASH_WORD_SIZE *read_val = (FLASH_WORD_SIZE *)0x200000;

static int write_word(flash_info_t *info, ulong dest, ulong data)
{
	volatile FLASH_WORD_SIZE *addr2 = (FLASH_WORD_SIZE *)(info->start[0]);
	volatile FLASH_WORD_SIZE *dest2 = (FLASH_WORD_SIZE *)dest;
	volatile FLASH_WORD_SIZE *data2 = (FLASH_WORD_SIZE *)&data;
	ulong start;
	int flag;
	int i;

	/* Disable interrupts which might cause a timeout here */
	flag = disable_interrupts();
//printf("Write 0x%08x to 0x%08x\n", data, dest);
    for (i=0; i < (4/sizeof(FLASH_WORD_SIZE)); i++) {
        if ((info->flash_id & FLASH_VENDMASK) == FLASH_MAN_INTEL) {
            FLASH_WORD_SIZE write_data = data2[i];
//            if ((write_data >> 8) && !(write_data & 0xff)) {
//                write_data |= 0x11;
//                printf("$BModifying zero word from 0x%04x to 0x%04x at 0x%08x\n", data2[i], write_data, &dest2[i]);
//            }

            unsigned attempts = CFG_FLASH_WRITE_ATTEMPTS;
            while (attempts--) {
                /* Clear the status register */
                addr2[0] = (FLASH_WORD_SIZE)0x00500050;

                /* Issue program setup command, this is a two stage command, with
                 * the second write latching the adr and data */
                addr2[0] = (FLASH_WORD_SIZE)0x00400040;
//printf("  -> 0x%04x to 0x%08x\n", write_data, &dest2[i]);
                dest2[i] = (FLASH_WORD_SIZE)write_data;

                /* Place in read status mode */
                addr2[0] = (FLASH_WORD_SIZE)0x00700070;

                /* Read status until indicates that writing has finished,
                   sampling every 10uS */
                unsigned wait_limit = (CFG_FLASH_WRITE_TOUT / CFG_HZ) * 100000;
                while (wait_limit-- && !(addr2[0] & (1UL << 7))) {
                    udelay(10);
                }
                if (!wait_limit) {
                    printf("Timed out\n");
                    return 1;
                }
//udelay(1000);
                /* Read the programming result */
                FLASH_WORD_SIZE program_status = addr2[0];

                /* Clear the status register */
                addr2[0] = (FLASH_WORD_SIZE)0x00500050;

                /* Check status for failure */
                if (program_status & (1UL << 1)) {
                    printf("$MProgramming attempted on a locked block\n");
                    break;
                }
                if (program_status & (1UL << 3)) {
                    printf("$MProgramming attempted with Vpp out of range\n");
                    break;
                }
                if (program_status & (1UL << 4)) {
                    // Try to program again
                    printf("$YProgramming failed, trying again...\n");
                    continue;
                }

                /* Put back in read array mode */
                addr2[0] = (FLASH_WORD_SIZE)0x00ff00ff;

                /* Check that the programmed location holds the correct value */
                if (dest2[i] != write_data) {
                    printf("$YError at %p, trying again\n", &dest2[i]);
                    continue;
                }

                // Programmed successfully, to don't try again
                break;
            }

            /* Put back in read array mode */
            addr2[0] = (FLASH_WORD_SIZE)0x00ff00ff;

            if (dest2[i] != write_data) {
                printf("$RError at %p 0x%04X should be 0x%04X\n", &dest2[i], dest2[i], write_data);
            }
		}
		else {
			addr2[ADDR0] = (FLASH_WORD_SIZE)0x00AA00AA;
			addr2[ADDR1] = (FLASH_WORD_SIZE)0x00550055;
			addr2[ADDR0] = (FLASH_WORD_SIZE)0x00A000A0;
			dest2[i] = data2[i];
			/* re-enable interrupts if necessary */
			if (flag)
				enable_interrupts();
			/* data polling for D7 */
			start = get_timer (0);
			while ((dest2[i] & (FLASH_WORD_SIZE)0x00800080) !=
				(data2[i] & (FLASH_WORD_SIZE)0x00800080)) {
				if (get_timer(start) > CFG_FLASH_WRITE_TOUT) {
					return (1);
				}
			}
		}
	}
	return (0);
}

/**
 * Copy memory to flash, returns:
 * 0 - OK
 * 1 - write timeout
 * 2 - Flash not erased
 */
int write_buff(flash_info_t* info, uchar* src, ulong addr, ulong cnt)
{
	ulong cp, wp, data;
	int i, l, rc;

//    printf("src = 0x%08x, addr = 0x%08x, cnt = %lu\n", src, addr, cnt);
//    for (i=0; i < cnt;) {
//        printf("0x%02x ", src[i++]);
//        if (!(i % 16)) {
//            printf("\n");
//        }
//    }
//    printf("\n");
    
	wp = (addr & ~3);	/* get lower word aligned address */
	/*
	 * handle unaligned start bytes
	 */
	if ((l = addr - wp) != 0) {
		data = 0;
		for (i=0, cp=wp; i<l; ++i, ++cp) {
			data = (data << 8) | (*(uchar *)cp);
		}
		for (; i<4 && cnt>0; ++i) {
			data = (data << 8) | *src++;
			--cnt;
			++cp;
		}
		for (; cnt==0 && i<4; ++i, ++cp) {
			data = (data << 8) | (*(uchar *)cp);
		}

		if ((rc = write_word(info, wp, __swab32(data))) != 0) {
			return (rc);
		}
		wp += 4;
	}

	/*
	 * handle word aligned part
	 */
	while (cnt >= 4) {
		data = 0;
		for (i=0; i<4; ++i) {
			data = (data << 8) | *src++;
		}
        if (!(cnt%1000)){
            spin_wheel();
        }
		if ((rc = write_word(info, wp, __swab32(data))) != 0) {
			return (rc);
		}
		wp  += 4;
		cnt -= 4;
	}

	if (cnt == 0) {
		return (0);
	}

	/*
	 * handle unaligned tail bytes
	 */
	data = 0;
	for (i=0, cp=wp; i<4 && cnt>0; ++i, ++cp) {
		data = (data << 8) | *src++;
		--cnt;
	}
	for (; i<4; ++i, ++cp) {
		data = (data << 8) | (*(uchar *)cp);
	}
	rc=write_word(info, wp, __swab32(data));
	return rc;
}

void flash_print_info(flash_info_t * info)
{
	int i;
	int k;
	int size;
	int erased;
	volatile unsigned long *flash;

	if (info->flash_id == FLASH_UNKNOWN) {
		printf ("missing or unknown FLASH type\n");
		return;
	}

	switch (info->flash_id & FLASH_VENDMASK) {
	case FLASH_MAN_AMD:	printf ("AMD ");		break;
	case FLASH_MAN_FUJ:	printf ("FUJITSU ");		break;
	case FLASH_MAN_SST:	printf ("SST ");		break;
	case FLASH_MAN_INTEL:	printf ("Intel ");		break;
	case FLASH_MAN_ATM:     printf ("Atmel ");              break;
	default:		printf ("Unknown Vendor ");	break;
	}

	switch (info->flash_id & FLASH_TYPEMASK) {
	case FLASH_AM040:	printf ("AM29F040 (512 Kbit, uniform sector size)\n");
				break;
	case FLASH_AM400B:	printf ("AM29LV400B (4 Mbit, bottom boot sect)\n");
				break;
	case FLASH_AM400T:	printf ("AM29LV400T (4 Mbit, top boot sector)\n");
				break;
	case FLASH_AM800B:	printf ("AM29LV800B (8 Mbit, bottom boot sect)\n");
				break;
	case FLASH_AM800T:	printf ("AM29LV800T (8 Mbit, top boot sector)\n");
				break;
	case FLASH_AM160B:	printf ("AM29LV160B (16 Mbit, bottom boot sect)\n");
				break;
	case FLASH_AM160T:	printf ("AM29LV160T (16 Mbit, top boot sector)\n");
				break;
	case FLASH_AM320B:	printf ("AM29LV320B (32 Mbit, bottom boot sect)\n");
				break;
	case FLASH_AM320T:	printf ("AM29LV320T (32 Mbit, top boot sector)\n");
				break;
	case FLASH_SST800A:	printf ("SST39LF/VF800 (8 Mbit, uniform sector size)\n");
				break;
	case FLASH_SST160A:	printf ("SST39LF/VF160 (16 Mbit, uniform sector size)\n");
				break;
    case FLASH_INTEL320T:	printf ("TE28F320{B/C}3T (32 Mbit, top sector size)\n");
				break;
	case FLASH_AM640U:	printf ("AM29LV640U (64 Mbit, uniform sector size)\n");
				break;
	case FLASH_AMDL322T:    printf("AT49BV322AT (64Mbit top boot sector)\n");
				break;
	default:		printf ("Unknown Chip Type\n");
				break;
	}

	printf ("  Size: %ld KB in %d Sectors\n",
		info->size >> 10, info->sector_count);

	printf ("  Sector Start Addresses:");
	for (i=0; i<info->sector_count; ++i) {
		/*
		 * Check if whole sector is erased
		*/
		if (i != (info->sector_count-1))
			size = info->start[i+1] - info->start[i];
		else
			size = info->start[0] + info->size - info->start[i];
		erased = 1;
		flash = (volatile unsigned long *)info->start[i];
		size = size >> 2;        /* divide by 4 for longword access */
		for (k=0; k<size; k++) {
			if (*flash++ != 0xffffffff) {
				erased = 0;
				break;
			}
		}
		if ((i % 5) == 0)
			printf ("\n   ");
		printf (" %08lX%s%s",
			info->start[i],
			erased ? " E" : "  ",
			info->protect[i] ? "RO " : "   ");
	}
	printf ("\n");
}
#endif // CFG_NO_FLASH

