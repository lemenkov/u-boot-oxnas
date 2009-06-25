/*
 * (C) Copyright 2006
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
#include <common.h>

#if defined(CFG_ENV_IS_IN_DISK)

#include <command.h>
#include <environment.h>
#include <ide.h>

extern int is_device_present(int device_number);
extern int ide_preinit(void);

/* Point to the environment as held in SRAM */
env_t *env_ptr = NULL;

char *env_name_spec = "Disk";

/* The default environment compiled into U-Boot */
extern uchar default_environment[];

uchar env_get_char_spec(int index)
{
    DECLARE_GLOBAL_DATA_PTR;

    return *((uchar *)(gd->env_addr + index));
}

#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

void env_relocate_spec(void)
{
    /* Compute the CRC of the environment in SRAM, copied from disk at boot */
    env_t *sram_env = (env_t*)CFG_ENV_ADDR;
    ulong  crc = crc32(0, sram_env->data, CFG_ENV_SIZE - offsetof(env_t, data));

    /* Copy the SRAM environment and CRC to the working environment */
    memcpy(env_ptr->data, sram_env->data, CFG_ENV_SIZE - offsetof(env_t, data));
    env_ptr->crc = crc;
}

int saveenv(void)
{
    /* Compute the CRC of the working environment */
    env_ptr->crc = crc32(0, env_ptr->data, CFG_ENV_SIZE - offsetof(env_t, data));

    /* Copy the working environment to the reserved area on each disk device */
    int status = 1;
    int i;
    for (i=0; i < CFG_IDE_MAXDEVICE; ++i) {
        if (!is_device_present(i)) {
            continue;
        }
        ide_preinit();
        unsigned long written = ide_write(i, CFG_ENV_DISK_SECTOR, CFG_ENV_SIZE/512, (ulong*)env_ptr);
        if (written != CFG_ENV_SIZE/512) {
            status = 0;
        }
    }

    return status;
}

int env_init(void)
{
    DECLARE_GLOBAL_DATA_PTR;

    /* Check the CRC on the environment in SRAM, as loaded from disk by an
     * earlier loader in the boot process */
    env_t *sram_env = (env_t*)CFG_ENV_ADDR;
    ulong  crc = crc32(0, sram_env->data, CFG_ENV_SIZE - offsetof(env_t, data));

    if (crc == sram_env->crc) {
        gd->env_addr  = (ulong)sram_env->data;
        gd->env_valid = 1;
    } else {
        gd->env_addr  = (ulong)default_environment;
        gd->env_valid = 0;
    }

    return 0;
}
#endif // CFG_ENV_IS_IN_DISK
