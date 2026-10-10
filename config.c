/* config.c - persistent (flash) user configuration for PicoTuner-WH
 *
 * The configuration lives in the last 4 KB sector of a 2 MB flash map, which
 * is valid on both the 2 MB W5500-EVB-Pico2 and a 4 MB Pico 2, and is well
 * clear of the program image.  Writes go through flash_safe_execute() so the
 * other core is locked out (multicore_lockout_victim_init() is called on
 * core 1) and the XIP cache is handled by the SDK.
 */
#include "config.h"

#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/flash.h"

devconfig_t devconfig;

#define CONFIG_FLASH_BASE   0x10000000u                        /* XIP base */
#define CONFIG_FLASH_OFFSET (0x200000u - FLASH_SECTOR_SIZE)    /* last 4 KB of 2 MB */
#define CONFIG_FLASH_ADDR   (CONFIG_FLASH_BASE + CONFIG_FLASH_OFFSET)

static uint32_t config_crc(const devconfig_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint32_t crc = 0xFFFFFFFFu;
    size_t n = offsetof(devconfig_t, crc);

    for (size_t i = 0; i < n; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc;
}

void config_defaults(void)
{
    memset(&devconfig, 0, sizeof(devconfig));

    devconfig.magic   = CONFIG_MAGIC;
    devconfig.version = CONFIG_VERSION;
    devconfig.dhcp    = 1;

    devconfig.ip[0] = 192; devconfig.ip[1] = 168; devconfig.ip[2] = 77; devconfig.ip[3] = 203;
    devconfig.sn[0] = 255; devconfig.sn[1] = 255; devconfig.sn[2] = 255; devconfig.sn[3] = 0;
    devconfig.gw[0] = 192; devconfig.gw[1] = 168; devconfig.gw[2] = 77; devconfig.gw[3] = 1;
    devconfig.dns[0] = 192; devconfig.dns[1] = 168; devconfig.dns[2] = 77; devconfig.dns[3] = 1;

    strcpy(devconfig.hostname, "PicoTunerWH");

    devconfig.baseipport     = 9900;
    devconfig.tsflash        = 1;
    devconfig.debug_dhcp     = 0;
    devconfig.lnb_x          = LNB_13V;
    devconfig.lnb_y          = LNB_13V;
    devconfig.lnb_autostart  = 0;

    devconfig.crc = config_crc(&devconfig);
}

void config_load(void)
{
    const devconfig_t *fc = (const devconfig_t *)CONFIG_FLASH_ADDR;

    if (fc->magic == CONFIG_MAGIC &&
        fc->version == CONFIG_VERSION &&
        fc->crc == config_crc(fc))
    {
        memcpy(&devconfig, fc, sizeof(devconfig));
    }
    else
    {
        config_defaults();
    }
}

static uint8_t config_page[FLASH_SECTOR_SIZE];

static void config_flash_write(void *param)
{
    (void) param;
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, config_page, FLASH_SECTOR_SIZE);
}

int config_save(void)
{
    devconfig.magic   = CONFIG_MAGIC;
    devconfig.version = CONFIG_VERSION;
    devconfig.crc     = config_crc(&devconfig);

    memset(config_page, 0xFF, sizeof(config_page));
    memcpy(config_page, &devconfig, sizeof(devconfig));

    return flash_safe_execute(config_flash_write, NULL, 2000);
}
