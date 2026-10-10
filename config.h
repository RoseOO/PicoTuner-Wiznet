/* config.h - persistent (flash) user configuration for PicoTuner-WH */
#ifndef PTWH_CONFIG_H
#define PTWH_CONFIG_H

#include <stdint.h>

#define CONFIG_MAGIC        0x50545748u   /* "PTWH" */
#define CONFIG_VERSION      1
#define CONFIG_HOSTNAME_MAX 24

/* LNB supply states */
#define LNB_OFF      0
#define LNB_13V      1
#define LNB_18V      2
#define LNB_13V_22K  3
#define LNB_18V_22K  4

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint8_t  dhcp;                  /* 0 = static, 1 = DHCP */
    uint8_t  ip[4];
    uint8_t  sn[4];
    uint8_t  gw[4];
    uint8_t  dns[4];
    char     hostname[CONFIG_HOSTNAME_MAX];
    uint16_t baseipport;
    uint8_t  tsflash;
    uint8_t  debug_dhcp;
    uint8_t  lnb_x;                 /* LNB_* for receiver X */
    uint8_t  lnb_y;                 /* LNB_* for receiver Y */
    uint8_t  lnb_autostart;         /* apply lnb_x/lnb_y at boot */
    uint8_t  reserved[61];
    uint32_t crc;
} devconfig_t;

extern devconfig_t devconfig;

/* Fill devconfig with compiled defaults. */
void config_defaults(void);

/* Load from flash; falls back to defaults if absent/invalid. */
void config_load(void);

/* Commit devconfig to flash (safe against core 1).  Returns 0 on success. */
int  config_save(void);

#endif
