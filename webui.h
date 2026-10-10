/* webui.h - minimal HTTP configuration/control server + JSON API */
#ifndef PTWH_WEBUI_H
#define PTWH_WEBUI_H

/* Start the HTTP listener.  Call once, after the WIZnet sockets exist. */
void webui_init(void);

/* Non-blocking.  Call every main-loop iteration (core 0 only). */
void webui_poll(void);

/* -------------------------------------------------------------------------
 * Application callbacks.  These are implemented in picotunewh.c, which owns
 * the network/tuner/LNB state; webui.c only handles HTTP and JSON framing.
 * ------------------------------------------------------------------------- */

/* Write a JSON object describing network + tuner + LNB status. */
void webui_status_json(char *out, int outlen);

/* Write a JSON object describing the persisted configuration. */
void webui_config_json(char *out, int outlen);

/* Apply a form-encoded configuration body.  Returns 0 on success; on success
 * a human readable result is written to msg. */
int webui_apply_config(const char *body, int len, char *msg, int msglen);

/* Set LNB supply for receiver rx (1 or 2) to LNB_* state. */
int webui_set_lnb(int rx, int state, char *msg, int msglen);

/* Queue a tune: frequency and LO in kHz, symbol rate in kS, fplug 'A'/'B'. */
int webui_tune(int rx, int freq, int sr, int lo, char fplug, char *msg, int msglen);

/* mode: 0 = reboot, 1 = reset settings + reboot, 2 = BOOTSEL. */
int webui_reboot(int mode, char *msg, int msglen);

/* Raw NIM register access. dev: 0 = STV0910, 1 = STV6120, 2 = STVVGLNA.
 * addr selects the STVVGLNA I2C address. Return 0 on success. */
int webui_reg_read (int dev, int addr, int reg, int *val);
int webui_reg_write(int dev, int addr, int reg, int val);

/* Named tuner / LNA chip controls (form-encoded body). Return 0 on success. */
int webui_nim_control(const char *body, int len, char *msg, int msglen);

#endif
