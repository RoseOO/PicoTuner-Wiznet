/* webui.c - minimal HTTP configuration/control server + JSON API
 *
 * Runs on core 0, serviced cooperatively from the main loop (webui_poll).
 * Uses WIZnet hardware socket WEBUI_SOCK in TCP mode; the application-level
 * state is provided by the webui_* callbacks implemented in picotunewh.c.
 */
#include "webui.h"
#include "webpage.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"

#include "wizchip_conf.h"
#include "socket.h"

#define WEBUI_SOCK   4
#define WEBUI_PORT   80
#define WEBUI_REQMAX 640

#if defined(_WIZCHIP_SOCK_NUM_) && (_WIZCHIP_SOCK_NUM_ > 4)

static int     webui_running = 0;
static uint8_t webui_req[WEBUI_REQMAX];
static int     webui_reqlen = 0;

static int webui_send_all(const uint8_t *data, int len)
{
    int sent = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (sent < len)
    {
        uint8_t sr = getSn_SR(WEBUI_SOCK);
        if (sr != SOCK_ESTABLISHED && sr != SOCK_CLOSE_WAIT)
        {
            return -1;
        }

        uint16_t fsr = getSn_TX_FSR(WEBUI_SOCK);
        if (fsr == 0)
        {
            sleep_ms(1);
        }
        else
        {
            uint16_t chunk = (uint16_t)((len - sent) < (int)fsr ? (len - sent) : (int)fsr);
            int32_t  s = send(WEBUI_SOCK, (uint8_t *)data + sent, chunk);
            if (s > 0)
            {
                sent += s;
            }
            else if (s == SOCK_BUSY)
            {
                sleep_ms(1);
            }
            else
            {
                return -1;
            }
        }

        if (to_ms_since_boot(get_absolute_time()) - start > 1500)
        {
            return -1;
        }
    }
    return sent;
}

static void webui_respond(int code, const char *ctype, const char *body, int blen)
{
    char hdr[176];
    int  n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n"
        "Access-Control-Allow-Origin: *\r\n\r\n",
        code, (code == 200) ? "OK" : "Error", ctype, blen);

    webui_send_all((const uint8_t *)hdr, n);
    if (blen > 0)
    {
        webui_send_all((const uint8_t *)body, blen);
    }
}

static void webui_json_ok(int ok, const char *msg)
{
    char body[224];
    int  n = snprintf(body, sizeof(body), "{\"ok\":%s,\"msg\":\"%s\"}",
                      ok ? "true" : "false", msg ? msg : "");
    webui_respond(ok ? 200 : 400, "application/json", body, n);
}

/* find key=value in a form-encoded / query string */
static int webui_param(const char *body, const char *key, char *out, int outlen)
{
    const char *p = body;
    int klen = (int)strlen(key);

    while ((p = strstr(p, key)) != 0)
    {
        if ((p == body || p[-1] == '&' || p[-1] == '?') && p[klen] == '=')
        {
            int i = 0;
            p += klen + 1;
            while (*p && *p != '&' && i < outlen - 1)
            {
                out[i++] = (*p == '+') ? ' ' : *p;
                p++;
            }
            out[i] = 0;
            return 1;
        }
        p += klen;
    }
    return 0;
}

static void webui_handle(char *req)
{
    char   method[8] = {0};
    char   target[96] = {0};
    char   params[512] = {0};
    char   val[64];
    char   msg[160];
    char  *path;
    char  *query;
    char  *body;
    char  *p;
    int    plen = 0;

    if (sscanf(req, "%7s %95s", method, target) != 2)
    {
        return;
    }

    query = strchr(target, '?');
    if (query)
    {
        *query++ = 0;
    }
    path = target;

    p = strstr(req, "\r\n\r\n");
    body = p ? p + 4 : 0;

    if (query && *query)
    {
        plen += snprintf(params + plen, sizeof(params) - plen, "%s", query);
    }
    if (body && *body && plen < (int)sizeof(params) - 2)
    {
        if (plen) params[plen++] = '&';
        snprintf(params + plen, sizeof(params) - plen, "%s", body);
    }

    if (strcmp(method, "GET") == 0 && (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0))
    {
        webui_respond(200, "text/html", WEBPAGE_HTML, (int)sizeof(WEBPAGE_HTML) - 1);
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/status") == 0)
    {
        static char json[1400];
        webui_status_json(json, sizeof(json));
        webui_respond(200, "application/json", json, (int)strlen(json));
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/config") == 0)
    {
        static char json[640];
        webui_config_json(json, sizeof(json));
        webui_respond(200, "application/json", json, (int)strlen(json));
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/config") == 0)
    {
        int ok = webui_apply_config(params, (int)strlen(params), msg, sizeof(msg));
        webui_json_ok(ok == 0, msg);
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/lnb") == 0)
    {
        int rx = 0, state = 0;
        if (webui_param(params, "rx", val, sizeof(val)))    rx = atoi(val);
        if (webui_param(params, "state", val, sizeof(val))) state = atoi(val);
        int ok = webui_set_lnb(rx, state, msg, sizeof(msg));
        webui_json_ok(ok == 0, msg);
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/tune") == 0)
    {
        int  rx = 0, freq = 0, sr = 0, lo = 9750000;
        char fplug = 'A';
        if (webui_param(params, "rx", val, sizeof(val)))    rx = atoi(val);
        if (webui_param(params, "freq", val, sizeof(val)))  freq = atoi(val);
        if (webui_param(params, "sr", val, sizeof(val)))    sr = atoi(val);
        if (webui_param(params, "lo", val, sizeof(val)))    lo = atoi(val);
        if (webui_param(params, "fplug", val, sizeof(val)) && val[0]) fplug = val[0];
        int ok = webui_tune(rx, freq, sr, lo, fplug, msg, sizeof(msg));
        webui_json_ok(ok == 0, msg);
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/reboot") == 0)
    {
        int mode = 0;
        if (webui_param(params, "mode", val, sizeof(val))) mode = atoi(val);
        int ok = webui_reboot(mode, msg, sizeof(msg));
        webui_json_ok(ok == 0, msg);
        return;
    }

    webui_respond(404, "text/plain", "not found", 9);
}

void webui_init(void)
{
    if (socket(WEBUI_SOCK, Sn_MR_TCP, WEBUI_PORT, SF_IO_NONBLOCK) == WEBUI_SOCK)
    {
#if (_WIZCHIP_ == W6100)
        /* W6100 ioLibrary does not apply the non-blocking flag in socket() */
        {
            uint8_t iomode = SOCK_IO_NONBLOCK;
            ctlsocket(WEBUI_SOCK, CS_SET_IOMODE, &iomode);
        }
#endif
        listen(WEBUI_SOCK);
        webui_running = 1;
    }
}

void webui_poll(void)
{
    if (!webui_running) return;

    switch (getSn_SR(WEBUI_SOCK))
    {
        case SOCK_ESTABLISHED:
        {
            int32_t r = recv(WEBUI_SOCK, webui_req + webui_reqlen,
                             (uint16_t)(WEBUI_REQMAX - 1 - webui_reqlen));
            if (r > 0)
            {
                webui_reqlen += r;
                webui_req[webui_reqlen] = 0;
                if (strstr((char *)webui_req, "\r\n\r\n") || webui_reqlen >= WEBUI_REQMAX - 1)
                {
                    webui_handle((char *)webui_req);
                    webui_reqlen = 0;
                    disconnect(WEBUI_SOCK);
                }
            }
            else if (r < 0)
            {
                webui_reqlen = 0;
                disconnect(WEBUI_SOCK);
            }
            break;
        }

        case SOCK_CLOSE_WAIT:
            webui_reqlen = 0;
            disconnect(WEBUI_SOCK);
            break;

        case SOCK_CLOSED:
        case SOCK_INIT:
            webui_reqlen = 0;
            if (socket(WEBUI_SOCK, Sn_MR_TCP, WEBUI_PORT, SF_IO_NONBLOCK) == WEBUI_SOCK)
            {
                listen(WEBUI_SOCK);
            }
            break;

        default:
            break;
    }
}

#else   /* chip with <= 4 sockets (e.g. W5100S) - no room for a web server */

void webui_init(void) {}
void webui_poll(void) {}

#endif
