# m1rxo_v1_2 HTTP / JSON API

The firmware exposes a small HTTP/1.1 server on **TCP port 80** (W5500/W6100
builds only). All endpoints return `application/json`. There is no
authentication. Requests are form-encoded (`application/x-www-form-urlencoded`)
or query strings.

Base URL: `http://<device-ip>/`

---

## `GET /` — web UI

Returns the embedded configuration page (`text/html`).

---

## `GET /api/status` — live status

Returns the current network, LNB and per-receiver tuner state.

```json
{
  "version": "m1rxo_v1_2-w5500",
  "uptime": 123,
  "mac": "28:CD:C1:99:11:D3",
  "ip": "192.168.150.146",
  "sn": "255.255.255.0",
  "gw": "192.168.150.1",
  "dns": "192.168.150.1",
  "dhcp": 1,
  "dhcp_status": "Success",
  "baseipport": 9900,
  "link": 1,
  "lnb_x": { "present": 1, "state": "HI", "value": 2 },
  "lnb_y": { "present": 0, "state": "OFF", "value": 0 },
  "rx": [
    {
      "id": 1, "nim": "FTS4334L", "active": 1, "state": "DVB-S2", "lock": 1,
      "freq": 10491.500, "lo": 9750.000, "hardwarefreq": 741.500,
      "sr": 1500, "sr_full": 1498304,
      "mer": "12.3", "mer_raw": 123, "modcod": "8PSK 3/4", "rolloff": 0,
      "frame": 0, "pilots": 1, "lna_gain": 12,
      "power_i": 0, "power_q": 0, "const_i": 0, "const_q": 0,
      "ber": 0, "viterbi": 0, "ldpc_err": 0, "bch_err": 0, "bch_uncorr": 0,
      "puncture": 0, "service": "", "provider": "", "null_pct": 0,
      "packets": 12345, "vlcstops": 1, "modechanges": 1, "ipchanges": 0,
      "input": "X", "ts": "192.168.150.10:9941"
    }
  ]
}
```

Field notes:

- `dhcp`: `1` = DHCP, `0` = static.
- `lnb_*.state`: `OFF` / `LO` (13 V) / `HI` (18 V) / `LOT` (13 V + 22 kHz) /
  `HIT` (18 V + 22 kHz); `value` is the numeric code `0..4`.
- `rx[].state`: `search`, `header`, `DVB-S`, `DVB-S2`, `lost`, `timeout`,
  `idle`.
- `rx[].lock`: `1` when a signal is locked (DVB-S/S2), else `0`.
- `rx[].freq` / `lo` / `hardwarefreq`: MHz. `sr`: kS. `input`: `X` (top) /
  `Y` (bottom).

> Note: some readback fields (`lna_gain`, `power_*`, `const_*`, `ber`,
> `viterbi`, `ldpc_err`, `bch_*`, `puncture`) are populated by the receiver
> while a channel is locked; they read `0`/empty otherwise.

---

## `GET /api/config` — stored configuration

```json
{
  "dhcp": 1,
  "ip": "192.168.77.203", "sn": "255.255.255.0",
  "gw": "192.168.77.1",   "dns": "192.168.77.1",
  "hostname": "PicoTunerWH",
  "baseipport": 9900,
  "tsflash": 1,
  "debug_dhcp": 0,
  "lnb_x": 1, "lnb_y": 1, "lnb_autostart": 0
}
```

---

## `POST /api/config` — save configuration

Form fields (all optional; omitted fields are left unchanged):

| Field           | Meaning                                             |
|-----------------|-----------------------------------------------------|
| `dhcp`          | `1` = DHCP, `0` = static                            |
| `ip`,`sn`,`gw`,`dns` | dotted-quad, used when static                 |
| `hostname`      | up to 23 chars (sanitised)                          |
| `baseipport`    | base IP port (`0` or even `1100..65400`, `xx00..xx14`) |
| `tsflash`       | `0`/`1` activity-LED mode                           |
| `debug_dhcp`    | `0`/`1` verbose DHCP/debug output                   |
| `lnb_x`,`lnb_y` | LNB power at boot `0..4`                            |
| `lnb_autostart` | `0`/`1` apply the LNB states above at boot          |

Settings are written to flash and the device **reboots** to apply.

Response: `{"ok":true,"msg":"saved - rebooting"}` or
`{"ok":false,"msg":"<reason>"}`.

---

## `POST /api/lnb` — LNB power

| Field   | Meaning                                              |
|---------|------------------------------------------------------|
| `rx`    | `1` = X (top), `2` = Y (bottom)                      |
| `state` | `0` off, `1` 13 V, `2` 18 V, `3` 13 V + 22 kHz, `4` 18 V + 22 kHz |

The current state is reflected in `GET /api/status` (and is updated when a
remote program changes it).

Response: `{"ok":true,"msg":"LNB 1 set to HI"}`.

---

## `POST /api/tune` — tune a receiver

| Field   | Meaning                                       |
|---------|-----------------------------------------------|
| `rx`    | `1` or `2`                                    |
| `freq`  | satellite frequency in kHz (0 = stop)         |
| `lo`    | LNB local oscillator in kHz (e.g. 9750000)    |
| `sr`    | symbol rate in kS (e.g. 1500)                 |
| `fplug` | input `A` (X) or `B` (Y)                      |

Response: `{"ok":true,"msg":"tuning RX1 to 10491500 kHz"}`.

---

## `POST /api/nim` — tuner / LNA / tone / roll-off controls

One control per request. `rx` defaults to `1` where relevant.

| Field      | Device  | Meaning                                                |
|------------|---------|--------------------------------------------------------|
| `input`    | STV6120 | RF input select `0..3` = RF A/B/C/D (with `rx`)        |
| `bbgain`   | STV6120 | baseband gain `0..8` (0..16 dB in 2 dB steps)          |
| `lna`      | STVVGLNA| LNA index `0` (X) or `1` (Y); combine with:           |
| `gain`     | STVVGLNA| LNA gain/VGO `0..31`                                   |
| `mode`     | STVVGLNA| AGC mode `0..7` (AUTO_TRACK … MAX_EXT)                 |
| `pref`     | STVVGLNA| RFAGC target `0..7` (−25 dBm … −18 dBm)               |
| `tone`     | STV0910 | 22 kHz tone `0`/`1` (with `rx`)                        |
| `rolloff`  | STV0910 | roll-off control field `0..3` (with `rx`)             |

Response: `{"ok":true,"msg":"..."}`.

---

## `POST /api/diseqc` — send a DiSEqC message *(experimental)*

| Field  | Meaning                                     |
|--------|---------------------------------------------|
| `rx`   | `1` (drives LNB X / demod P2) or `2` (Y / P1) |
| `data` | hex bytes, e.g. `E0 10 38 F0`                |

Response: `{"ok":true,"msg":"DiSEqC 4 byte(s) sent on RX1 (experimental)"}`.

---

## `GET /api/reg` — read a NIM register

Query: `dev`, `addr`, `reg` (hex accepted, e.g. `0xf000`).

| `dev` | Device   | Register width | `addr`            |
|-------|----------|----------------|-------------------|
| `0`   | STV0910  | 16-bit         | ignored           |
| `1`   | STV6120  | 8-bit          | ignored           |
| `2`   | STVVGLNA | 8-bit          | I²C address (0xC8/0xCA/0xCC/0xCE) |

```json
{ "dev": 0, "reg": 61440, "val": "0x51", "err": 0 }
```

`err` is `0` on success, non-zero on I²C failure.

---

## `POST /api/reg` — write a NIM register

Form fields: `dev`, `addr`, `reg`, `val` (hex accepted).

Response: `{"dev":1,"reg":9,"err":0}`.

---

## `POST /api/reboot`

| `mode` | Action                              |
|--------|-------------------------------------|
| `0`    | reboot (keep settings)              |
| `1`    | reset settings to defaults + reboot |
| `2`    | enter USB BOOTSEL (firmware upload) |

Response: `{"ok":true,"msg":"rebooting"}`.

---

## Error responses

Action endpoints return HTTP `400` with
`{"ok":false,"msg":"<reason>"}` on invalid input. Unknown paths return HTTP
`404`.

## Concurrency / safety notes

- All NIM I²C traffic runs on core 1; register/control requests are marshalled
  from the web (core 0) to core 1 and can briefly block.
- Raw register writes and PLL/loop-filter changes can unlock a demod; use the
  named controls where possible.
- DiSEqC transmit is best-effort and marked experimental.
