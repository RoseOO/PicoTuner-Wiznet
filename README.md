# m1rxo_v1_2 — PicoTuner-WH

Standalone **WinterHill-mode** dual-channel DVB‑S/S2 receiver firmware for the
WIZnet **W5100S / W5500 / W6100 EVB‑Pico and EVB‑Pico2** boards
(RP2040 and RP2350), with a built-in web configuration UI and a small JSON API.

It is derived from the LongMynd receiver (Heather Lomond) and the WinterHill
project (BATC / G4EWJ).

---

## Supported builds

| Image (release asset)              | WIZnet chip | Board                | MCU     |
|------------------------------------|-------------|----------------------|---------|
| `m1rxo_v1_2-w5100s-pico.uf2`       | W5100S      | W5100S-EVB-Pico      | RP2040  |
| `m1rxo_v1_2-w5100s-pico2.uf2`      | W5100S      | W5100S-EVB-Pico2     | RP2350  |
| `m1rxo_v1_2-w5500-pico.uf2`        | W5500       | W5500-EVB-Pico       | RP2040  |
| `m1rxo_v1_2-w5500-pico2.uf2`       | W5500       | W5500-EVB-Pico2      | RP2350  |
| `m1rxo_v1_2-w6100-pico.uf2`        | W6100       | W6100-EVB-Pico       | RP2040  |
| `m1rxo_v1_2-w6100-pico2.uf2`       | W6100       | W6100-EVB-Pico2      | RP2350  |

Flash by holding **BOOTSEL**, plugging in USB, and dragging the matching
`.uf2` onto the mass-storage volume. Pick the image for **both** your WIZnet
chip and your MCU (`pico` = RP2040, `pico2` = RP2350).

The firmware detects whether the wrong chip image was flashed and flashes
`NO` in Morse on the LEDs if so.

> The web server is only built for chips with spare hardware sockets
> (W5500/W6100 = 8 sockets). The W5100S (4 sockets) has no web UI.

---

## Features

- **Two simultaneous DVB‑S / DVB‑S2 channels** (tune + receive), TS streamed
  over UDP to the requesting client (WinterHill protocol).
- **NIM support**: STV0910 dual demodulator + STV6120 tuner + STVVGLNA LNA
  (`FTS4334L` with LNA, `FTS4335` without).
- **Network**: DHCP or static IP; settings persisted in flash and only applied
  after a reboot.
- **LNB control**: independent X (top) and Y (bottom) feeds — off / 13 V / 18 V
  with optional 22 kHz tone.
- **Web UI** at `http://<device-ip>/`: network config, LNB power, tuning,
  tuner-chip / LNA controls, DiSEqC, a raw register console, and live status.
- **JSON API** (see [`docs/API.md`](docs/API.md)) for scripting/automation.
- **Live status**: per-receiver lock state, MER, frequency, symbol rate,
  modulation/code, roll-off, power, BER and error counters, packet counts.
- **Broadcast**: a status packet is sent every second on UDP port `9997`.
- **Raw NIM register access** to the STV0910 / STV6120 / STVVGLNA over the API.

### Network ports (base IP port 9900)

| Port            | Direction | Purpose                            |
|-----------------|-----------|------------------------------------|
| 9920            | in        | system commands (WinterHill)       |
| 9921 / 9922     | in        | RX1 / RX2 tuning commands          |
| 9941 / 9942     | out       | RX1 / RX2 transport stream         |
| 9901/9902/9903/9904 | out   | info streams                       |
| 9997            | out       | broadcast status (every second)    |
| 80              | in        | web UI / JSON API (W5500/W6100)    |

---

## Web UI & API quick start

Open `http://<device-ip>/` in a browser. The JSON API is available under the
same host, e.g.:

```bash
# status (network, LNB, per-receiver tuner state)
curl http://<device-ip>/api/status

# set LNB X to 18 V + 22 kHz
curl -X POST -d 'rx=1&state=4' http://<device-ip>/api/lnb

# tune RX1
curl -X POST -d 'rx=1&freq=10491500&lo=9750000&sr=1500&fplug=A' http://<device-ip>/api/tune

# read a NIM register (STV0910 MID)
curl 'http://<device-ip>/api/reg?dev=0&addr=0&reg=0xf000'
```

Full reference: [`docs/API.md`](docs/API.md).

---

## Building

Requires the Raspberry Pi Pico SDK, the WIZnet-PICO-C sources, CMake/Ninja and
the ARM GNU toolchain. See `build.ps1` (Windows) — the defaults build the
W5500-EVB-Pico2 (RP2350). Override per invocation:

```powershell
# default: pico2 / W5500 / W5500_EVB_PICO2
.\build.ps1

# another variant
.\build.ps1 -BuildDir build-w6100 -PicoBoard pico -WiznetChip W6100 -WiznetBoard W6100_EVB_PICO
```

Each build emits `m1rxo_v1_2-<chip>-<board>.uf2` alongside `picotunewh.uf2`.

---

## License / attribution

Derived from LongMynd (Copyright 2019 Heather Lomond, GPLv3) and the BATC
WinterHill project (Copyright Brian Jordan, G4EWJ). The NIM control software
is an implementation of the Serit NIM controlling software. See source headers
for the full notices.
