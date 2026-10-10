# Building on Windows

This documents a complete, from-scratch Windows build environment for
`m1rxo_v1_2` (PicoTuner-WH).

The build is driven by [`build.ps1`](../build.ps1) and produces
`picotunewh.uf2` plus a self-describing `m1rxo_v1_2-<chip>-<board>.uf2`.

---

## 1. What you need to download

| Component | Link | Notes / version |
|-----------|------|-----------------|
| **Git for Windows** | https://git-scm.com/download/win | any recent |
| **Python 3** | https://www.python.org/downloads/windows/ | 3.9+; tick *“Add python.exe to PATH”*. Required by the Pico SDK tooling. |
| **CMake** | https://cmake.org/download/ | 3.29.x — grab the **Windows x64 ZIP** (`cmake-3.29.3-windows-x86_64`) |
| **Ninja** | https://github.com/ninja-build/ninja/releases | `ninja-win.zip` (just `ninja.exe`) |
| **ARM GNU Toolchain** | https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads | **13.3.rel1**, *AArch32 bare-metal*, **mingw-w64 i686**, hosted on Windows — `arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi` |
| **Raspberry Pi Pico SDK** | https://github.com/raspberrypi/pico-sdk | 2.1.x (needed for RP2350). Clone **with submodules**. |
| **WIZnet-PICO-C** | https://github.com/WIZnet-ioNIC/WIZnet-PICO-C | provides `ioLibrary_Driver` and the Pico port layer. Clone **with submodules**. |

You do **not** need Visual Studio. `picotool` is fetched and built automatically
by the Pico SDK during the first configure (needs the ARM toolchain and Python).

---

## 2. Recommended directory layout

`build.ps1` expects the SDK and WIZnet sources as siblings of this repository,
and (by default) the tools under a `tools` folder:

```
C:\Users\<you>\src\
├─ pico-sdk\                         <- Raspberry Pi Pico SDK
├─ WIZnet-PICO-C\                    <- WIZnet-PICO-C
├─ Ptwh0v3e-source\                  <- this repository
└─ tools\
   ├─ cmake-3.29.3-windows-x86_64\   <- CMake ZIP, extracted
   ├─ mingw64\bin\ninja.exe          <- Ninja (mingw64 folder is only used to hold it)
   └─ arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi\
```

`build.ps1` looks for:

- `%PICO_TOOLS_DIR%\cmake-3.29.3-windows-x86_64\bin\cmake.exe`
- `%PICO_TOOLS_DIR%\mingw64\bin\ninja.exe`
- `%PICO_TOOLS_DIR%\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gcc.exe`

`PICO_TOOLS_DIR` defaults to `C:\Users\<you>\src\tools`. Anything not found
there is looked up on `PATH`, so you may instead install CMake/Ninja/the ARM
toolchain normally and skip the `tools` folder.

---

## 3. Set up the sources

```powershell
cd C:\Users\<you>\src

# Raspberry Pi Pico SDK (RP2040 + RP2350 support) and its submodules
git clone https://github.com/raspberrypi/pico-sdk.git --branch 2.1.1 --recursive

# WIZnet ioLibrary + Pico port layer
git clone https://github.com/WIZnet-ioNIC/WIZnet-PICO-C.git --recursive
```

> If you cloned without `--recursive`, run
> `git -C pico-sdk submodule update --init --recursive` (and the same for
> `WIZnet-PICO-C`).

---

## 4. Extract the tools (if using the `tools` layout)

From the downloaded ZIPs, extract so the folder names match the layout above:

- **CMake**: `cmake-3.29.3-windows-x86_64.zip` → `tools\cmake-3.29.3-windows-x86_64\`
- **Ninja**: `ninja-win.zip` → put `ninja.exe` into `tools\mingw64\bin\`
  (create the folder; the tools folder name is historical)
- **ARM toolchain**: `arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi.zip`
  → `tools\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi\`

No installer or PATH changes are required for this layout — `build.ps1`
prepends the right `bin` folders.

---

## 5. Build

From the repository root:

```powershell
# Default: pico2 (RP2350) / W5500 / W5500_EVB_PICO2
powershell -ExecutionPolicy Bypass -File .\build.ps1

# Clean, or configure only
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Clean
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Configure
```

Other variants (each needs its own build directory):

```powershell
# RP2040 / W5500
.\build.ps1 -BuildDir build-w5500-pico -PicoBoard pico -WiznetChip W5500 -WiznetBoard W5500_EVB_PICO

# RP2350 / W6100
.\build.ps1 -BuildDir build-w6100-pico2 -PicoBoard pico2 -WiznetChip W6100 -WiznetBoard W6100_EVB_PICO2

# RP2040 / W5100S
.\build.ps1 -BuildDir build-w5100s-pico -PicoBoard pico -WiznetChip W5100S -WiznetBoard W5100S_EVB_PICO
```

Board names come from WIZnet-PICO-C's
[`port/board_list.h`](https://github.com/WIZnet-ioNIC/WIZnet-PICO-C/blob/main/port/board_list.h)
(`W5100S_EVB_PICO`, `W5500_EVB_PICO2`, `W6100_EVB_PICO`, …).

Successful output ends with:

```
Build OK: ...\build\picotunewh.uf2 (n bytes)
Variant  : ...\build\m1rxo_v1_2-w5500-pico2.uf2
```

Drag `m1rxo_v1_2-<chip>-<board>.uf2` to the board (hold **BOOTSEL**, plug USB).

---

## 6. Environment overrides

`build.ps1` honours these environment variables:

| Variable | Purpose | Default |
|----------|---------|---------|
| `PICO_SDK_PATH` | Pico SDK location | `..\pico-sdk` |
| `WIZNET_DIR` | `ioLibrary_Driver` sources | `..\WIZnet-PICO-C\libraries\ioLibrary_Driver` |
| `PORT_DIR` | WIZnet Pico port layer | `..\WIZnet-PICO-C\port` |
| `PICO_TOOLS_DIR` | folder holding the tools | `C:\Users\<you>\src\tools` |

---

## 7. Troubleshooting

- **“Could not locate 'cmake'/'ninja'/'arm-none-eabi-gcc'”** — either extract
  the tools into `%PICO_TOOLS_DIR%` with the exact folder names above, or
  install them and put them on `PATH`.
- **`picotool` build errors on first configure** — the SDK fetches and builds
  `picotool`; make sure Python 3 is on `PATH` and the ARM toolchain is found.
- **Wrong image flashed** — the firmware detects an incorrect chip and flashes
  `NO` in Morse (`-. ---`) on the LEDs. Flash the image matching your WIZnet
  chip **and** MCU.
- **No web UI on W5100S** — the W5100S only has 4 hardware sockets; the web
  server is compiled out on that target.
