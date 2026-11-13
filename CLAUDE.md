# BigSky Laser Controller - Project Context

## Environment

- **Python on the BeamBox lab PC (MITBeamBox)**: the system Python, `py -3.13` (PyQt5, pyzmq,
  pyserial, pytest installed). Its `bbx_zmq` is an editable install of the beambox-daq repo's
  `src`. To run against another checkout (a worktree), put that `src` first: `PYTHONPATH=<checkout>\src`.
- (Upstream RaX PC: conda env `guis`, `/c/Users/radmo/miniconda/envs/guis/python.exe`.)
- **Framework**: PyQt5 with Qt Designer `.ui` files loaded via `uic.loadUiType`

## How to Run

```bash
# Multi-laser hub (production use): scans the COM ports (opens each one and writes >sn)
py -3.13 HugeSkyController.pyw

# No-laser mode: no COM port is scanned or opened; one dummy laser YAG_1 is open to BLACS
py -3.13 HugeSkyController.pyw --no-laser

# Single-laser standalone (no BLACS link)
py -3.13 BigSkyControllerAmbitious.py

# Tests (no serial port, no socket; runs from any folder)
PYTHONPATH=<beambox-daq>\src QT_QPA_PLATFORM=offscreen py -3.13 -m pytest -q tests
```

Without a laser connected, a laser tab runs in dummy mode with no serial port.

## Project Structure

| File | Purpose |
|------|---------|
| `HugeSkyController.pyw` | Main hub - discovers lasers on COM ports, creates tabbed interface per laser |
| `BigSkyControllerAmbitious.py` | `SingleLaserController` class - per-laser control widget (the main code) |
| `GuiBigSkyWidget.ui` | Qt Designer XML defining the widget layout (loaded by `uic.loadUiType`) |
| `laserNames.pkl` | Pickle file mapping serial numbers to user-assigned laser labels |
| `CalibrationFiles/` | Per-laser calibration CSVs (voltage -> power mapping) |
| `Big Sky LabView/` | Reference LabView code from SteimleLab (Greg Hall, Nov 2018) |
| `Big Sky YAG Manual.pdf` | Hardware manual for the BigSky Nd:YAG laser |

## Critical Convention: UI Widget Names

The `.ui` file and `.py` file are tightly coupled through `uic.loadUiType`. Widget `name` attributes in the XML become `self.<name>` attributes in Python. If you add a widget to the `.ui` file, you **must** use the exact `objectName` when referencing it in Python, and vice versa. A mismatch causes `AttributeError` at runtime.

## Serial Communication

- **Baud rate**: 9600, timeout 1 second
- **Command format**: `>command\n` (e.g., `>a\n` to activate)
- **Response quirk**: BigSky controller sends `\r\n` *before* the response message (not after). The existing code handles this by reading 140 bytes and stripping `\r\n`. See `SendCommand.vi` notes in the LabView reference for the double-write/double-read approach used historically.

### Serial Error Handling Convention

All serial I/O **must** route through `_sendCommand(cmd_bytes)` in `BigSkyControllerAmbitious.py`. This method:
- Returns the response string on success, or `None` on failure
- Catches `SerialException`, `OSError`, `UnicodeDecodeError`
- Calls `_handleDisconnect()` on any serial error

Raw `self.ser.*` calls should only appear in four places:
1. `_sendCommand()` itself
2. `_attemptReconnect()` (needs to test the port directly)
3. `safeExit()` cleanup (wrapped in try/except)
4. `_handleDisconnect()` (`self.ser.close()` on the dead port)

Every caller of `_sendCommand()` must check for `None` return and bail out (typically `if response is None: return`). Response parsing (`int()`, `float()` casts) should be wrapped in `try/except ValueError`.

### Serial Command Reference

| Command | Description | Response format |
|---------|-------------|-----------------|
| `>a` | Activate lamps (start firing) | |
| `>s` | Standby (stop everything) | |
| `>r1` / `>r0` | Open / close shutter | |
| `>pq` / `>sq` | Enable / disable Q-switch | |
| `>oq` | Single Q-switch pulse | |
| `>cg` | Query temperature | `temp. CG XX.X deg` |
| `>v` | Query voltage | `voltage XXXX V` |
| `>vmo####` | Set voltage (e.g., `>vmo0725`) | `voltage m XXXX V` |
| `>f` / `>f####` | Query / set frequency (units: Hz * 100) | `freq. XXXX Hz` |
| `>ene` | Query flashlamp energy | `energy X.XX J` |
| `>sn` | Query serial number | `s/number XXX` |
| `>qsm` / `>qsm0/1/2` | Query / set Q-switch mode (0=internal, 1=burst, 2=external) | `QS mode : X` |
| `>lpm` / `>lpm0/1` | Query / set lamp mode (0=internal, 1=external) | `LP synch : X` |
| `>sav1` | Save current settings to laser EEPROM | |

## Operational Parameters

- **Temperature thresholds**: <37C = too cold (blue), 37-39C = warming (gold), >=39C = operating (green)
- **Warmup voltage**: 725V is operator convention (below lasing threshold) — `startWarmup()` does **not** set voltage; whatever is currently programmed stays in effect
- **Voltage range**: 500-1400V
- **Max frequency**: 56 Hz (30 Hz is comfortable for external trigger)
- **External trigger**: Pin 4 (+) and Pin 9 (-) on 9-pin serial connector, +5V 100us pulse
- **Warmup procedure**: Fire lamps with shutter closed until temp > 37C. Normal operating temp ~39C.

## Laser States and Safety

The laser has three independent subsystems that must be activated in order:
1. **Lamps** (`>a` / `>s`) - Flashlamps must be firing first
2. **Shutter** (`>r1` / `>r0`) - Shutter can only open when lamps are active
3. **Q-Switch** (`>pq` / `>sq`) - Q-switch can only arm when lamps active AND shutter open

Mode changes (Q-switch mode, lamp mode) require the laser to be in standby first.

## Testing Notes

- The GUI can run without a laser connected (dummy mode, or `--no-laser` for the hub) for UI testing
- The `dangerMode` flag is set to `True` on connection and gates the manual Q-switch/single-pulse buttons
- `QTimer` polls the temperature every 60 s while the serial link is up (also drives keep-warm)
- `tests/test_menu_server.py`: the real BLACS client against the real menu server and laser
  controller, in memory, with a `FakeLaser` on the serial line. `tests/test_serial_gateway.py`:
  `_sendCommand` / disconnect / reconnect on duck-typed objects.

## BLACS Integration (menu link, BBX-189, 2026-11-12)

BLACS (beambox-daq, device `bbx_devices/BigSkyHub`) talks to this GUI through one shared
**menu**: `bbx_zmq/menus/bigsky.py` in the beambox-daq repo. BLACS and this GUI load the same
file: the names, ranges, buttons, time limits and the port (REP **55540**; no PUB stream, no
heartbeat). Wire format: the v2 request/reply envelope of `bbx_zmq.zmq_v2`; actions HELLO,
SNAPSHOT, SET, PRESS.

- `HugeSkyController.pyw`: `BigSkyMenuServer` (thread, port) and `BigSkyMenuLink` (the handlers).
  The menu server (`bbx_zmq.MenuServer`) refuses a name not on the menu, a laser that is not
  open and a value out of range **before** any GUI code runs. This GUI is the one guard for the
  laser. A set or press then runs on the laser tab's Qt thread (`executeRemoteCommand` ->
  `_handleRemoteCommand`), and the answer says what really happened.
- Names per laser, `<laser>_<name>` (`YAG_1_voltage`); a laser is open to BLACS as YAG_1, YAG_2
  (`LASER_SN_TO_CONNECTION`, else tab order). Hello answers the menu name/version and the open lasers.
- Values (all in every snapshot; set = BLACS may set it): `voltage` [V] 500-1400 set, `lamps`,
  `shutter`, `qswitch`, `lamp_mode` (0 int, 1 ext), `qswitch_mode` (0 int, 1 burst, 2 ext) set,
  `temperature` [degC] read only, `keep_warm` set. The GUI refuses: shutter open with lamps
  off, Q-switch on without lamps + shutter, a mode change unless standby, a laser whose serial
  link is down; a set the laser does not answer or reads back differently is refused too.
- Buttons: `arm`, `warm_up`, `stop` = `startLaser`, `startWarmup`, `stopLaser`. Each runs its
  steps in order and stops at the first one that fails; the answer is "done" or a refusal
  `failed_at_step` with words like "arm failed at step 5 of 6 (shutter open, >r1): the laser did
  not answer >r1". Arm: standby, lamp mode external, **Q-switch mode internal**, lamps, shutter,
  Q-switch. Warm-up: standby, lamp mode internal, lamps. Stop: standby. A mode step needs the
  mode the laser reports back to match.
- **Keep warm lives here only** (BLACS just ticks `keep_warm`): below 37 degC a warm-up, then
  not again until 39 degC; an armed laser (lamps, shutter, Q-switch on) is left alone.
- **Auto Arm Ext lives in BLACS** (a tick box per laser in the Big Sky tab): it presses `arm`
  before a shot (unless the snapshot shows the laser armed already) and checks lamps, shutter and
  Q-switch in the next snapshot; it presses `warm_up` at queue end and on abort.
- Changing a name, range or button: change the menu file and its `version` (BLACS refuses to run
  shots when the versions differ).

**Remote command GUI sync convention:** Every function that changes hardware state (e.g., `setFlashLampExternal`, `toggleShutterStatus`) must also update the corresponding GUI widget (radio button, checkbox, label). When called by user click, the widget is already correct. But when called via ZMQ remote command (`_handleRemoteCommand` -> `_remoteDo`), the widget won't update unless the function explicitly sets it. This also applies to disconnect/reconnect transitions — `_handleDisconnect()` and `_handleReconnect()` must update all GUI elements since no user click triggers the change. Failure to sync causes the GUI to show stale state.

## Disconnection Resilience

- **Detection**: `SerialException`/`OSError` in `_sendCommand()` triggers `_handleDisconnect()` immediately; an empty response returns `None` and only disconnects after 3 consecutive empties (`_consecutiveErrors`) — isolated `None` returns leave status CONNECTED. `_handleDisconnect()` resets all state and shows "DISCONNECTED" in dark red
- **Auto-reconnect**: `_reconnectTimer` fires every 5 seconds, validates with `>cg` temperature query, calls `_handleReconnect()` on success
- **State restore**: `_handleReconnect()` re-queries voltage, frequency, modes, energy from the laser
- **Signal**: `connectionStatusChanged(bool)` signal emitted on connect/disconnect — hub uses this to gray out / restore tab text
- **BLACS link**: a set or press for a laser whose serial link is down is refused (`laser_disconnected`); a snapshot still reports its cached (reset) values

**If changing what BLACS can set, read or press**: change the menu (`bbx_zmq/menus/bigsky.py` in beambox-daq), bump its version, and change the handlers in `HugeSkyController.pyw` / `BigSkyControllerAmbitious.py` together.
