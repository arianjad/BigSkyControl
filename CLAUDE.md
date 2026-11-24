# BigSkyControl

The Big Sky Nd:YAG GUI on the BeamBox lab PC: one window per laser, and the link that lets BLACS set and read the lasers. A fork of RaX's Big Sky GUI (`upstream/main`), kept on branch `beambox`. R&D prototype: rewrite freely.

## How it is built

- **Two files do the work.** `HugeSkyController.pyw` is the hub: it finds lasers on the COM ports, opens one tab per laser, and runs the BLACS link. `BigSkyControllerAmbitious.py` is one laser's tab (`SingleLaserController`): serial I/O, the buttons, the keep-warm rule, and the handlers BLACS's requests end up in.
- **The `.ui` file and the code are coupled by name.** `GuiBigSkyWidget.ui` is loaded with `uic.loadUiType`; each widget's `objectName` becomes `self.<name>`. Rename on both sides or the tab fails with `AttributeError`.
- **All serial I/O goes through `_sendCommand`.** It returns the answer or `None` and handles a dead port (`_handleDisconnect`, then a reconnect try every 5 s). Raw `self.ser` calls exist only in `_sendCommand`, `_attemptReconnect`, `_handleDisconnect` and `safeExit`. The laser sends `\r\n` before its answer, not after.
- **The laser's order.** Lamps (`>a`) before shutter (`>r1`); Q-switch (`>pq`) only with lamps and shutter on; lamp mode (`>lpm0/1`) and Q-switch mode (`>qsm0/1/2`) change only in standby (`>s`). Full command list: `CommandsForBigSky.pdf`; the ones in use are in the code.
- **Jobs report the step that failed.** Arm (`startLaser`): standby, lamp mode external, Q-switch mode internal, lamps, shutter, Q-switch. Warm-up: standby, lamp mode internal, lamps. Stop: standby. Each step needs an answer from the laser, and a mode step needs the read-back to match. A job stops at the first failed step and returns words like "arm failed at step 5 of 6 (shutter open, >r1): the laser did not answer >r1".
- **The BLACS link is RaX's RemoteControl protocol v2** (`bbx_zmq.zmq_v2`, RaX's file, from beambox-daq's `src`). REQ/REP on `tcp://*:55540`: `HELLO`, `PING`, `PROGRAM_VALUE`, `CHECK_VALUE`, one JSON object each way. PUB on `tcp://*:55541`, from its own thread so it keeps going while a request waits: every 250 ms `YAG_1_<name>_monitor <value>` for each laser whose serial link is up, and `heartbeat` once a second.
- **Names on the wire.** `YAG_1_<name>`, `<name>` one of `voltage` [V], `lamps`, `shutter`, `qswitch` (0/1), `lamp_mode` (0 internal, 1 external), `qswitch_mode` (0 internal, 1 burst, 2 external), `keep_warm` (0/1), `temperature` [degC, read only]; jobs `start_lasing`, `warmup`, `stop` (value 1 runs the job, 0 does nothing). A laser is `YAG_1`, `YAG_2` by `LASER_SN_TO_CONNECTION`, else by the order its tab was opened.
- **Answers.** A set or job runs on the tab's Qt thread (`executeRemoteCommand`); the request waits up to 10 s. `SUCCESS` carries what was set (voltage: the laser's read-back). The tab's refusals (`RemoteRefused`) come back `REJECTED` with its code and words, for example `lamps_not_active`, `qswitch_requires_lamps_and_shutter`, `lamp_mode_requires_standby`, `qswitch_mode_requires_standby`, `did_not_take_effect` (read-back differs), `no_answer`, `failed_at_step`. Not started in 10 s: `TIMEOUT` and it never runs later. Serial link down: `ERROR laser_disconnected`. Laser not open: `UNKNOWN_CONNECTION`. There is no range check here: the laser's read-back is the judge.
- **Keep warm lives here only.** With the box ticked (by hand or by BLACS), a laser below 37 degC gets one warm-up, then none until it was back at 39 degC; a laser with lamps, shutter and Q-switch on is left alone. The temperature is polled every 60 s.

## What can hurt

- **The hub writes `>sn` to every COM port** when it starts and on "Refresh Connections", except the ports in `NOT_LASERS` (COM10 the Quantum Composer, COM11 the PrawnBlaster), so it can start before or after BLACS. A new instrument on another COM port gets those bytes: add its port there. `--no-laser` opens no COM port.
- **The lab runs the GUI from the main folder `GUIs\BigSkyControl`.** Editing files there or switching its branch changes the running program's next start. Work in a git worktree.
- **A BLACS request moves the laser**: lamps fire, the shutter opens. The ports bind on all network interfaces, so anything that reaches this PC on 55540 can do the same.
- **Closing a tab or the hub sends `>s`** (standby) to that laser.
- **Keep warm fires the lamps on its own** when a ticked laser is cold.

## How we build

- **Upstream first.** When unsure, do it as RaX's Big Sky GUI does (`git show upstream/main:<file>`) and say so. What this fork changes: the step reports, `>lpm1` (upstream sends `>>lpm1`), the keep-warm rule that spares an armed laser and does not pause for BLACS, `--no-laser`, our `laserNames.pkl`, the three extra PUB values (`lamp_mode`, `qswitch_mode`, `keep_warm`), `failed_at_step` answers, PUB on its own thread.
- **The laser is the judge.** The GUI refuses only what the laser's order needs and a read-back that differs. Other ideas for checks go to the beambox-daq `TODO.md` as questions with default "not built".
- **The wire belongs to both sides.** A change to a name, code or port goes together with BLACS's Big Sky device in beambox-daq; an old BLACS talking to a new GUI gets `unknown_action`.
- **A GUI change reaches the widget.** A handler that changes the laser's state also sets its radio button, checkbox or label: a BLACS request or a reconnect has no click to do it.

## Working in the repo

This is its own git repo, inside beambox-daq's git-ignored `GUIs\`. Remotes: `origin` (our fork), `upstream` (RaX, fetch only).

- **Python:** the system `py -3.13` (PyQt5, pyzmq, pyserial, pytest). Its `bbx_zmq` is an editable install of beambox-daq's `src`; to use another checkout put its `src` first on `PYTHONPATH`.
- **Run:** `py -3.13 HugeSkyController.pyw` (scans the COM ports), `py -3.13 HugeSkyController.pyw --no-laser` (no COM port; one dummy `YAG_1` whose serial link is down, so BLACS gets the heartbeat and every set or read answers `laser_disconnected`).
- **Tests** (about 2 s, no serial port, no socket): `QT_QPA_PLATFORM=offscreen py -3.13 -m pytest -q tests`. `tests/test_zmq_link.py` runs the real hub link and laser tab over RaX's `InMemoryTransport` with a `FakeLaser` on the serial line; `BigSkyZmqServer.requestServer(transport)` and `publish_once(publish)` are the seams. `tests/test_serial_gateway.py` covers `_sendCommand`, disconnect and reconnect. A test earns its place by naming, in its docstring, the real harm it stops.
- `LabNotes_*.html/.md` and `DocumentationForBigSkyController.docx` are upstream history; the code is the reference.
