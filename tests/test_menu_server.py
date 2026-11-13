"""BBX-189: the Big Sky GUI as BLACS's menu server, end to end in memory.

The real BLACS client (bbx_zmq Client) talks over the in-memory link to the real menu server of
HugeSkyController.pyw (BigSkyMenuLink), which runs the real SingleLaserController code on the Qt
main thread, as in the GUI. Only the laser is fake: a FakeLaser object stands on the serial line
(no serial port is opened, no socket is bound). Each test docstring names the harm it stops.

Run from this folder, with the beambox-daq src first on the path (its bbx_zmq has the menus):
    PYTHONPATH=<beambox-daq>\\src QT_QPA_PLATFORM=offscreen py -3.13 -m pytest -q tests
"""
from __future__ import annotations

import os
import re
import threading
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
menu_link = pytest.importorskip("bbx_zmq.menus.bigsky", reason="needs beambox-daq src with the menus")


class FakeLaser:
    """Stands in for serial.Serial on a Big Sky: answers each '>cmd' as the laser does, keeps
    the lamp and Q-switch modes, and records what it got. `silent_at`: a command it does not
    answer (empty read, as a laser that went quiet). `lamp_mode_stuck`: the lamp mode it reports
    whatever it is told."""

    def __init__(self, temperature=38.5, silent_at=None, lamp_mode_stuck=None):
        self.sent, self.temperature = [], temperature
        self.silent_at, self.lamp_mode_stuck = silent_at, lamp_mode_stuck
        self.lamp_mode, self.qswitch_mode, self.voltage = 0, 2, 900
        self._reply = b""

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        self._reply = b"" if cmd == self.silent_at else ("\r\n" + self._answer(cmd)).encode()
        return len(data)

    def read(self, n):
        reply, self._reply = self._reply, b""
        return reply

    def _answer(self, cmd):
        if m := re.fullmatch(r">lpm(\d?)", cmd):
            if m.group(1):
                self.lamp_mode = int(m.group(1)) if self.lamp_mode_stuck is None else self.lamp_mode_stuck
            return "LP synch :  %d" % self.lamp_mode
        if m := re.fullmatch(r">qsm(\d?)", cmd):
            if m.group(1):
                self.qswitch_mode = int(m.group(1))
            return "QS mode :  %d" % self.qswitch_mode
        if m := re.fullmatch(r">vmo(\d{4})", cmd):
            self.voltage = int(m.group(1))
            return "voltage m %04d V" % self.voltage
        if cmd == ">v":
            return "voltage %04d V" % self.voltage
        if cmd == ">cg":     # what updateTemp's strip('\r\ntemp.CG d') parses (exact laser text: live run)
            return "temp. CG %.1f" % self.temperature
        return "ok"


@pytest.fixture(scope="module")
def app():
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def laser():
    return FakeLaser()


@pytest.fixture
def gui(app, hsc_module, laser):
    """The GUI's menu server with one laser YAG_1 (a real SingleLaserController on `laser`),
    served on a thread as in the GUI; `gui.call(fn)` runs a client call on another thread while
    this (the Qt main) thread runs the GUI's event loop, as the GUI does."""
    import BigSkyControllerAmbitious as bsa
    from bbx_zmq import Client, InMemoryTransport

    ctrl = bsa.SingleLaserController(cPort=-1, lString="YAG_1")   # no serial port is opened
    ctrl.ser, ctrl.serialConnected, ctrl.dangerMode = laser, True, True
    link = hsc_module.BigSkyMenuLink()
    link.addLaser("YAG_1", ctrl)
    blacs_end, gui_end = InMemoryTransport.pair()
    server = link.menuServer(gui_end)
    stop = threading.Event()
    thread = threading.Thread(target=lambda: [server.serve_once(timeout_ms=10) for _ in iter(stop.is_set, True)],
                              daemon=True)
    thread.start()
    client = Client(menu_link.MENU, lambda: blacs_end, margin_s=0.5)

    def call(fn, *args):
        box = {}

        def run():
            try:
                box["value"] = fn(*args)
            except Exception as e:          # noqa: BLE001  (handed back to the test)
                box["error"] = e
        t = threading.Thread(target=run, daemon=True)
        t.start()
        while t.is_alive():
            app.processEvents()
            time.sleep(0.001)
        if "error" in box:
            raise box["error"]
        return box["value"]

    class Gui:
        pass
    g = Gui()
    g.ctrl, g.link, g.client, g.call = ctrl, link, client, call
    call(client.hello)
    yield g
    stop.set()
    thread.join(timeout=2)
    ctrl.safeExit()
    ctrl.deleteLater()


ARMED = {"YAG_1_lamps": 1, "YAG_1_shutter": 1, "YAG_1_qswitch": 1, "YAG_1_lamp_mode": 1,
         "YAG_1_qswitch_mode": 0}


def test_an_arm_reports_done_only_when_every_step_is_done_in_the_safe_order(gui, laser):
    """Harm stopped: the YAG is reported armed while it is not, or armed out of order (a mode
    change while the lamps fire is refused by the laser; a shutter before the lamps), or with the
    Q-switch on burst / external trigger. An arm sends standby, lamp mode external, Q-switch mode
    internal, lamps, shutter, Q-switch, in that order, and the snapshot shows it armed."""
    assert gui.call(gui.client.press, "YAG_1_arm") is None
    assert [c for c in laser.sent if c != ">qsm"] == [">s", ">lpm1", ">qsm0", ">a", ">r1", ">pq"]
    snap = gui.call(gui.client.snapshot)
    assert {k: snap[k] for k in ARMED} == ARMED


def test_an_arm_that_fails_at_a_step_is_reported_as_failed_with_the_step(gui, laser):
    """Harm stopped: an arm that gave up half way (the laser did not answer the shutter command)
    is answered "OK", so BLACS runs the shot with no YAG pulse; or it fails with no word of where.
    The arm is refused with code failed_at_step, its words name the step, and no later step (the
    Q-switch) is sent."""
    from bbx_zmq import Refused
    laser.silent_at = ">r1"
    with pytest.raises(Refused) as e:
        gui.call(gui.client.press, "YAG_1_arm")
    assert e.value.code == "failed_at_step"
    assert "arm failed at step 5 of 6 (shutter open" in e.value.words
    assert ">pq" not in laser.sent


def test_an_arm_stops_before_the_lamps_when_the_lamp_mode_did_not_change(gui):
    """Harm stopped: the flashlamps fire on the internal trigger when the arm asked for external
    (the laser kept its old lamp mode): the YAG then pulses out of step with the experiment. The
    arm fails at the lamp-mode step and the lamps are never switched on."""
    from bbx_zmq import Refused
    laser = gui.ctrl.ser
    laser.lamp_mode_stuck = 0
    with pytest.raises(Refused, match="arm failed at step 2 of 6 \\(lamp mode external"):
        gui.call(gui.client.press, "YAG_1_arm")
    assert ">a" not in laser.sent


def test_warm_up_and_stop_say_which_step_failed(gui, laser):
    """Harm stopped: a warm-up or stop that did not happen is reported done (BLACS's queue end
    then leaves the laser armed, shutter open, or a stop click leaves the lamps firing). Warm-up
    sends standby, lamp mode internal, lamps; a silent laser makes it fail with the step; stop
    likewise."""
    from bbx_zmq import Refused
    gui.call(gui.client.press, "YAG_1_warm_up")
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
    snap = gui.call(gui.client.snapshot)
    assert (snap["YAG_1_lamps"], snap["YAG_1_shutter"], snap["YAG_1_lamp_mode"]) == (1, 0, 0)
    laser.silent_at = ">s"
    with pytest.raises(Refused, match="warm_up failed at step 1 of 3 \\(standby"):
        gui.call(gui.client.press, "YAG_1_warm_up")
    with pytest.raises(Refused, match="stop failed at step 1 of 1 \\(standby"):
        gui.call(gui.client.press, "YAG_1_stop")


def test_a_voltage_out_of_range_is_refused_before_the_handler_runs(gui, laser):
    """Harm stopped: a flashlamp voltage outside 500..1400 V (a typo, kV for V) reaches the laser.
    The menu check refuses it before any GUI code runs: nothing goes to the laser. A voltage in
    range is set and its read-back answered."""
    from bbx_zmq import Refused
    for bad in (1500, 499, 900.5):
        with pytest.raises(Refused) as e:
            gui.call(gui.client.set, "YAG_1_voltage", bad)
        assert e.value.code == "out_of_range"
    assert laser.sent == []
    assert gui.call(gui.client.set, "YAG_1_voltage", 1050) == 1050
    assert laser.sent == [">vmo1050"]


def test_the_gui_refuses_a_shutter_before_the_lamps_and_a_laser_that_is_gone(gui, laser):
    """Harm stopped: BLACS opens the shutter with the lamps off, or a set to a laser whose serial
    link dropped looks done. The GUI refuses both with its own code and words; nothing is sent."""
    from bbx_zmq import Refused
    with pytest.raises(Refused) as e:
        gui.call(gui.client.set, "YAG_1_shutter", 1)
    assert e.value.code == "lamps_not_active"
    gui.ctrl.serialConnected = False
    with pytest.raises(Refused) as e:
        gui.call(gui.client.press, "YAG_1_arm")
    assert e.value.code == "laser_disconnected"
    assert laser.sent == []


def test_hello_and_snapshot_give_the_lasers_and_every_menu_value(gui):
    """Harm stopped: BLACS's shot file misses a Big Sky value (no record of the temperature or
    the Q-switch mode during a shot), or a BLACS table for YAG_1 runs on a GUI without it. Hello
    names the open lasers; a snapshot has every menu value for each."""
    assert gui.call(gui.client.hello)["units"] == ["YAG_1"]
    snap = gui.call(gui.client.snapshot)
    assert sorted(snap) == sorted(f"YAG_1_{v.name}" for v in menu_link.MENU.values)
    assert snap["YAG_1_keep_warm"] == 0 and snap["YAG_1_voltage"] == 0


def test_keep_warm_warms_a_cold_laser_once_and_never_an_armed_one(gui, laser):
    """Harm stopped: the keep-warm rule (in the GUI only) lets the YAG go cold below 37 degC, or
    starts warm-ups over and over while it stays cold, or drops an armed laser out of its armed
    state in the middle of a run (shutter closed, lamps on internal trigger). Set from BLACS, the
    rule warms a cold laser once, again only after it was back at 39 degC, and leaves an armed
    laser alone."""
    laser.temperature = 39.0
    gui.ctrl.pollTemperature()                                     # a real reading first
    gui.call(gui.client.set, "YAG_1_keep_warm", 1)
    assert [c for c in laser.sent if c != ">cg"] == []             # warm: nothing to do
    laser.temperature = 36.5
    gui.ctrl.pollTemperature()
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
    laser.sent.clear()
    gui.ctrl.pollTemperature()                                     # still cold: no second warm-up
    laser.temperature = 39.2
    gui.ctrl.pollTemperature()                                     # warm again: the rule re-arms
    assert [c for c in laser.sent if c != ">cg"] == []
    gui.call(gui.client.press, "YAG_1_arm")
    laser.sent.clear()
    laser.temperature = 36.0
    gui.ctrl.pollTemperature()                                     # cold but armed: left alone
    assert [c for c in laser.sent if c != ">cg"] == []
    gui.call(gui.client.press, "YAG_1_stop")
    laser.sent.clear()
    gui.ctrl.pollTemperature()                                     # cold, standby: warm-up
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
