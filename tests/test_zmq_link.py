"""The Big Sky GUI's BLACS link (RaX RemoteControl v2), end to end in memory.

BLACS's requests (zmq_v2.encode_request) go over RaX's InMemoryTransport to the real request
server of HugeSkyController.pyw, which runs the real SingleLaserController code on the Qt main
thread, as in the GUI. The PUB stream is read through `publish_once(frames.append)`. Only the
laser is fake: a FakeLaser object stands on the serial line. No serial port is opened, no socket
is bound. Each test docstring names the harm it stops.

Run from this folder:  QT_QPA_PLATFORM=offscreen py -3.13 -m pytest -q tests
"""
from __future__ import annotations

import json
import os
import re
import threading
import time
import types

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
zmq_v2 = pytest.importorskip("bbx_zmq.zmq_v2", reason="needs beambox-daq src (bbx_zmq)")


class FakeLaser:
    """Stands in for serial.Serial on a Big Sky: answers each '>cmd' as the laser does, keeps
    the lamp and Q-switch modes, and records what it got. `silent_at`: a command it does not
    answer (empty read, as a laser that went quiet). `lamp_mode_stuck`: the lamp mode it reports
    whatever it is told. `voltage_stuck`: the voltage it reports whatever it is told."""

    def __init__(self, temperature=38.5, silent_at=None, lamp_mode_stuck=None, voltage_stuck=None):
        self.sent, self.temperature = [], temperature
        self.silent_at, self.lamp_mode_stuck, self.voltage_stuck = silent_at, lamp_mode_stuck, voltage_stuck
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
            self.voltage = int(m.group(1)) if self.voltage_stuck is None else self.voltage_stuck
            return "voltage m %04d V" % self.voltage
        if cmd == ">v":
            return "voltage %04d V" % self.voltage
        if cmd == ">cg":     # what updateTemp's strip('\r\ntemp.CG d') parses
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
    """The GUI's link with one laser YAG_1 (a real SingleLaserController on `laser`).
    `gui.ask(action, connection, value)` sends one BLACS request and serves it on another thread
    while this (the Qt main) thread runs the GUI's event loop, as in the GUI; returns the reply."""
    import BigSkyControllerAmbitious as bsa

    ctrl = bsa.SingleLaserController(cPort=-1, lString="YAG_1")   # no serial port is opened
    ctrl.ser, ctrl.serialConnected, ctrl.dangerMode = laser, True, True
    link = hsc_module.BigSkyZmqServer()
    link.addLaser("YAG_1", ctrl)
    blacs_end, gui_end = zmq_v2.InMemoryTransport.pair()
    server = link.requestServer(gui_end)

    def ask(action, connection="", value=None, raw=None):
        blacs_end.send(raw or zmq_v2.encode_request(action, request_id=7, connection=connection,
                                                    value=value))
        t = threading.Thread(target=server.serve_once, kwargs={"timeout_ms": 0}, daemon=True)
        t.start()
        while t.is_alive():
            app.processEvents()
            time.sleep(0.001)
        return json.loads(blacs_end.recv(timeout_ms=0))

    yield types.SimpleNamespace(ctrl=ctrl, link=link, server=server, blacs=blacs_end, ask=ask)
    ctrl.safeExit()
    ctrl.deleteLater()


def set_(gui, name, value):
    return gui.ask("PROGRAM_VALUE", name, value)


ARMED = {"lamps": 1, "shutter": 1, "qswitch": 1, "lamp_mode": 1, "qswitch_mode": 0}


def read_armed(gui):
    return {k: gui.ask("CHECK_VALUE", "YAG_1_" + k)["value"] for k in ARMED}


def test_start_lasing_reports_success_only_when_every_step_is_done_in_the_safe_order(gui, laser):
    """Harm stopped: the YAG is reported armed while it is not, or armed out of order (a mode
    change while the lamps fire is refused by the laser; a shutter before the lamps), or with the
    Q-switch on burst / external trigger. start_lasing sends standby, lamp mode external (>lpm1,
    not upstream's >>lpm1), Q-switch mode internal, lamps, shutter, Q-switch, in that order, and
    CHECK_VALUE then shows it armed."""
    reply = set_(gui, "YAG_1_start_lasing", 1)
    assert (reply["status"], reply["id"]) == ("SUCCESS", 7)
    assert [c for c in laser.sent if c != ">qsm"] == [">s", ">lpm1", ">qsm0", ">a", ">r1", ">pq"]
    assert read_armed(gui) == ARMED


def test_start_lasing_that_fails_at_a_step_is_rejected_with_the_step(gui, laser):
    """Harm stopped: an arm that gave up half way (the laser did not answer the shutter command)
    is answered SUCCESS, so BLACS runs the shot with no YAG pulse; or it fails with no word of
    where. The reply is REJECTED, code failed_at_step, its words name the step, and no later step
    (the Q-switch) is sent."""
    laser.silent_at = ">r1"
    reply = set_(gui, "YAG_1_start_lasing", 1)
    assert (reply["status"], reply["error"]["code"]) == ("REJECTED", "failed_at_step")
    assert "arm failed at step 5 of 6 (shutter open" in reply["error"]["message"]
    assert ">pq" not in laser.sent


def test_start_lasing_stops_before_the_lamps_when_the_lamp_mode_did_not_change(gui, laser):
    """Harm stopped: the flashlamps fire on the internal trigger when the arm asked for external
    (the laser kept its old lamp mode): the YAG then pulses out of step with the experiment. The
    arm fails at the lamp-mode step and the lamps are never switched on."""
    laser.lamp_mode_stuck = 0
    reply = set_(gui, "YAG_1_start_lasing", 1)
    assert reply["status"] == "REJECTED"
    assert "arm failed at step 2 of 6 (lamp mode external" in reply["error"]["message"]
    assert ">a" not in laser.sent


def test_warmup_and_stop_say_which_step_failed(gui, laser):
    """Harm stopped: a warm-up or stop that did not happen is reported done (BLACS's queue end
    then leaves the laser armed, shutter open, or a Stop click leaves the lamps firing). warmup
    sends standby, lamp mode internal, lamps; a silent laser makes it fail with the step; stop
    likewise."""
    assert set_(gui, "YAG_1_warmup", 1)["status"] == "SUCCESS"
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
    assert [gui.ask("CHECK_VALUE", "YAG_1_" + k)["value"] for k in ("lamps", "shutter", "lamp_mode")] == [1, 0, 0]
    laser.silent_at = ">s"
    reply = set_(gui, "YAG_1_warmup", 1)
    assert reply["status"] == "REJECTED"
    assert "warm_up failed at step 1 of 3 (standby" in reply["error"]["message"]
    reply = set_(gui, "YAG_1_stop", 1)
    assert reply["status"] == "REJECTED"
    assert "stop failed at step 1 of 1 (standby" in reply["error"]["message"]


def test_a_voltage_is_answered_with_what_the_laser_reads_back(gui, laser):
    """Harm stopped: BLACS records a flashlamp voltage the laser did not take (it clamped or
    ignored the set), so the shot file says 1500 V for a shot fired at another voltage. A set is
    answered SUCCESS with the read-back; a different read-back is REJECTED with both numbers."""
    reply = set_(gui, "YAG_1_voltage", 1050)
    assert (reply["status"], reply["value"]) == ("SUCCESS", 1050)
    assert laser.sent == [">vmo1050"]
    laser.voltage_stuck = 1400
    reply = set_(gui, "YAG_1_voltage", 1500)
    assert (reply["status"], reply["error"]["code"]) == ("REJECTED", "did_not_take_effect")
    assert reply["error"]["message"] == "asked 1500 V, the laser reports 1400 V"
    assert gui.ask("CHECK_VALUE", "YAG_1_voltage_monitor")["value"] == 1400


def test_the_gui_refuses_a_shutter_before_the_lamps_and_a_laser_that_is_gone(gui, laser):
    """Harm stopped: BLACS opens the shutter with the lamps off, or a set to a laser whose serial
    link dropped (or one not open in this GUI) looks done, or a read gives a stale value as if
    measured. The GUI answers each with its code and words; nothing is sent to the laser."""
    reply = set_(gui, "YAG_1_shutter", 1)
    assert (reply["status"], reply["error"]["code"]) == ("REJECTED", "lamps_not_active")
    assert set_(gui, "YAG_2_voltage", 900)["status"] == "UNKNOWN_CONNECTION"
    gui.ctrl.serialConnected = False
    for reply in (set_(gui, "YAG_1_start_lasing", 1), gui.ask("CHECK_VALUE", "YAG_1_voltage")):
        assert (reply["status"], reply["error"]["code"]) == ("ERROR", "laser_disconnected")
    assert laser.sent == []


def test_a_set_that_outlasts_the_time_limit_never_reaches_the_laser_late(gui, laser, hsc_module, monkeypatch, app):
    """Harm stopped: BLACS is told TIMEOUT and moves on, and the set then reaches the laser
    seconds later (lamps switched on after the shot was given up). A set the laser tab has not
    started within the limit is answered TIMEOUT and is never run."""
    monkeypatch.setattr(hsc_module, "_REMOTE_CMD_TIMEOUT_S", 0.05)
    gui.blacs.send(zmq_v2.encode_request("PROGRAM_VALUE", connection="YAG_1_lamps", value=1))
    t = threading.Thread(target=gui.server.serve_once, kwargs={"timeout_ms": 0})
    t.start()
    t.join()                                 # the Qt thread runs no events meanwhile: a busy tab
    reply = json.loads(gui.blacs.recv(timeout_ms=0))
    assert (reply["status"], reply["error"]["code"]) == ("TIMEOUT", "command_timeout")
    app.processEvents()
    assert laser.sent == []
    assert gui.ask("CHECK_VALUE", "YAG_1_lamps")["value"] == 0


def test_hello_lists_the_open_lasers_and_every_value_can_be_read(gui):
    """Harm stopped: a BLACS table for YAG_1 runs on a GUI without it, or BLACS's check of the
    armed state cannot read a value it needs. HELLO names the open lasers as RaX's prefix
    pattern; every value reads by name and by its _monitor name."""
    hello = gui.ask("HELLO")
    assert (hello["status"], hello["protocol_version"], hello["connections"]) == ("SUCCESS", 2, ["YAG_1_*"])
    for name in ("voltage", "lamps", "shutter", "qswitch", "lamp_mode", "qswitch_mode",
                 "temperature", "keep_warm"):
        for wire in ("YAG_1_" + name, "YAG_1_%s_monitor" % name):
            assert gui.ask("CHECK_VALUE", wire)["status"] == "SUCCESS", wire


def test_the_stream_carries_every_value_and_a_heartbeat_once_a_second(gui):
    """Harm stopped: BLACS's shot file misses a Big Sky value (no record of the temperature, the
    lamp mode or keep-warm during a shot), or BLACS shows the GUI gone while it runs (no
    heartbeat). Each turn publishes the eight values of each laser whose serial link is up; every
    fourth turn (once a second at 250 ms) adds "heartbeat"; a laser whose link is down is left out."""
    frames = []
    for _ in range(4):
        gui.link.publish_once(frames.append)
    assert frames.count("heartbeat") == 1 and frames[-1] == "heartbeat"
    first = dict(f.split(" ") for f in frames[:8])
    assert sorted(first) == sorted("YAG_1_%s_monitor" % n for n in (
        "voltage", "lamps", "shutter", "qswitch", "lamp_mode", "qswitch_mode", "temperature", "keep_warm"))
    assert (first["YAG_1_qswitch_mode_monitor"], first["YAG_1_keep_warm_monitor"]) == ("0", "0")
    gui.ctrl.serialConnected = False
    frames.clear()
    for _ in range(4):
        gui.link.publish_once(frames.append)
    assert frames == ["heartbeat"]


def test_old_or_foreign_requests_are_refused_and_reach_no_laser(gui, laser):
    """Harm stopped: during the change-over, a BLACS still on the menu link (SET, PRESS) or on
    RaX's v1 protocol has its write half understood and the laser moves, or the write looks done.
    Both are answered ERROR and nothing is sent to the laser."""
    assert gui.ask("SET", "YAG_1_lamps", 1)["error"]["code"] == "unknown_action"
    reply = gui.ask(None, raw=json.dumps({"action": "PROGRAM_VALUE", "connection": "YAG_1_lamps",
                                          "value": 1}).encode())
    assert reply["error"]["code"] == "v1_protocol_refused"
    assert laser.sent == []


def test_keep_warm_warms_a_cold_laser_once_and_never_an_armed_one(gui, laser):
    """Harm stopped: the keep-warm rule (in the GUI only) lets the YAG go cold below 37 degC, or
    starts warm-ups over and over while it stays cold, or drops an armed laser out of its armed
    state in the middle of a run (shutter closed, lamps on internal trigger). Set from BLACS, the
    rule warms a cold laser once, again only after it was back at 39 degC, and leaves an armed
    laser alone."""
    laser.temperature = 39.0
    gui.ctrl.pollTemperature()                                     # a real reading first
    assert set_(gui, "YAG_1_keep_warm", 1)["value"] == 1
    assert [c for c in laser.sent if c != ">cg"] == []             # warm: nothing to do
    laser.temperature = 36.5
    gui.ctrl.pollTemperature()
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
    laser.sent.clear()
    gui.ctrl.pollTemperature()                                     # still cold: no second warm-up
    laser.temperature = 39.2
    gui.ctrl.pollTemperature()                                     # warm again: the rule re-arms
    assert [c for c in laser.sent if c != ">cg"] == []
    set_(gui, "YAG_1_start_lasing", 1)
    laser.sent.clear()
    laser.temperature = 36.0
    gui.ctrl.pollTemperature()                                     # cold but armed: left alone
    assert [c for c in laser.sent if c != ">cg"] == []
    set_(gui, "YAG_1_stop", 1)
    laser.sent.clear()
    gui.ctrl.pollTemperature()                                     # cold, standby: warm-up
    assert [c for c in laser.sent if c not in (">cg", ">qsm")] == [">s", ">lpm0", ">a"]
