"""B1-B4: serial gateway + disconnect/reconnect invariants.

Run:
    conda activate guis
    pytest GUIs/BigSkyControl/tests/test_serial_gateway.py -v
    # or standalone:
    python GUIs/BigSkyControl/tests/test_serial_gateway.py
"""
from __future__ import annotations

import serial


# --- B1 -------------------------------------------------------------------

def test_B1_send_command_serial_exception_calls_disconnect(
        bsa_module, make_controller_self, monkeypatch):
    """SerialException in `ser.write` -> `_handleDisconnect` called, None returned.
    _consecutiveErrors is NOT incremented for hard serial errors (only for
    soft empty/decode failures); disconnect is immediate."""
    s = make_controller_self(raises=serial.SerialException("port gone"))
    called = []
    monkeypatch.setattr(s, "updateAllStatusIndicators", lambda: called.append("disc"))
    # Replace _handleDisconnect with a probe -- the real one mutates the
    # whole namespace, B3 covers it separately.
    s._handleDisconnect = lambda reason="": called.append(("disc", reason))

    out = bsa_module.SingleLaserController._sendCommand(s, b">cg\n")

    assert out is None
    assert any(c[0] == "disc" for c in called if isinstance(c, tuple)), called


def test_B1_send_command_oserror_calls_disconnect(
        bsa_module, make_controller_self):
    s = make_controller_self(raises=OSError(22, "fake"))
    called = []
    s._handleDisconnect = lambda reason="": called.append(reason)

    out = bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    assert out is None and len(called) == 1


# --- B2 -------------------------------------------------------------------

def test_B2_three_consecutive_empty_responses_trigger_disconnect(
        bsa_module, make_controller_self):
    """Three consecutive empty reads -> _handleDisconnect; not two, not four."""
    s = make_controller_self(read_returns=b"")
    triggered = []
    s._handleDisconnect = lambda reason="": triggered.append(reason)

    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    assert triggered == [], "must NOT disconnect after only 2 empties"
    assert s._consecutiveErrors == 2

    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    assert len(triggered) == 1


def test_B2_three_consecutive_decode_errors_trigger_disconnect(
        bsa_module, make_controller_self):
    """Decode errors are 'soft' (counted) like empties, not hard like
    SerialException -- gives transient noise on the line a chance to clear."""
    s = make_controller_self(read_returns=b"\xff\xfe\xff invalid utf8")
    triggered = []
    s._handleDisconnect = lambda reason="": triggered.append(reason)

    for _ in range(3):
        out = bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
        assert out is None
    assert len(triggered) == 1, "exactly one disconnect on the 3rd decode failure"


def test_B2_successful_response_resets_counter(
        bsa_module, make_controller_self):
    """A good response between bad ones must reset _consecutiveErrors to 0,
    so 'two empties, one good, two empties' is NOT a disconnect."""
    s = make_controller_self(read_returns=b"")
    s._handleDisconnect = lambda **k: None
    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    assert s._consecutiveErrors == 2

    s.ser.read.return_value = b">cg ok\r\n"
    out = bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    assert out is not None
    assert s._consecutiveErrors == 0


# --- B3 -------------------------------------------------------------------

def test_B3_handle_disconnect_is_idempotent(
        bsa_module, make_controller_self):
    """Second call must early-return; no double GUI updates, no double
    timer.start, no double signal emission."""
    s = make_controller_self(connected=False)  # already disconnected
    bsa_module.SingleLaserController._handleDisconnect(s, "test")
    assert s.connectionStatusChanged.emit.call_count == 0
    assert s._reconnectTimer.start.call_count == 0


def test_B3_first_call_runs_full_teardown(bsa_module, make_controller_self):
    s = make_controller_self(connected=True)
    bsa_module.SingleLaserController._handleDisconnect(s, "real")
    assert s.serialConnected is False
    s.connectionStatusChanged.emit.assert_called_once_with(False)
    s._reconnectTimer.start.assert_called_once_with(5000)
    assert s.activeStatus == 0 and s.shutterStatus == 0 and s.qSwitchStatus == 0


# --- B4 -------------------------------------------------------------------

def test_B4_handle_reconnect_resets_consecutive_errors(
        bsa_module, make_controller_self):
    """After reconnect, _consecutiveErrors must be 0 so the very next
    transient empty response isn't read as already-2-of-3 toward disconnect."""
    s = make_controller_self(connected=False, consecutive_errors=2)
    bsa_module.SingleLaserController._handleReconnect(s, initial_temp=35.0)
    assert s._consecutiveErrors == 0
    assert s.serialConnected is True
    s._reconnectTimer.stop.assert_called_once()
    s.connectionStatusChanged.emit.assert_called_once_with(True)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))


# --- read timing and stale answers ------------------------------------------

def test_each_command_clears_old_input_and_the_port_ends_a_read_at_the_gap(
        bsa_module, make_controller_self, hsc_module):
    """Harm stopped: every Big Sky command waiting the full 1 s timeout (each BLACS set or arm
    step about 1 s late), or a late answer to an earlier command read as this one's."""
    s = make_controller_self()
    bsa_module.SingleLaserController._sendCommand(s, b">cg\n")
    names = [c[0] for c in s.ser.method_calls]
    assert names.index("reset_input_buffer") < names.index("write") < names.index("read")
    assert 0 < bsa_module.READ_GAP_S < 1 and hsc_module.READ_GAP_S is bsa_module.READ_GAP_S


def test_a_laser_not_in_the_serial_number_list_never_gets_a_yag_name(hsc_module):
    """Harm stopped: an unknown laser launched first named YAG_1, so BLACS's YAG_1 arm (lamps
    fire, the shutter opens) goes to the wrong laser."""
    import types
    hub = types.SimpleNamespace()
    name = hsc_module.MyTableWidget._assignConnectionName
    assert name(hub, "151") == "YAG_1" and name(hub, "082") == "SN_082"
    assert not name(hub, "999").startswith("YAG_")
