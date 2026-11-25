#written by Alex Brinson (brinson@mit.edu, alexjbrinson@gmail.com) on behalf of EMA Lab
import sys
import re
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot
import serial
import time
import numpy as np
import os
import concurrent.futures
import threading

# A read ends this long after the laser's last byte, not at the 1 s timeout. Each answer is CR LF
# + 15 characters (CommandsForBigSky.pdf, page 10 of 19): 17 bytes, about 18 ms at 9600 baud.
READ_GAP_S = 0.05   # [s]


def read_reply(ser):
  """One answer, as bytes. The read ends READ_GAP_S after the last byte. The laser sends CR LF
  first: if only that came, it may still be working, so read once more."""
  data = ser.read(140)
  if data and not data.strip():
    data += ser.read(140)
  return data

# Match the trailing integer in a BigSky response (e.g. "LP synch :  1\r\n" → 1).
# Tolerant of leading/trailing whitespace and varying prefix punctuation so we
# don't have to hard-code each command's exact response prefix.
_TRAILING_INT_RE = re.compile(r'(-?\d+)\s*$')


class RemoteRefused(Exception):
  """A remote (BLACS) request this laser could not do: `code` for programs, `words` for people.
  HugeSkyController answers it to BLACS as REJECTED with this code and these words."""
  def __init__(self, code, words):
    super().__init__("[%s] %s" % (code, words))
    self.code = code
    self.words = words

qtCreatorFile = "GuiBigSkyWidget.ui" # Enter file here.

Ui_Widget, QtBaseClass = uic.loadUiType(qtCreatorFile)

class SingleLaserController(QtWidgets.QWidget, Ui_Widget):
  #Signal for thread-safe remote command execution from ZMQ daemon thread
  _remoteCommandRequested = pyqtSignal(str, object, object)  # (command, value, future)
  connectionStatusChanged = pyqtSignal(bool)  # emitted on disconnect/reconnect

  def __init__(self, cPort=-1, lString=''):
    super().__init__()
    self.setupUi(self)
    self.comPort = cPort
    self.labelString=lString

    self.calibrationFilePresent=False #TODO: check for calibration file based on laser head serial number

    #Testing different possible serial ports to see if any of them is a Big Sky laser. If ">cg" evokes a temperature readout, we found a live one.
    #cPort=-1 (no laser): no serial port is opened here or later (_attemptReconnect only runs
    #after a disconnect of a port that was open).
    self.serialConnected = False
    self.dangerMode = False
    self._consecutiveErrors = 0
    if self.comPort!=-1:
      try:
        self.ser = serial.Serial(self.comPort, 9600, timeout=1, inter_byte_timeout=READ_GAP_S); self.fetchSerial()
        tiempo = time.strftime("%d %b %Y %H:%M:%S",time.localtime())#
        self.terminalOutputTextBrowser.append('Connection established at '+str(tiempo))
        self.serialConnected=True
        self.dangerMode = True
      except:
        self.terminalOutputTextBrowser.append('Connection failed... Investigate if this ever happens')

    if self.serialConnected==False:
      print("Error: Laser not found. Ensure laser is on and check serial port connection.")
      self.fLampVoltage=-1
      self.serialNumber=''
      #quit()

    #Initializing dummy values. These are updated to true laser settings once all widgets are connected, so they can be updated too.
    self.qSwitchMode = 0; self.flashLampMode = 0
    self.activeStatus = 0; self.shutterStatus = 0; self.qSwitchStatus = 0
    self.terminalEnabled = False
    self.proposedEnergy = 7; self.proposedVoltage = 500; self.proposedFrequency = 0; self.fLampVoltage=0

    #Thread-safe state lock for ZMQ accessor methods
    self._stateLock = threading.Lock()
    self._remoteCommandRequested.connect(self._handleRemoteCommand)

    #Warmup / keep-warm state
    self.warmupActive = False
    self.keepWarmActive = False
    self._warmupTriggered = False  # hysteresis flag for Auto Keep Warm
    self.lastTemperature = 0.0
    self.tempPollTimer = QTimer(self)
    self.tempPollTimer.timeout.connect(self.pollTemperature)
    self.TEMP_COLD = 37.0
    self.TEMP_OPERATING = 39.0

    #Reconnect timer — tries to re-establish serial every 5s after disconnect
    self._reconnectTimer = QTimer(self)
    self._reconnectTimer.timeout.connect(self._attemptReconnect)

    #Latched energy readback — set when voltage changes, consumed by next temp poll
    self._energyReadbackPending = False

   #Initializing GUI values

   #Checking for self.calibration file in local directory
    try:
      cwd = os.getcwd()
      if self.serialConnected:
        self.calibData=np.loadtxt(cwd+"\\CalibrationFiles\\CalibrationDataBigSky"+str(self.serialNumber)+".csv",dtype="float",comments='#',delimiter=',')
      else:
        self.calibData=np.loadtxt(cwd+"\\CalibrationFiles\\CalibrationDataBigSky.csv",dtype="float",comments='#',delimiter=',')
      self.calibVolts = self.calibData[:,0]; self.calibPower = self.calibData[:,1]
      self.calibrationFilePresent=True
    except:
      defaultCalibVolts=[800,900,950,1000,1050,1080]
      defaultCalibPower=[0.05,1.54,3.09,4.73,6.14,6.78]
    if self.calibrationFilePresent: print("self.calibration file loaded successfully")
    else: print("failed to load self.calibration file"); self.calibVolts=defaultCalibVolts; self.calibPower=defaultCalibPower
    self.PowerEstimateValue.setText('%.2f'%np.interp(self.fLampVoltage,self.calibVolts,self.calibPower)+" W")

    if self.serialConnected:
      self.label.setText(self.labelString)#("BIG SKY " + str(self.comPort) + " LASER CONTROL")
      self.updateTemp()
      self.update_fLampMode()
      self.update_qSwitchMode()
      #self.update_fLampValues()
      self.update_fLampVoltage()
      self.update_fLampEnergy()
      self.lastUpdateOutput.setText(str(tiempo))#
      self.updateFreq()
      # Always-on temperature polling (60s interval, safe no-op if serial disconnects)
      self.tempPollTimer.start(60000)
    else: self.label.setText("Laser not found. This is a dummy GUI\n"+self.labelString)

    self.frequencyDoubleSpinBox.setEnabled(not(self.flashLampMode));
    self.frequencyConfirmationButton.setEnabled(not(self.flashLampMode))
    self.updateAllStatusIndicators()

    #Connecting signals to slots
    self.frequencyDoubleSpinBox.valueChanged.connect(self.setFrequency)
    self.frequencyDoubleSpinBox.editingFinished.connect(self.setFrequency)
    self.qSwitchRadioButton_0.clicked.connect(self.setQSwitchInternal)
    self.qSwitchRadioButton_1.clicked.connect(self.setQSwitchBurst)
    self.qSwitchRadioButton_2.clicked.connect(self.setQSwitchExternal)
    self.flashLampRadioButton_0.clicked.connect(self.setFlashLampInternal)
    self.flashLampRadioButton_1.clicked.connect(self.setFlashLampExternal)
    self.flashLampVoltageSpinBox.valueChanged.connect(self.setVoltage)
    self.voltageConfirmationButton.clicked.connect(self.confirmVoltageSetting)
    self.frequencyConfirmationButton.clicked.connect(self.confirmFrequencySetting)
    self.laserSaveButton.clicked.connect(self.saveLaserSettings)

    #Independent controls
    self.lampToggleButton.clicked.connect(self.toggleActiveStatus)
    self.shutterToggleButton.clicked.connect(self.toggleShutterStatus)
    self.qSwitchToggleButton.clicked.connect(self.toggleQSwitchStatus)
    self.singlePulseButton.clicked.connect(self.singlePulse)

    #Compound controls
    self.warmupButton.clicked.connect(self.startWarmup)
    self.startLasingButton.setText("Arm External")
    self.startLasingButton.clicked.connect(self.startLaser)
    self.fullStopButton.clicked.connect(self.stopLaser)

    #Keep warm — auto-poll temperature and enter warmup if cold
    self.keepWarmCheckBox.toggled.connect(self.toggleKeepWarm)

    self.toggleInputButton.clicked.connect(self.toggleTerminalInput)
    self.terminalInputLineEdit.textChanged.connect(self.updateTerminalCommand)
    self.terminalInputLineEdit.returnPressed.connect(self.sendTerminalCommand)
    self.terminalInputLabel.setEnabled(False); self.terminalInputLineEdit.setEnabled(False)


  # --- Serial I/O gateway ---

  def _sendCommand(self, cmd_bytes):
    """Send a command to the laser and return the response string, or None on failure.

    All serial I/O must route through this method. On repeated failures,
    calls _handleDisconnect(). Callers must check for None return.
    """
    if not self.serialConnected:
      return None
    try:
      self.ser.reset_input_buffer()    # a late answer to an earlier command is not read as this one's
      self.ser.write(cmd_bytes if isinstance(cmd_bytes, bytes) else bytes(cmd_bytes, "utf-8"))
      response = read_reply(self.ser).decode('utf-8')
      if not response.strip():
        self._consecutiveErrors += 1
        if self._consecutiveErrors >= 3:
          self._handleDisconnect("3 consecutive empty responses")
        return None
      self._consecutiveErrors = 0
      return response
    except (serial.SerialException, OSError) as e:
      self._handleDisconnect("serial error: %s" % e)
      return None
    except UnicodeDecodeError as e:
      self._consecutiveErrors += 1
      if self._consecutiveErrors >= 3:
        self._handleDisconnect("3 consecutive decode errors: %s" % e)
      return None

  def _handleDisconnect(self, reason=""):
    """Handle serial disconnection: reset state, update GUI, start reconnect timer."""
    if not self.serialConnected:
      return  # already disconnected
    self.serialConnected = False
    self._consecutiveErrors = 0

    # Reset all cached hardware state
    with self._stateLock:
      self.activeStatus = 0
      self.shutterStatus = 0
      self.qSwitchStatus = 0
      self.flashLampMode = 0
      self.qSwitchMode = 0
    self.warmupActive = False
    if self.keepWarmActive:
      self.keepWarmActive = False
      self._warmupTriggered = False
      self.keepWarmCheckBox.blockSignals(True)
      self.keepWarmCheckBox.setChecked(False)
      self.keepWarmCheckBox.blockSignals(False)

    # Close the dead serial port
    try:
      self.ser.close()
    except Exception:
      pass

    # Update GUI
    self.label.setText("DISCONNECTED \u2014 " + self.labelString)
    self.terminalOutputTextBrowser.append(
        "<p style='color: red'>Serial disconnected: %s</p>" % reason)
    self.updateAllStatusIndicators()
    self.overallStatusLabel.setText("OVERALL: DISCONNECTED")
    self._setLabelColor(self.overallStatusLabel, bg="#8B0000", fg="white")

    # Start reconnect attempts
    self._reconnectTimer.start(5000)

    # Notify hub/ZMQ
    self.connectionStatusChanged.emit(False)
    print("Serial disconnected: %s" % reason)

  def _attemptReconnect(self):
    """Try to re-establish serial connection. Called by _reconnectTimer every 5s."""
    try:
      self.ser = serial.Serial(self.comPort, 9600, timeout=1, inter_byte_timeout=READ_GAP_S)
      self.ser.reset_input_buffer()
      self.ser.write(b'>cg\n')
      response = read_reply(self.ser).decode('utf-8')
      temp = float(response.strip('\r\ntemp.CG d'))
      self._handleReconnect(temp)
    except Exception:
      # Still disconnected — timer will retry
      try:
        self.ser.close()
      except Exception:
        pass

  def _handleReconnect(self, initial_temp):
    """Restore state after successful reconnection."""
    self._reconnectTimer.stop()
    self.serialConnected = True
    self._consecutiveErrors = 0

    with self._stateLock:
      self.lastTemperature = initial_temp

    self.terminalOutputTextBrowser.append(
        "<p style='color: green'>Serial reconnected! Re-querying laser state...</p>")

    # Re-query all laser state
    self.update_fLampVoltage()
    self.updateFreq()
    self.update_fLampMode()
    self.update_qSwitchMode()
    self.update_fLampEnergy()
    self.updateTemp()

    # Restore GUI
    self.label.setText(self.labelString)
    self.updateAllStatusIndicators()

    self.connectionStatusChanged.emit(True)
    print("Serial reconnected to %s" % self.comPort)

  def isConnected(self):
    """Return serial connection status. Thread-safe (GIL-atomic bool read)."""
    return self.serialConnected

  # --- Laser commands ---

  def setFrequency(self):
    self.proposedFrequency = float(self.frequencyDoubleSpinBox.value())

  def confirmFrequencySetting(self):
    toWrite = ">f{freq}\n".format(freq = str(int(self.proposedFrequency*100)) )
    self.terminalOutputTextBrowser.append(">f{freq}".format(freq = str(int(self.proposedFrequency*100)) ))#this is just a test feature
    response = self._sendCommand(toWrite)
    if response is None: return
    try:
      self.frequency=float(response.strip('\r\nfreq. Hz'))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>Frequency parse error</p>"); return
    self.frequencyDoubleSpinBox.setValue(self.frequency); print("self.frequency = {f}Hz".format(f=self.frequency))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    self.updateTemp()

  def updateFreq(self):
    if not self.serialConnected: return
    self.terminalOutputTextBrowser.append('>f')
    response = self._sendCommand(b'>f\n')
    if response is None: return
    try:
      self.frequency=float(response.strip('\r\nfreq. Hz'))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>Frequency parse error</p>"); return
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    print("self.frequency = {f}Hz".format(f=self.frequency))
    self.frequencyDoubleSpinBox.setValue(self.frequency)

  def saveLaserSettings(self):
    self.terminalOutputTextBrowser.append('>sav1')
    response = self._sendCommand(b'>sav1\n')
    if response is not None:
      self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    print("Laser settings saved")

  '''NOTE: These can only be changed while laser is in standby (>s). The GUI should now reproduce this behavior'''
  def setQSwitchInternal(self):
    return self._setQSwitchMode(0)
  def setQSwitchBurst(self):
    return self._setQSwitchMode(1)
  def setQSwitchExternal(self):
    return self._setQSwitchMode(2)

  def _setQSwitchMode(self, target):
    """Send >qsm{target} and verify the mode the laser reports (from the reply, or from a >qsm
    query when the reply carries no number). Returns the same result dicts as _setLampMode.
    The cached qSwitchMode and the radio buttons follow the reported mode."""
    cmd = '>qsm%d' % target
    response = self._sendCommand((cmd + '\n').encode('ascii'))
    if response is None:
      return {"status": "REJECTED", "code": "serial_failure", "message": "serial failure on %s" % cmd}
    self.terminalOutputTextBrowser.append(cmd)
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    m = _TRAILING_INT_RE.search(response.strip())
    if m is None:                                   # no number in the reply: ask the laser
      response = self._sendCommand(b'>qsm\n')
      m = _TRAILING_INT_RE.search(response.strip()) if response is not None else None
    if m is None:
      return {"status": "REJECTED", "code": "parse_failure",
              "message": "could not read the Q-switch mode back after %s" % cmd}
    actual = int(m.group(1))
    with self._stateLock: self.qSwitchMode = actual
    radio = {0: self.qSwitchRadioButton_0, 1: self.qSwitchRadioButton_1, 2: self.qSwitchRadioButton_2}.get(actual)
    if radio is not None: radio.setChecked(True)
    if actual != target:
      return {"status": "REJECTED", "code": "did_not_take_effect",
              "message": "%s did not take effect (the laser reports %d)" % (cmd, actual)}
    return {"status": "SUCCESS"}
  def _setLampMode(self, target, cmd_label):
    """Send >lpm{target} and verify the controller's reported mode.

    On success (actual == target): returns SUCCESS, caches the actual value,
    updates radio buttons + frequency interlock.
    On serial failure: returns ERROR with 'rejected: serial failure' so the
    ZMQ caller (BLACS) doesn't update its cache to the requested value.
    On parse failure: returns ERROR with 'rejected: could not parse'; cache
    unchanged.
    On verify mismatch (actual != target): returns ERROR with 'rejected: ...
    did not take effect'. The cached flashLampMode is set to the *actual*
    reported value (so callers like ``startLaser`` see reality).
    """
    cmd_bytes = ('%s\n' % cmd_label).encode('ascii')   # cmd_label is '>lpm0' / '>lpm1' (BBX-189: was '>>lpm1')
    print(cmd_label)
    response = self._sendCommand(cmd_bytes)
    if response is None:
      msg = "rejected: serial failure on %s" % cmd_label
      return {"status": "REJECTED", "code": "serial_failure", "message": msg}
    m = _TRAILING_INT_RE.search(response.strip())
    if m is None:
      msg = "rejected: could not parse %s response (%r)" % (cmd_label, response)
      self.terminalOutputTextBrowser.append(
          "<p style='color: red'>%s — flashLampMode unchanged</p>" % msg)
      return {"status": "REJECTED", "code": "parse_failure", "message": msg}
    actual = int(m.group(1))
    with self._stateLock: self.flashLampMode = actual
    if actual == 0:
      self.flashLampRadioButton_0.setChecked(True)
      self.frequencyDoubleSpinBox.setEnabled(True); self.frequencyConfirmationButton.setEnabled(True)
    elif actual == 1:
      self.flashLampRadioButton_1.setChecked(True)
      self.frequencyDoubleSpinBox.setEnabled(False); self.frequencyConfirmationButton.setEnabled(False)
    else:
      self.terminalOutputTextBrowser.append(
          "<p style='color: orange'>Unexpected lamp mode reported: %d</p>" % actual)
    print("response:", response)
    self.terminalOutputTextBrowser.append(cmd_label)
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    if actual != target:
      msg = "rejected: %s did not take effect (got %d)" % (cmd_label, actual)
      self.terminalOutputTextBrowser.append(
          "<p style='color: orange'>Warning: %s</p>" % msg)
      return {"status": "REJECTED",
              "code": "did_not_take_effect", "message": msg}
    return {"status": "SUCCESS"}

  def setFlashLampInternal(self):
    return self._setLampMode(0, '>lpm0')

  def setFlashLampExternal(self):
    return self._setLampMode(1, '>lpm1')

  def setVoltage(self):
    self.proposedVoltage = int(self.flashLampVoltageSpinBox.value())

  def confirmVoltageSetting(self):
    realUpdate=False
    try:
      self.proposedVoltage = int(self.flashLampVoltageSpinBox.value())
      if self.proposedVoltage<500 or self.proposedVoltage>1400:
        print("please enter an integer between 500 and 1400"); self.proposedVoltage = self.fLampVoltage
      else: realUpdate=True
    except: print("please enter an integer value."); self.proposedVoltage = self.fLampVoltage
    if realUpdate:
      toWrite = ">vmo{vol}\n".format( vol = str(0)+str(int(self.proposedVoltage)) if self.proposedVoltage<1000 else str(int(self.proposedVoltage)) )
      self.terminalOutputTextBrowser.append(toWrite.strip('\n'))
      response = self._sendCommand(toWrite)
      if response is not None:
        try:
          with self._stateLock: self.fLampVoltage=int(response.strip('\r\nvoltage m V'))
        except ValueError:
          self.terminalOutputTextBrowser.append("<p style='color: orange'>Voltage parse error</p>"); return
        print("voltage = {V}V".format(V=self.fLampVoltage))
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
        self.flashLampVoltageSpinBox.setValue(self.fLampVoltage)
        self._energyReadbackPending = True  # deferred to next temp poll
        self.PowerEstimateValue.setText('%.2f'%np.interp(self.fLampVoltage,self.calibVolts,self.calibPower) + " W")
      # On timeout (None): leave cache unchanged — don't assume command succeeded
    else:
      self.flashLampVoltageSpinBox.setValue(self.fLampVoltage)

  def toggleActiveStatus(self):
    if not self.activeStatus:
      # Activating
      print(">a")
      self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>a'+"</p>");
      response = self._sendCommand(b'>a\n')
      if response is not None:
        with self._stateLock: self.activeStatus = 1
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    else:
      # Standby
      print(">s")
      self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>s'+"</p>");
      response = self._sendCommand(b'>s\n')
      if response is not None:
        with self._stateLock: self.activeStatus = 0; self.shutterStatus = 0; self.qSwitchStatus = 0
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    self.updateAllStatusIndicators()

  def toggleShutterStatus(self):
    if not self.shutterStatus:
      # Opening shutter
      print(">r1")
      self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>r1'+"</p>");
      response = self._sendCommand(b'>r1\n')
      if response is not None:
        with self._stateLock: self.shutterStatus = 1
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    else:
      # Closing shutter
      print(">r0")
      self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>r0'+"</p>");
      response = self._sendCommand(b'>r0\n')
      if response is not None:
        with self._stateLock: self.shutterStatus = 0
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    self.updateAllStatusIndicators()

  def toggleQSwitchStatus(self):
    if self.qSwitchStatus:
      # Disarming Q-switch
      print(">sq"); self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>sq'+"</p>");
      response = self._sendCommand(b'>sq\n')
      if response is not None:
        with self._stateLock: self.qSwitchStatus = 0
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    else:
      # Arming Q-switch
      print(">pq"); self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>pq'+"</p>");
      if self.dangerMode:
        response = self._sendCommand(b'>pq\n')
        if response is not None:
          with self._stateLock: self.qSwitchStatus = 1
          print("response:", response)
          self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    self.updateAllStatusIndicators()

  def singlePulse(self):
    print(">oq"); self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>oq'+"</p>");
    if self.dangerMode:
      response = self._sendCommand(b'>oq\n')
      if response is not None:
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")

  # --- Jobs of several steps (the Arm External / Warmup / STOP buttons, and BLACS's buttons) ---
  # Each returns None when every step is done, or words naming the step that failed
  # ("arm failed at step 5 of 6 (shutter open, >r1): the laser did not answer >r1"). A job stops
  # at the first failed step and sends nothing after it. Every step needs an answer from the
  # laser; a mode step also needs the mode the laser reports back to be the one asked for.

  def _runJob(self, job, steps):
    n = len(steps)
    for i, (label, step) in enumerate(steps, 1):
      why = step() if self.serialConnected else "no serial connection to the laser"
      if why:
        words = "%s failed at step %d of %d (%s): %s" % (job, i, n, label, why)
        self.terminalOutputTextBrowser.append("<p style='color: red'>%s</p>" % words)
        self.updateAllStatusIndicators()
        return words
    self.updateAllStatusIndicators()
    return None

  def _commandStep(self, cmd, **state):
    """One step: send cmd; on an answer set the cached state (e.g. shutterStatus=1)."""
    response = self._sendCommand((cmd + '\n').encode('ascii'))
    if response is None:
      return "the laser did not answer %s" % cmd
    with self._stateLock:
      for name, value in state.items(): setattr(self, name, value)
    self.terminalOutputTextBrowser.append(cmd)
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    return None

  def _modeStep(self, set_mode):
    result = set_mode()
    return None if result.get("status") == "SUCCESS" else result.get("message", "refused")

  def _standbyStep(self):
    return self._commandStep('>s', activeStatus=0, shutterStatus=0, qSwitchStatus=0)

  def startLaser(self):
    """Arm for the external flashlamp trigger. None when armed, else the failed step."""
    words = self._runJob("arm", [
        ("standby, >s", self._standbyStep),
        ("lamp mode external, >lpm1", lambda: self._modeStep(self.setFlashLampExternal)),
        ("Q-switch mode internal, >qsm0", lambda: self._modeStep(self.setQSwitchInternal)),
        ("lamps on, >a", lambda: self._commandStep('>a', activeStatus=1)),
        ("shutter open, >r1", lambda: self._commandStep('>r1', shutterStatus=1)),
        ("Q-switch on, >pq", lambda: self._commandStep('>pq', qSwitchStatus=1)),
    ])
    if words is None:
      self.warmupActive = False
      self.terminalOutputTextBrowser.append(
          "<p style='color: blue'>Armed for external trigger (lamp external, QS internal, shutter open, QS armed).</p>")
    return words

  def stopLaser(self):
    """Standby (lamps off, shutter closed, Q-switch off). None when done, else the failed step.
    Always sent, whatever the GUI thinks the laser is doing (it may be out of step)."""
    self.warmupActive = False
    return self._runJob("stop", [("standby, >s", self._standbyStep)])

  def toggleTerminalInput(self):
    if self.terminalEnabled:
      self.terminalEnabled=False;
      self.stopLaser()
      self.update_fLampMode()
      self.update_qSwitchMode()
      self.update_fLampVoltage()
      self.update_fLampEnergy()
      self.updateTemp()
      self.updateFreq()
    else:
      self.terminalEnabled=True;
    self.terminalInputLabel.setEnabled(self.terminalEnabled); self.terminalInputLineEdit.setEnabled(self.terminalEnabled)
    self.qSwitchRadioButton_0.setEnabled(not(self.terminalEnabled)); self.qSwitchRadioButton_1.setEnabled(not(self.terminalEnabled)); self.qSwitchRadioButton_2.setEnabled(not(self.terminalEnabled))
    self.flashLampRadioButton_0.setEnabled(not(self.terminalEnabled)); self.flashLampRadioButton_1.setEnabled(not(self.terminalEnabled))
    frequencyBoolean = not(self.terminalEnabled) and not(self.flashLampMode)
    self.frequencyDoubleSpinBox.setEnabled(frequencyBoolean); self.FrequencyLabel.setEnabled(frequencyBoolean); self.frequencyConfirmationButton.setEnabled(frequencyBoolean)
    self.flashLampVoltageLabel.setEnabled(not(self.terminalEnabled))
    self.flashLampVoltageSpinBox.setEnabled(not(self.terminalEnabled))
    self.voltageConfirmationButton.setEnabled(not(self.terminalEnabled))
    #New controls
    self.lampToggleButton.setEnabled(not(self.terminalEnabled))
    self.shutterToggleButton.setEnabled(not(self.terminalEnabled))
    self.qSwitchToggleButton.setEnabled(not(self.terminalEnabled))
    self.singlePulseButton.setEnabled(not(self.terminalEnabled))
    self.warmupButton.setEnabled(not(self.terminalEnabled))
    self.startLasingButton.setEnabled(not(self.terminalEnabled))
    self.fullStopButton.setEnabled(not(self.terminalEnabled))
    self.keepWarmCheckBox.setEnabled(not(self.terminalEnabled))

  def fetchSerial(self):
    print(">sn"); self.terminalOutputTextBrowser.append("<p style='color: black'>"+'>sn'+"</p>");
    if self.serialConnected and self.dangerMode:
      response = self._sendCommand(b'>sn\n')
      if response is not None:
        print("response:", response)
        self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
        sn = response.strip(' \r\ns/number')
      else: sn=''
    else: sn=''
    self.serialNumber=sn

  def updateTerminalCommand(self,text):
    self.terminalLineCurrently = text

  def sendTerminalCommand(self):
    toWrite = '>'+self.terminalLineCurrently+'\n'
    print("sending to terminal:",toWrite) #TODO: finish this function
    self.terminalOutputTextBrowser.append("<p style='color: blue'>"+toWrite.strip('\n')+"</p>");
    response = self._sendCommand(toWrite)
    if response is not None:
      self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    self.terminalLineCurrently = ''
    self.terminalInputLineEdit.setText(self.terminalLineCurrently)

  def updateTemp(self):
    if not self.serialConnected:
      return
    self.terminalOutputTextBrowser.append('>cg')
    response = self._sendCommand(b'>cg\n')
    if response is None: return
    try:
      temp = float(response.strip('\r\ntemp.CG d'))
    except ValueError as e:
      self.terminalOutputTextBrowser.append(
          "<p style='color: orange'>Temperature parse error: %s</p>" % e)
      return
    with self._stateLock: self.lastTemperature = temp
    print("temperature = {T}C".format(T=temp))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    tiempo = time.strftime("%d %b %Y %H:%M:%S", time.localtime())
    print("time = {t}".format(t=tiempo))
    self.temperatureOutput.setText(str(temp)+" C")
    self.lastUpdateOutput.setText(str(tiempo))
    self._updateTemperatureStatusColor()

  def update_fLampVoltage(self):
    if not self.serialConnected: return
    self.terminalOutputTextBrowser.append('>v')
    response = self._sendCommand(b'>v\n')
    if response is None: return
    try:
      with self._stateLock: self.fLampVoltage=int(response.strip('\r\nvoltage V'))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>Voltage parse error</p>"); return
    print("voltage = {V}V".format(V=self.fLampVoltage))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    #self.flashLampVoltageHorizontalSlider.setValue(self.fLampVoltage)
    self.flashLampVoltageSpinBox.setValue(self.fLampVoltage)

    self.PowerEstimateValue.setText('%.2f'%np.interp(self.fLampVoltage,self.calibVolts,self.calibPower) + " W")

  def update_fLampEnergy(self):
    if not self.serialConnected: return
    self.terminalOutputTextBrowser.append('>ene')
    response = self._sendCommand(b'>ene\n')
    if response is None: return
    try:
      self.fLampEnergy=float(response.strip('\r\nenergy J'))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>Energy parse error</p>"); return
    print("energy = {E}J".format(E=self.fLampEnergy))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    #self.flashLampEnergyHorizontalSlider.setValue(int(10*self.fLampEnergy))
    self.flashLampEnergyValue.setText(str(self.fLampEnergy)+" J")

  def update_fLampMode(self):
    if not self.serialConnected: return
    self.terminalOutputTextBrowser.append('>lpm')
    response = self._sendCommand(b'>lpm\n')
    if response is None: return
    try:
      self.flashLampMode=int(response.strip('\r\nLP synch :  '))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>Lamp mode parse error</p>"); return
    print("self.flashLampMode = {f}".format(f=self.flashLampMode))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    if self.flashLampMode==0: self.flashLampRadioButton_0.setChecked(True)
    elif self.flashLampMode==1: self.flashLampRadioButton_1.setChecked(True)
    else:
      print("ERROR. self.flashLampMode makes no sense")
      self._sendCommand(b'>s\n')

  def update_qSwitchMode(self):
    if not self.serialConnected: return
    self.terminalOutputTextBrowser.append('>qsm')
    response = self._sendCommand(b'>qsm\n')
    if response is None: return
    try:
      self.qSwitchMode=int(response.strip('\r\nQS mode :  '))
    except ValueError:
      self.terminalOutputTextBrowser.append("<p style='color: orange'>QS mode parse error</p>"); return
    print("self.qSwitchMode = {q}".format(q=self.qSwitchMode))
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>");
    if self.qSwitchMode==0: self.qSwitchRadioButton_0.setChecked(True)
    elif self.qSwitchMode==1: self.qSwitchRadioButton_1.setChecked(True)
    elif self.qSwitchMode==2: self.qSwitchRadioButton_2.setChecked(True)
    else:
      print("ERROR. self.qSwitchMode makes no sense")
      self._sendCommand(b'>s\n')

  def updateAllStatusIndicators(self):
    #Lamp status
    if self.activeStatus:
      self.lampStatusLabel.setText("LAMPS: FIRING")
      self._setLabelColor(self.lampStatusLabel, bg="#90EE90", fg="black")
      self.lampToggleButton.setText("DEACTIVATE LAMPS")
    else:
      self.lampStatusLabel.setText("LAMPS: STANDBY")
      self._setLabelColor(self.lampStatusLabel, bg="#D3D3D3", fg="black")
      self.lampToggleButton.setText("ACTIVATE LAMPS")

    #Shutter status
    if self.shutterStatus:
      self.shutterStatusLabel.setText("SHUTTER: OPEN")
      self._setLabelColor(self.shutterStatusLabel, bg="#FFA500", fg="black")
      self.shutterToggleButton.setText("CLOSE SHUTTER")
    else:
      self.shutterStatusLabel.setText("SHUTTER: CLOSED")
      self._setLabelColor(self.shutterStatusLabel, bg="#D3D3D3", fg="black")
      self.shutterToggleButton.setText("OPEN SHUTTER")

    #Q-Switch status
    if self.qSwitchStatus:
      self.qSwitchStatusLabel.setText("Q-SWITCH: ARMED")
      self._setLabelColor(self.qSwitchStatusLabel, bg="#FF6347", fg="white")
      self.qSwitchToggleButton.setText("DISARM Q-SWITCH")
    else:
      self.qSwitchStatusLabel.setText("Q-SWITCH: DISABLED")
      self._setLabelColor(self.qSwitchStatusLabel, bg="#D3D3D3", fg="black")
      self.qSwitchToggleButton.setText("ARM Q-SWITCH")

    #Overall status
    if not self.serialConnected:
      self.overallStatusLabel.setText("OVERALL: DISCONNECTED")
      self._setLabelColor(self.overallStatusLabel, bg="#8B0000", fg="white")
    elif self.activeStatus and self.shutterStatus and self.qSwitchStatus:
      self.overallStatusLabel.setText("OVERALL: LASING")
      self._setLabelColor(self.overallStatusLabel, bg="#FF0000", fg="white")
    elif self.activeStatus and not self.shutterStatus:
      self.overallStatusLabel.setText("OVERALL: WARMING UP")
      self._setLabelColor(self.overallStatusLabel, bg="#FFD700", fg="black")
    elif self.activeStatus:
      self.overallStatusLabel.setText("OVERALL: LAMPS ACTIVE")
      self._setLabelColor(self.overallStatusLabel, bg="#90EE90", fg="black")
    else:
      self.overallStatusLabel.setText("OVERALL: STANDBY")
      self._setLabelColor(self.overallStatusLabel, bg="#D3D3D3", fg="black")

    #Temperature status
    self._updateTemperatureStatusColor()

    #Button enable/disable logic
    self.shutterToggleButton.setEnabled(self.activeStatus)
    self.qSwitchToggleButton.setEnabled(self.activeStatus and self.shutterStatus)
    self.singlePulseButton.setEnabled(
        self.activeStatus and self.shutterStatus
        and not self.qSwitchStatus and (self.qSwitchMode == 0))

    #Mode radio buttons only changeable in standby
    for rb in [self.qSwitchRadioButton_0, self.qSwitchRadioButton_1,
               self.qSwitchRadioButton_2, self.flashLampRadioButton_0,
               self.flashLampRadioButton_1]:
        rb.setEnabled(not self.activeStatus)

    tiempo = time.strftime("%d %b %Y %H:%M:%S", time.localtime())
    self.lastUpdateOutput.setText(str(tiempo))

  def _setLabelColor(self, label, bg, fg):
    label.setStyleSheet(
        "background-color: %s; color: %s; padding: 4px; border: 1px solid gray;" % (bg, fg))

  def _updateTemperatureStatusColor(self):
    temp = self.lastTemperature
    if temp < self.TEMP_COLD:
      self.temperatureStatusLabel.setText("TEMP: %.1f C (COLD)" % temp)
      self._setLabelColor(self.temperatureStatusLabel, bg="#87CEEB", fg="black")
    elif temp < self.TEMP_OPERATING:
      self.temperatureStatusLabel.setText("TEMP: %.1f C (WARMING)" % temp)
      self._setLabelColor(self.temperatureStatusLabel, bg="#FFD700", fg="black")
    else:
      self.temperatureStatusLabel.setText("TEMP: %.1f C (OK)" % temp)
      self._setLabelColor(self.temperatureStatusLabel, bg="#90EE90", fg="black")

  def startWarmup(self):
    """Warm-up: lamps firing on the internal trigger, shutter closed, Q-switch off. None when
    done, else the failed step. startWarmup does not set the voltage."""
    words = self._runJob("warm_up", [
        ("standby, >s", self._standbyStep),
        ("lamp mode internal, >lpm0", lambda: self._modeStep(self.setFlashLampInternal)),
        ("lamps on, >a", lambda: self._commandStep('>a', activeStatus=1)),
    ])
    if words is None:
      self.warmupActive = True
      self.updateTemp()
      self.updateAllStatusIndicators()
      self.terminalOutputTextBrowser.append(
          "<p style='color: blue'>Warmup started (internal trigger). Lamps firing, shutter closed.</p>")
    return words

  def toggleKeepWarm(self, checked):
    """Toggle the auto-warmup-if-cold behaviour.

    Temperature polling and display are always on while serial is connected;
    this toggle only governs whether the GUI auto-enters warmup when the
    coolant temperature drops below TEMP_COLD (37°C).

    Uses hysteresis to prevent oscillation:
    - Triggers warmup when temp drops below TEMP_COLD (37°C)
    - Resets trigger when temp rises to TEMP_OPERATING (39°C)
    Works standalone (no BLACS needed) and can be synced from BLACS via ZMQ.
    """
    self.keepWarmActive = checked
    if checked:
      self._warmupTriggered = False
      self.terminalOutputTextBrowser.append(
          "<p style='color: blue'>Auto-warmup enabled — will enter warmup if temp &lt; %.0f°C.</p>"
          % self.TEMP_COLD)
      # Check temperature immediately
      self._evaluateKeepWarm()
    else:
      self._warmupTriggered = False
      self.warmupActive = False
      self.terminalOutputTextBrowser.append(
          "<p style='color: blue'>Auto-warmup disabled.</p>")

  def pollTemperature(self):
    if not self.serialConnected:
      return
    self.updateTemp()
    # Latched energy readback after voltage change
    if self._energyReadbackPending:
      self._energyReadbackPending = False
      self.update_fLampEnergy()
    if self.warmupActive and self.lastTemperature >= self.TEMP_OPERATING:
      self.terminalOutputTextBrowser.append(
          "<p style='color: green'>Temperature %.1fC >= %.1fC. Laser is warm enough to lase.</p>"
          % (self.lastTemperature, self.TEMP_OPERATING))
      self.warmupActive = False
    # Auto Keep Warm: check if we need to enter warmup
    self._evaluateKeepWarm()
    self.updateAllStatusIndicators()

  def _evaluateKeepWarm(self):
    """Check if Auto Keep Warm should trigger warmup.

    Uses hysteresis to prevent oscillation:
    - Triggers warmup when temp drops below TEMP_COLD (37°C)
    - Resets trigger when temp rises to TEMP_OPERATING (39°C)
    The one keep-warm rule (BLACS has none; it only ticks this box through the BLACS link).
    An armed laser (lamps, shutter, Q-switch on) is left alone: a warm-up would drop it out of
    its armed state in the middle of a run. It is warmed once it is no longer armed.
    """
    if not self.keepWarmActive:
      return
    if not self.serialConnected:
      return
    temp = self.lastTemperature
    armed = self.activeStatus and self.shutterStatus and self.qSwitchStatus
    if temp < self.TEMP_COLD and not self._warmupTriggered and not armed:
      self.terminalOutputTextBrowser.append(
          "<p style='color: blue'>Auto-warmup: cold (%.1f°C), entering warmup.</p>" % temp)
      self._warmupTriggered = self.startWarmup() is None    # a failed warm-up is tried again
    elif temp >= self.TEMP_OPERATING:
      if self._warmupTriggered:
        self._warmupTriggered = False

  # --- What BLACS reads and does (HugeSkyController's BLACS link, RaX RemoteControl v2) ---
  # These handlers refuse only what the laser's order needs (shutter needs the lamps, Q-switch
  # needs lamps and shutter, modes need standby) and a read-back that differs from the ask.

  def remoteSnapshot(self):
    """Every value BLACS can read of this laser, from the GUI's cached state. Thread-safe read
    (the link's threads)."""
    with self._stateLock:
      return {"voltage": self.fLampVoltage, "lamps": int(self.activeStatus),
              "shutter": int(self.shutterStatus), "qswitch": int(self.qSwitchStatus),
              "lamp_mode": int(self.flashLampMode), "qswitch_mode": int(self.qSwitchMode),
              "temperature": float(self.lastTemperature), "keep_warm": int(bool(self.keepWarmActive))}

  def executeRemoteCommand(self, command, value, future):
    """Called from the link's request thread: runs `command` on the Qt main thread (where the
    serial port and the widgets live). `future` gets the result (what was really set; None for
    a button), or the exception: RemoteRefused(code, words) when it could not be done."""
    self._remoteCommandRequested.emit(command, value, future)

  @pyqtSlot(str, object, object)
  def _handleRemoteCommand(self, command, value, future):
    if future.done():          # the server stopped waiting (time limit): do not act late
      return
    self.terminalOutputTextBrowser.append("<p style='color: blue'>[BLACS] %s %s</p>"
                                          % (command, "" if value is None else "= %s" % value))
    try:
      result = self._remoteDo(command, value)
    except Exception as e:     # noqa: BLE001  (RemoteRefused, or a bug: both go to BLACS)
      if not future.done(): future.set_exception(e)
      return
    if not future.done(): future.set_result(result)

  def _remoteDo(self, command, value):
    jobs = {'arm': self.startLaser, 'warm_up': self.startWarmup, 'stop': self.stopLaser}
    if command in jobs:
      if not int(round(float(value))):    # a job set to 0 (a sequence's "not now"): nothing to do
        return None
      words = jobs[command]()
      if words: raise RemoteRefused("failed_at_step", words)
      return None
    sets = {'voltage': self._remoteSetVoltage, 'lamps': self._remoteSetLamps,
            'shutter': self._remoteSetShutter, 'qswitch': self._remoteSetQSwitch,
            'lamp_mode': self._remoteSetLampMode, 'qswitch_mode': self._remoteSetQSwitchMode,
            'keep_warm': self._remoteSetKeepWarm}
    if command not in sets:
      raise RemoteRefused("unknown_command", "this GUI has no %r" % command)
    return sets[command](int(round(float(value))))

  def _remoteCommand(self, cmd, **state):
    """Send cmd; RemoteRefused when the laser does not answer. Updates the indicators."""
    why = self._commandStep(cmd, **state)
    self.updateAllStatusIndicators()
    if why: raise RemoteRefused("no_answer", why)

  def _remoteSetVoltage(self, voltage_V):
    cmd = ">vmo%04d" % voltage_V
    response = self._sendCommand((cmd + '\n').encode('ascii'))
    if response is None:
      raise RemoteRefused("no_answer", "the laser did not answer %s" % cmd)
    try:
      actual = int(response.strip('\r\nvoltage m V'))
    except ValueError:
      raise RemoteRefused("bad_answer", "could not read the voltage in the answer to %s: %r" % (cmd, response))
    with self._stateLock: self.fLampVoltage = actual
    self.terminalOutputTextBrowser.append("<p style='color: green'>"+response.strip('\r\n')+"</p>")
    self.flashLampVoltageSpinBox.setValue(actual)
    self.PowerEstimateValue.setText('%.2f'%np.interp(actual,self.calibVolts,self.calibPower) + " W")
    self._energyReadbackPending = True  # deferred to next temp poll
    if actual != voltage_V:
      raise RemoteRefused("did_not_take_effect", "asked %d V, the laser reports %d V" % (voltage_V, actual))
    return actual

  def _remoteSetLamps(self, state):
    if state: self._remoteCommand('>a', activeStatus=1)
    else: self._remoteCommand('>s', activeStatus=0, shutterStatus=0, qSwitchStatus=0)
    return state

  def _remoteSetShutter(self, state):
    if state and not self.activeStatus:
      raise RemoteRefused("lamps_not_active", "the shutter opens only while the lamps fire")
    self._remoteCommand('>r1' if state else '>r0', shutterStatus=state)
    return state

  def _remoteSetQSwitch(self, state):
    if state and (not self.activeStatus or not self.shutterStatus):
      raise RemoteRefused("qswitch_requires_lamps_and_shutter",
                          "the Q-switch arms only with the lamps firing and the shutter open")
    self._remoteCommand('>pq' if state else '>sq', qSwitchStatus=state)
    return state

  def _remoteSetMode(self, what, set_mode):
    if self.activeStatus:
      raise RemoteRefused("%s_requires_standby" % what, "%s changes only in standby (lamps off)" % what)
    result = set_mode()
    self.updateAllStatusIndicators()
    if result.get("status") != "SUCCESS":
      raise RemoteRefused(result.get("code", "refused"), result.get("message", ""))

  def _remoteSetLampMode(self, mode):
    self._remoteSetMode("lamp_mode", self.setFlashLampExternal if mode else self.setFlashLampInternal)
    return mode

  def _remoteSetQSwitchMode(self, mode):
    self._remoteSetMode("qswitch_mode", lambda: self._setQSwitchMode(mode))
    return mode

  def _remoteSetKeepWarm(self, on):
    if bool(on) != self.keepWarmActive:
      self.keepWarmCheckBox.setChecked(bool(on))     # toggled -> toggleKeepWarm (the GUI's rule)
    return int(self.keepWarmActive)

  def safeExit(self):
    self.tempPollTimer.stop()
    self._reconnectTimer.stop()
    print(">s")
    if self.serialConnected:
      try:
        self.ser.flush(); self.ser.write(b'>s\n'); response = self.ser.read(140).decode('utf-8'); print("response:", response)
        self.ser.close()
      except Exception:
        pass

if __name__ == "__main__":
  app = QtWidgets.QApplication(sys.argv)
  window = SingleLaserController()
  app.aboutToQuit.connect(window.safeExit)
  window.show()
  sys.exit(app.exec_())
