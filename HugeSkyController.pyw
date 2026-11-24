import os
import sys
from PyQt5.QtWidgets import (QMainWindow, QApplication, QPushButton,
 QWidget, QAction, QTabWidget,QVBoxLayout, QGridLayout, QTabBar, QLineEdit, QTextBrowser)
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject
from BigSkyControllerAmbitious import SingleLaserController, RemoteRefused
import serial.tools.list_ports
import ctypes
import pickle
import threading
import concurrent.futures

# The BLACS link speaks RaX's RemoteControl protocol v2: REQ/REP for requests, PUB/SUB for the
# live values. zmq_v2 is RaX's file, in the beambox-daq package bbx_zmq. Install once into this
# GUI's Python:  python -m pip install -e <beambox-daq> --no-deps
from bbx_zmq.zmq_v2 import (RemoteControlServerBase, handler, encode_reply, PROTOCOL_VERSION,
                            ZmqRepTransport)


# --- BLACS link (adapted from upstream HugeSkyController.pyw: _BigSkyV2Server, BigSkyZmqServer) ---

REP_PORT = 55540               # requests from BLACS (upstream DEFAULT_REP_PORT)
PUB_PORT = 55541               # monitors and heartbeat (upstream DEFAULT_PUB_PORT)
_REMOTE_CMD_TIMEOUT_S = 10.0   # longest wait for the laser tab to finish a set or a job (upstream)

# Name on the wire (after "YAG_1_") -> this GUI's command (SingleLaserController._remoteDo).
_COMMANDS = {'voltage': 'voltage', 'lamps': 'lamps', 'shutter': 'shutter', 'qswitch': 'qswitch',
             'lamp_mode': 'lamp_mode', 'qswitch_mode': 'qswitch_mode', 'keep_warm': 'keep_warm',
             'start_lasing': 'arm', 'warmup': 'warm_up', 'stop': 'stop'}


def _error(status, code, words, request_id, retryable=False):
  return encode_reply(status=status, request_id=request_id,
                      error={"code": code, "message": words, "retryable": retryable})


class _BigSkyV2Server(RemoteControlServerBase):
  """Answers BLACS's requests (HELLO, PING, PROGRAM_VALUE, CHECK_VALUE) for the open lasers.
  Names are "<laser>_<name>" (YAG_1_voltage); a read also takes "<laser>_<name>_monitor"."""
  CAPABILITIES = frozenset({"monitors", "heartbeat"})

  def __init__(self, link, transport):
    super().__init__("BigSkyLasers", transport)
    self._link = link

  def _handle_hello(self, request_id):
    return encode_reply(status="SUCCESS", request_id=request_id, extra={
        "protocol_version": PROTOCOL_VERSION, "server": self._server_name,
        "capabilities": sorted(self.CAPABILITIES),
        "connections": ["%s_*" % laser for laser in sorted(self._link.lasers)]})

  def _find(self, connection, request_id):
    """(controller, name, is_monitor), or the reply that refuses the request."""
    name = connection[:-len("_monitor")] if connection.endswith("_monitor") else connection
    for laser in sorted(self._link.lasers, key=len, reverse=True):     # YAG_10 before YAG_1
      if name.startswith(laser + "_"):
        ctrl = self._link.lasers[laser]
        if not ctrl.isConnected():
          return _error("ERROR", "laser_disconnected", "%s: the laser's serial link is down" % laser,
                        request_id, retryable=True)
        return ctrl, name[len(laser) + 1:], name != connection
    return _error("UNKNOWN_CONNECTION", "unknown_connection", "no open laser for %r" % connection,
                  request_id)

  @handler("PROGRAM_VALUE")
  def _handle_program(self, connection, value, args, request_id):
    """Runs the set or job on the laser tab's Qt thread and answers what really happened."""
    found = self._find(connection, request_id)
    if isinstance(found, bytes):
      return found
    ctrl, name, is_monitor = found
    if is_monitor:
      return _error("ERROR", "cannot_program_monitor", "%r is a read-out" % connection, request_id)
    if name not in _COMMANDS:
      return _error("ERROR", "unknown_writable_param", "%r cannot be set" % connection, request_id)
    self._link.log("BLACS: %s = %s" % (connection, value))
    future = concurrent.futures.Future()
    ctrl.executeRemoteCommand(_COMMANDS[name], value, future)
    try:
      result = future.result(timeout=_REMOTE_CMD_TIMEOUT_S)
    except concurrent.futures.TimeoutError:
      future.cancel()      # not started yet: it never runs
      return _error("TIMEOUT", "command_timeout",
                    "%s: not finished within %.0f s (if it had started, it may still finish; look "
                    "at the GUI)" % (connection, _REMOTE_CMD_TIMEOUT_S), request_id, retryable=True)
    except RemoteRefused as e:
      self._link.log("BLACS: %s refused: [%s] %s" % (connection, e.code, e.words))
      return _error("REJECTED", e.code, e.words, request_id)
    return encode_reply(status="SUCCESS", request_id=request_id, value=result)

  @handler("CHECK_VALUE")
  def _handle_check(self, connection, value, args, request_id):
    found = self._find(connection, request_id)
    if isinstance(found, bytes):
      return found
    ctrl, name, _ = found
    snap = ctrl.remoteSnapshot()
    if name not in snap:
      return _error("ERROR", "unknown_monitor_param", "%r cannot be read" % connection, request_id)
    return encode_reply(status="SUCCESS", request_id=request_id, value=snap[name])


class BigSkyZmqServer(QObject):
  """The BLACS link: REP on tcp://*:55540 and PUB on tcp://*:55541, each on its own thread, so
  the heartbeat keeps coming while a request waits up to 10 s for the laser."""
  logMessage = pyqtSignal(str)

  def __init__(self, parent=None):
    super().__init__(parent)
    self.lasers = {}       # {"YAG_1": SingleLaserController}
    self._turn = 0
    self._stop_event = threading.Event()
    self._threads = []

  def addLaser(self, name, controller):
    self.lasers[name] = controller

  def removeLaser(self, name):
    self.lasers.pop(name, None)

  def requestServer(self, transport):
    return _BigSkyV2Server(self, transport)

  def publish_once(self, publish):
    """One turn of the PUB stream (every 250 ms): "<laser>_<name>_monitor <value>" for every
    value of every open laser whose serial link is up; "heartbeat" every fourth turn."""
    self._turn += 1
    for laser, ctrl in list(self.lasers.items()):
      if ctrl.isConnected():     # a laser whose serial link is down is not published (upstream)
        for name, value in ctrl.remoteSnapshot().items():
          publish("%s_%s_monitor %s" % (laser, name, value))
    if self._turn % 4 == 0:
      publish("heartbeat")

  def start(self):
    if any(t.is_alive() for t in self._threads):
      self.log("BLACS link: already running."); return
    self._stop_event.clear()
    self._threads = [threading.Thread(target=f, daemon=True, name=n) for f, n in
                     ((self._requestLoop, "BigSkyREP"), (self._publishLoop, "BigSkyPUB"))]
    for t in self._threads:
      t.start()

  def stop(self):
    self._stop_event.set()
    for t in self._threads:
      t.join(timeout=3.0)
    self._threads = []
    self.log("BLACS link: stopped.")

  def log(self, msg):
    print(msg)
    self.logMessage.emit(msg)

  def _requestLoop(self):
    try:
      transport = ZmqRepTransport("tcp://*:%d" % REP_PORT, recv_timeout_ms=250)
    except Exception as e:     # noqa: BLE001  (port taken, no pyzmq): the GUI runs without BLACS
      self.log("BLACS link: no requests, port %d: %s" % (REP_PORT, e)); return
    server = self.requestServer(transport)
    self.log("BLACS link: RemoteControl v2, REP port %d, PUB port %d." % (REP_PORT, PUB_PORT))
    failures = 0
    while not self._stop_event.is_set():
      try:
        server.serve_once(timeout_ms=250)
        failures = 0
      except Exception as e:   # noqa: BLE001  (handler errors are answered; this is the socket)
        failures += 1
        self.log("BLACS link: socket error (%d/5): %s" % (failures, e))
        if failures >= 5:
          self.log("BLACS link: too many socket errors; requests stopped."); break
    transport.close()

  def _publishLoop(self):
    try:
      import zmq
      sock = zmq.Context.instance().socket(zmq.PUB)
      sock.setsockopt(zmq.LINGER, 0)
      sock.bind("tcp://*:%d" % PUB_PORT)
    except Exception as e:     # noqa: BLE001  (port taken, no pyzmq): the GUI runs without the stream
      self.log("BLACS link: no PUB stream, port %d: %s" % (PUB_PORT, e)); return
    while not self._stop_event.wait(0.25):
      try:
        self.publish_once(sock.send_string)
      except Exception as e:   # noqa: BLE001  (a tab closing mid-turn): the next turn goes on
        self.log("BLACS link: publish error: %s" % e)
    sock.close()


# --- Connection Name Mapping ---
# Maps laser serial numbers to BLACS connection names.
# COM ports that hold other instruments: the port scan never opens them, so ">sn" at 9600 baud
# never reaches them and the GUI can start before or after BLACS (which holds them) in any order.
# BeamBox: COM10 the Quantum Composer, COM11 the PrawnBlaster.
NOT_LASERS = {"COM10", "COM11"}

# Edit this dict to match your physical setup.
# Serial numbers are strings as returned by the >sn command.
LASER_SN_TO_CONNECTION = {
  '151': 'YAG_1',
  '213': 'YAG_2',
  # SN 082 (label 'sdfhjkasehkfs' in laserNames.pkl — test/typo entry) and
  # SN 261 (label 'COBRA PUMP' — a different physical laser, not a BLACS-driven YAG)
  # are intentionally omitted. If those COM ports are launched, the
  # _laserLaunchOrder counter fallback applies (identical to pre-2026-05-19 behavior).
  # Populated 2026-05-19 per refactor plan; previously this dict was empty
  # placeholder, which made YAG_N assignment deterministic-by-accident only.
}


class BigSkyHub(QMainWindow):
  def __init__(self, noLaser=False):
    """noLaser: the no-laser mode (`--no-laser`): no COM port is scanned or opened; one dummy
    laser, YAG_1, is open (no serial port): BLACS gets HELLO and the heartbeat; every set or
    read of YAG_1 answers laser_disconnected, and no YAG_1 value is published."""
    super().__init__()
    self.setWindowTitle('Big Sky Controller Hub' + (' (no laser)' if noLaser else ''))
    self.setWindowIcon(QIcon('BigSkyDesktopIcon.ico'))
    self.left = 0; self.width = 900
    self.top = 0 ; self.height = 800
    self.setGeometry(self.left, self.top, self.width, self.height)

    # BLACS link; lasers are added to it as their tabs open
    self.zmqServer = BigSkyZmqServer(parent=self)
    self.zmqServer.logMessage.connect(self._onZmqLog)
    self.table_widget = MyTableWidget(self, noLaser=noLaser)
    self.setCentralWidget(self.table_widget)
    self.zmqServer.start()

    self.show()

  def _onZmqLog(self, msg):
    """Append BLACS link messages to the home tab's text browser."""
    self.table_widget.homeTab.text.append("<p style='color: purple'>%s</p>" % msg)

  def closeEvent(self, event):
    """Override close to shut down the BLACS link before exiting."""
    self.zmqServer.stop()
    self.table_widget.safeExit()
    event.accept()

class HomeTab(QWidget):
  def __init__(self,parent,scan=True):
    super().__init__()
    try:
        with open('laserNames.pkl','rb') as file: self.laserNames=pickle.load(file); file.close()
    except: self.laserNames={}
    self.layout = QGridLayout(self)
    self.buttons=[]
    self.devices=[]
    self.serialNumbers=[]
    self.labelLineEdits=[]
    self.processes={}
    self._parentWidget = parent  # store ref for connecting new buttons

    # Initial scan (it opens every COM port and writes >sn: not in the no-laser mode)
    if scan:
      self._scanPorts()

    # Bottom row: text browser, refresh button, save button
    bottomRow = len(self.buttons)
    self.text = QTextBrowser()
    self.layout.addWidget(self.text, bottomRow, 0)
    btnLayout = QVBoxLayout()
    self.refreshButton = QPushButton('Refresh Connections')
    self.refreshButton.setStyleSheet("background-color: #E3F2FD; font-weight: bold;")
    self.refreshButton.pressed.connect(self.refreshConnections)
    self.refreshButton.setEnabled(scan)     # the refresh scans (opens) the COM ports
    btnLayout.addWidget(self.refreshButton)
    self.saveButton = QPushButton('Save Labels')
    self.saveButton.pressed.connect(self.saveLabels)
    btnLayout.addWidget(self.saveButton)
    btnContainer = QWidget()
    btnContainer.setLayout(btnLayout)
    self.layout.addWidget(btnContainer, bottomRow, 1)

  def _scanPorts(self):
    """Scan COM ports for BigSky lasers. Adds new ones to the button list."""
    possibleDevices = [comport.device for comport in serial.tools.list_ports.comports()]
    print(possibleDevices)
    for dev in possibleDevices:
      if dev in self.devices or dev.upper() in NOT_LASERS:
        continue  # already known, or another instrument (never sent >sn)
      try:
        print('trying com port %s' % dev)
        ser = serial.Serial(dev, 9600, timeout=1)
      except:
        print("nope not this one")
        continue
      try:
        ser.flush(); ser.write(b'>sn\n')
        response = ser.read(140).decode('utf-8'); print("response:", response)
        if 'number' in response:
          print("yeah this one."); ser.close()
          sn = response.strip('s// number\r\n')
          self.serialNumbers += [sn]
          self.devices += [dev]
          btn = QPushButton('launch %s ; SN %s' % (dev, sn))
          self.buttons += [btn]
          if sn in self.laserNames.keys():
            self.labelLineEdits += [QLineEdit(self.laserNames[sn])]
          else:
            self.labelLineEdits += [QLineEdit('')]
          self.layout.addWidget(self.buttons[-1], len(self.buttons)-1, 0)
          self.layout.addWidget(self.labelLineEdits[-1], len(self.buttons)-1, 1)
        else:
          ser.close()
      except Exception as e:
        print("scan error for %s: %s" % (dev, e))
        try: ser.close()
        except: pass

  def refreshConnections(self):
    """Re-scan COM ports for newly powered-on BigSky lasers."""
    prevCount = len(self.buttons)
    self._scanPorts()
    newCount = len(self.buttons)
    found = newCount - prevCount

    # Connect new buttons to createTab (via parent MyTableWidget)
    if hasattr(self._parentWidget, 'parent') and hasattr(self._parentWidget, 'tabs'):
      tableWidget = self._parentWidget  # MyTableWidget
      for i in range(prevCount, newCount):
        self.buttons[i].pressed.connect(lambda i=i: tableWidget.createTab(i))

    # Move bottom row widgets down to make room
    bottomRow = len(self.buttons)
    self.layout.addWidget(self.text, bottomRow, 0)
    btnContainer = self.layout.itemAtPosition(prevCount if prevCount > 0 else 0, 1)
    if btnContainer:
      self.layout.addWidget(btnContainer.widget(), bottomRow, 1)

    if found > 0:
      self.text.append("<p style='color: green'>Found %d new laser(s).</p>" % found)
    else:
      self.text.append("<p style='color: gray'>No new lasers found. Ensure lasers are powered on and connected via USB.</p>")
  def saveLabels(self):
    for i in range(len(self.buttons)):
      self.laserNames[self.serialNumbers[i]]=self.labelLineEdits[i].text()
    with open('laserNames.pkl','wb') as file: pickle.dump(self.laserNames, file); file.close()
    self.text.append('laser names saved to file.')

class MyTableWidget(QWidget):
  def __init__(self, parent, noLaser=False):
    super(QWidget, self).__init__(parent)
    self.parent = parent
    self.layout = QVBoxLayout(self)
    # Initialize tab screen
    self.tabs = QTabWidget()
    #self.tabs.resize(width,height)
    self.homeTab=HomeTab(self, scan=not noLaser)
    print("test: ", len(self.homeTab.buttons))

    # Track how many lasers have been launched (for fallback tab-order connection naming)
    self._laserLaunchOrder = 0

    for i in range(len(self.homeTab.buttons)):
      self.homeTab.buttons[i].pressed.connect(lambda i=i: self.createTab(i))

    self.tabs.addTab(self.homeTab,"Home")
    self.tabs.setTabsClosable(True)
    self.tabs.tabCloseRequested.connect(self.closeTab)
    self.tabs.tabBar().setTabButton(0, QTabBar.RightSide, None) #removes close button from homeTab
    # Add tabs to widget
    self.layout.addWidget(self.tabs)
    if noLaser:   # one dummy laser (cPort=-1: SingleLaserController opens no serial port)
      ctrl = SingleLaserController(cPort=-1, lString="YAG_1 (no laser)")
      self.tabs.addTab(ctrl, "YAG_1 (no laser)")
      self._registerLaser("YAG_1", ctrl, "YAG_1 (no laser)")

  def _assignConnectionName(self, serialNumber):
    """Determine the BLACS connection name for a laser.
    First checks LASER_SN_TO_CONNECTION dict (keyed by serial number).
    Falls back to tab-order assignment: YAG_1, YAG_2, YAG_3, ..."""
    sn = str(serialNumber)
    if sn in LASER_SN_TO_CONNECTION:
      return LASER_SN_TO_CONNECTION[sn]
    self._laserLaunchOrder += 1
    return "YAG_%d" % self._laserLaunchOrder

  def createTab(self, i):
    com=self.homeTab.devices[i]; labelString=self.homeTab.labelLineEdits[i].text()
    if labelString=='': labelString='test'+str(self.tabs.count())
    self.homeTab.text.append("Creating Gui for %s, with label \'%s\'"%(com,labelString))
    ctrl = SingleLaserController(cPort=com, lString=labelString)
    self.tabs.addTab(ctrl, labelString)
    self.homeTab.buttons[i].setEnabled(False)

    sn = self.homeTab.serialNumbers[i] if i < len(self.homeTab.serialNumbers) else str(i)
    self._registerLaser(self._assignConnectionName(sn), ctrl, labelString)

  def _registerLaser(self, connName, ctrl, labelString):
    """Open the laser to BLACS under connName (YAG_1, ...)."""
    ctrl._zmqConnectionName = connName  # stash on controller for unregister lookup
    ctrl._tabLabel = labelString  # store original label for disconnect/reconnect styling
    self.parent.zmqServer.addLaser(connName, ctrl)
    self.homeTab.text.append("BLACS link: laser open as '%s'" % connName)

    # Connect disconnect/reconnect signal to tab styling
    ctrl.connectionStatusChanged.connect(
        lambda connected, cn=connName: self._onConnectionChanged(cn, connected))

  def _onConnectionChanged(self, connName, connected):
    """Update tab text when a laser connects/disconnects."""
    for idx in range(1, self.tabs.count()):  # skip Home tab at 0
      widget = self.tabs.widget(idx)
      cn = getattr(widget, '_zmqConnectionName', None)
      if cn == connName:
        label = getattr(widget, '_tabLabel', connName)
        if connected:
          self.tabs.setTabText(idx, label)
          self.homeTab.text.append(
              "<p style='color: green'>%s reconnected</p>" % connName)
        else:
          self.tabs.setTabText(idx, "%s (DISCONNECTED)" % label)
          self.homeTab.text.append(
              "<p style='color: red'>%s serial disconnected</p>" % connName)
        break

  def closeTab(self,i):
    widget = self.tabs.widget(i)
    comport = widget.comPort
    label = widget.labelString
    self.homeTab.text.append("Closing tab %d aka %s"%(i,label))

    # Close it to BLACS
    connName = getattr(widget, '_zmqConnectionName', None)
    if connName:
      self.parent.zmqServer.removeLaser(connName)

    widget.safeExit()
    self.tabs.removeTab(i)
    for j in range(len(self.homeTab.buttons)): #I don't have a good way of identifying which tab number corresponds to which laser...
      if self.homeTab.devices[j]==comport:
        self.homeTab.buttons[j].setEnabled(True); break

  def safeExit(self):
    for i in range(self.tabs.count() - 1, 0, -1):
      print('safely closing tab %d' % i)
      self.closeTab(i)

if __name__ == '__main__':
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(u'BigSkyControllerHub')
    app = QApplication(sys.argv)
    window = BigSkyHub(noLaser='--no-laser' in sys.argv[1:])
    app.aboutToQuit.connect(lambda: (window.zmqServer.stop(), window.table_widget.safeExit()))
    sys.exit(app.exec_())
