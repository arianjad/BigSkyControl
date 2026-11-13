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

# The menu link to BLACS comes from the beambox-daq package bbx_zmq (BBX-189).
# Install once into this GUI's Python:  python -m pip install -e <beambox-daq> --no-deps
# (or put <beambox-daq>\src first on PYTHONPATH). The Big Sky menu, the same file BLACS loads:
# bbx_zmq/menus/bigsky.py (names, ranges, buttons, time limits, port).
from bbx_zmq import MenuServer, Refused, ZmqRepTransport
from bbx_zmq.menus.bigsky import MENU


# --- BLACS link: the menu server and its handlers ---


class BigSkyMenuLink:
  """The handlers of this GUI's menu server, on the lasers this hub has open.

  BLACS reads every value in one snapshot, sets one value, or presses one button (arm, warm_up,
  stop); names are "<laser>_<name>" (YAG_1_voltage). The menu server (bbx_zmq MenuServer)
  refuses, before these handlers run, a name that is not on the Big Sky menu, a laser that is
  not open and a value out of range. A set or press then runs on the laser's own Qt thread; this
  waits for it up to the menu's time limit and answers what really happened, or refuses with
  the laser's code and words (an arm that stopped at a step: "failed_at_step").
  """

  def __init__(self, log=print):
    self._lasers = {}     # {"YAG_1": SingleLaserController}
    self._log = log

  def addLaser(self, name, controller):
    self._lasers[name] = controller

  def removeLaser(self, name):
    self._lasers.pop(name, None)

  def units(self):
    return sorted(self._lasers)

  def readAll(self):
    snap = {}
    for name, ctrl in list(self._lasers.items()):
      snap.update({"%s_%s" % (name, k): v for k, v in ctrl.remoteSnapshot().items()})
    return snap

  def setValue(self, wire_name, value):
    laser, item = self._split(wire_name)
    limit_s = MENU.value(wire_name, [laser]).set_s
    return self._onLaser(laser, item, value, limit_s, "%s = %s" % (wire_name, value))

  def pressButton(self, wire_name):
    laser, item = self._split(wire_name)
    return self._onLaser(laser, item, None, MENU.button(wire_name, [laser]).press_s, wire_name)

  def menuServer(self, transport):
    return MenuServer(MENU, transport, read_all=self.readAll, set_value=self.setValue,
                      press_button=self.pressButton, units=self.units)

  def _split(self, wire_name):
    for laser in sorted(self._lasers, key=len, reverse=True):   # YAG_10 before YAG_1
      if wire_name.startswith(laser + "_"):
        return laser, wire_name[len(laser) + 1:]
    raise Refused("not_on_menu", "no open laser for %r" % wire_name)

  def _onLaser(self, laser, command, value, limit_s, what):
    ctrl = self._lasers.get(laser)
    if ctrl is None:
      raise Refused("not_on_menu", "%s is not open in this GUI" % laser)
    if not ctrl.isConnected():
      raise Refused("laser_disconnected", "%s: the laser's serial link is down" % laser)
    self._log("BLACS: %s" % what)
    future = concurrent.futures.Future()
    ctrl.executeRemoteCommand(command, value, future)
    try:
      result = future.result(timeout=limit_s)
    except concurrent.futures.TimeoutError:
      future.cancel()      # not started yet: it never runs
      raise Refused("gui_busy", "%s: not finished within %.0f s (if it had started, it may still "
                                "finish; look at the GUI)" % (what, limit_s)) from None
    except RemoteRefused as e:
      self._log("BLACS: %s refused: [%s] %s" % (what, e.code, e.words))
      raise Refused(e.code, e.words) from None
    return result


class BigSkyMenuServer(QObject):
  """The menu server on its own thread, on the menu's port (REP, tcp://*:<MENU.port>)."""
  logMessage = pyqtSignal(str)

  def __init__(self, parent=None):
    super().__init__(parent)
    self.link = BigSkyMenuLink(log=self._log)
    self._stop_event = threading.Event()
    self._thread = None

  def start(self):
    if self._thread is not None and self._thread.is_alive():
      self._log("BLACS link: already running."); return
    self._stop_event.clear()
    self._thread = threading.Thread(target=self._serverLoop, daemon=True, name="BigSkyMenuServer")
    self._thread.start()

  def stop(self):
    self._stop_event.set()
    if self._thread is not None:
      self._thread.join(timeout=3.0)
      self._thread = None
    self._log("BLACS link: stopped.")

  def _log(self, msg):
    print(msg)
    self.logMessage.emit(msg)

  def _serverLoop(self):
    try:
      transport = ZmqRepTransport("tcp://*:%d" % MENU.port, recv_timeout_ms=250)
    except Exception as e:     # noqa: BLE001  (port taken, no pyzmq): the GUI runs without BLACS
      self._log("BLACS link: not started, port %d: %s" % (MENU.port, e)); return
    server = self.link.menuServer(transport)
    self._log("BLACS link: menu %s v%d on port %d." % (MENU.name, MENU.version, MENU.port))
    failures = 0
    while not self._stop_event.is_set():
      try:
        server.serve_once(timeout_ms=250)
        failures = 0
      except Exception as e:   # noqa: BLE001  (handler errors are answered; this is the socket)
        failures += 1
        self._log("BLACS link: socket error (%d/5): %s" % (failures, e))
        if failures >= 5:
          self._log("BLACS link: too many socket errors; stopped."); break
    transport.close()


# --- Connection Name Mapping ---
# Maps laser serial numbers to BLACS connection names.
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
    laser, YAG_1, is open (no serial port), so BLACS can connect, read and press (every press
    fails at step 1: no serial connection)."""
    super().__init__()
    self.setWindowTitle('Big Sky Controller Hub' + (' (no laser)' if noLaser else ''))
    self.setWindowIcon(QIcon('BigSkyDesktopIcon.ico'))
    self.left = 0; self.width = 900
    self.top = 0 ; self.height = 800
    self.setGeometry(self.left, self.top, self.width, self.height)

    # BLACS link (the menu server); lasers are added to it as their tabs open
    self.zmqServer = BigSkyMenuServer(parent=self)
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
      if dev in self.devices:
        continue  # already known
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
    self.parent.zmqServer.link.addLaser(connName, ctrl)
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
      self.parent.zmqServer.link.removeLaser(connName)

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
