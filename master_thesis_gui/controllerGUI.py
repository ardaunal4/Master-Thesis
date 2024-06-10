"""
************************************* USAGE OF PROGRAMMING *************************************
-> Default baudrate = 115200. Please set it accordingly before connect.
-> By clicking COM list, you refresh the COM list. Be sure that true COM has chosen.

-> There are 5 different variables to control timer:
    1. Delay of the first pulse
    2. Delay between 2 pulses
    3. Pulse width of pulse 1
    4. Pulse width of pulse 2
    5. Repetition Frequency of the pulse train

-> MCU will started with default variables for the first 4 variables. If user want to
change variable, it must enter the value for corresponding variable and click OK.

-> Variables accept integer value, but actually 1 corresponds to 217 ps for the first 4 different
variables metioned above. If someone want to change delay, one should use a number between
0 and 65503.

-> Note: Delay of the first pulse variable has a limit. Its smallest value is 96 and its max
value is 65503.

-> Note 2: These 2 pulses work under a master clock with a frequency of 70345 Hz. Therefore, delay
between pulses and pulse widths have limitation which should be respected. Please be aware that
what you are doing before enter values.

-> Note 3: Repetition frequency can be maximum 10 Hz because of the safety of MOSFETs in this
project!

-> Note 4: If you want to change baudrate, then change in the MCU program as well!

************************************* PROGRAMMING STRUCTURE *************************************
-> There are 6 different modes in message struct:
    1. 0x1 : Connect
    2. 0x2 : Trigger pulse
    3. 0x3 : Pulse 1 width
    4. 0x4 : Pulse 2 width
    5. 0x5 : Delay between pulses
    6. 0x6 : Delay of the pulse 1

-> All the messages have a feedback which can be seen in the serial monitor except Trigger pulse.

-> packet_value is a 64-bit struct with 1 byte header, 1 byte mode, 2 bytes variable, 1 byte of check
field. Rest of reserverd and could be dummy. Header byte is same in both MCU and here. The rest of the
packet_value is taking into consideration if the header is arrived. 

-> After all the message is received, since  communication protocol is USART, check_func checks if the 
message is arrived truely. If message is true, then variables are taking considered by the MCU. 
Otherwise, MCU will ignore the message and  will send not acknowledge message. If not acknowledge is 
received, this program will send the same message as before until acquire acknowledge.

-> Message take up 2 bytes of space. When user gives an integer value and click OK, then this value
will be converted into hexadecimal form and will be in place in the packet_bytes accordingly.
"""
import qdarkstyle
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QIcon
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import serial
from serial.tools import list_ports
import sys
import time
import threading
import matplotlib.image as mpimg
import struct

matplotlib.use('Qt5Agg')

class TaskScheduler:

    def __init__(self, initial_interval):

        self.interval = initial_interval
        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.job = None

    def start(self):

        if self.job is None:
            raise ValueError("Job function not set.")
        
        self.thread.start()

    def stop(self):

        self._stop_event.set()
        self.thread.join()

    def _run(self):

        while not self._stop_event.is_set():

            time.sleep(self.interval)
            if self.job:
                self.job()

    def set_interval(self, new_interval):

        self.interval = new_interval

class ComboBox(QComboBox):
    
    popupAboutToBeShown = pyqtSignal()

    def showPopup(self):
        self.popupAboutToBeShown.emit()
        super(ComboBox, self).showPopup()

class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None, width=10, height=8, dpi=100):

        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)

        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                QSizePolicy.Expanding,
                QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def plot(self, x, y, color, variable_name):

        ax = self.fig.add_subplot(111)
        ax.plot(x, y, color = color, label = variable_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("PWM Signal")
        plt.grid('on')
        ax.legend()
        self.draw()

    def show_image(self, img):

        self.fig.patch.set_facecolor('lightcyan')
        self.imCanvas = FigureCanvas(self.fig)
        self.axes.imshow(img)

    def clear(self):

        self.fig.clf()

class Window(QMainWindow): 
 
    def __init__(self):

        super().__init__()
        self.setGeometry(250, 250, 900, 700) 
        self.setWindowTitle("APP")
        self.setWindowIcon(QIcon('kahvelab.png'))
        self.tabWidget()
        self.Widgets()
        self.layouts()
        self.init_variables()
        self.show()

    def tabWidget(self):

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tab1 = QWidget()
        self.tabs.addTab(self.tab1, "RF SWITCH")
    
    def init_variables(self):

        self.baud_rate = 0
        self.com = ""
        self.freq_rep = 0
        self.pulse_mode = 0  # 0 means that single pulse, 1 means that repeated pulses
        self.header = 0xA
        self.ack = 0xF
        self.send_list = []
        self.scheduler = TaskScheduler(self.freq_rep)

    def message_packet(self, mode: bytes, message: bytes) -> bytes:
        
        packet_check = (self.header |
                        (mode << 4) |
                        (message << 16))
        
        check = hex(self.check_func(packet_check))

        packet_value = (self.header |
                        (mode << 4) |
                        (message << 16) |
                        (check << 24))
        
        return struct.pack('<I', packet_value)
    
    def check_func(self, message: bytes) -> int:

        sum = 0
        for i in range(24):
            sum += (message >> i) & 0b1

        return sum

    def Widgets(self):

        self.pltObj = PlotCanvas(self)

        self.baudrate_list_qlabel = QLabel("Baud Rate")
        self.baudrate_cb = QComboBox(self)
        self.baudrate_cb.addItems(["----------", "300", "600", "1200", "2400", 
                                   "4800", "9600", "14400", "19200", 
                                   "28800", "38400", "56000", "57600", 
                                   "115200", "128000", "256000"])
        self.baudrate_cb.currentTextChanged.connect(self.baudrate_clicked_func)

        self.pulse_width1_qlabel = QLabel("Change Pulse Width of the Pulse 1", self)
        self.pulse_width1_txtbox = QLineEdit(self)
        self.pulse_width1_button = QPushButton("OK", self)
        self.pulse_width1_button.clicked.connect(self.pulse_width1_func)
        self.pulse_width1_button.setEnabled(False)

        self.pulse_width2_qlabel = QLabel("Change Pulse Width of the Pulse 2", self)
        self.pulse_width2_txtbox = QLineEdit(self)
        self.pulse_width2_button = QPushButton("OK", self)
        self.pulse_width2_button.clicked.connect(self.pulse_width2_func)
        self.pulse_width2_button.setEnabled(False)

        self.delay_qlabel = QLabel("Change the Delay Between Pulses", self)
        self.delay_txtbox = QLineEdit(self)
        self.delay_button = QPushButton("OK", self)
        self.delay_button.clicked.connect(self.delay_func)
        self.delay_button.setEnabled(False)

        self.delay_p1_qlabel = QLabel("Add a Delay For the Pulse 1", self)
        self.delay_p1_txtbox = QLineEdit(self)
        self.delay_p1_button = QPushButton("OK", self)
        self.delay_p1_button.clicked.connect(self.delay_pulse1_func)
        self.delay_p1_button.setEnabled(False)

        self.rep_rate_qlabel = QLabel("Repetition Freq", self)
        self.rep_rate_txtbox = QLineEdit(self)
        self.rep_rate_button = QPushButton("OK", self)
        self.rep_rate_button.clicked.connect(self.rep_func)
        self.rep_rate_button.setEnabled(False)
        
        self.com_qlabel = QLabel("Enter COM", self)
        self.com_cb = ComboBox(self)
        self.com_cb.addItem("----------")
        self.com_cb.popupAboutToBeShown.connect(self.update_coms_func)

        self.com_qlabel = QLabel("PULSE TYPE", self)
        self.com_cb2 = ComboBox(self)
        self.com_cb2.addItem("Single Pulse")
        self.com_cb2.addItem("Repeated Pulses")
        self.com_cb2.activated[str].connect(self.mode_var_func)
        self.com_cb2.setEnabled(False)

        self.connect_button = QPushButton("Connect", self)
        self.connect_button.clicked.connect(self.connect_MCU)

        self.trigger_button = QPushButton("SEND PULSES", self)
        self.trigger_button.clicked.connect(self.send_pulse)
        self.trigger_button.setEnabled(False)

        self.configure_button = QPushButton("SET Configuration of Pulses", self)
        self.configure_button.clicked.connect(self.set_PWM)
        self.configure_button.setEnabled(False)

        self.serial_monitor_list =  QListWidget()
        self.serial_monitor_list.addItem("Welcome Beam Chopper GUI!")  
        self.serial_monitor_list.addItem("Maximum timer resolution is 217 ps!")
        self.serial_monitor_list.addItem("Please read the docstring before use the program!")

        self.clear_serial_monitor = QPushButton("CLEAR", self)
        self.clear_serial_monitor.clicked.connect(self.clear_list)

        img = mpimg.imread('kahvelab.png')
        self.pltObj.show_image(img)

    def send_pulse(self):

        if self.pulse_mode == 0:
            self.trigger_pulse()

        elif self.pulse_mode == 1:
            self.scheduler.start()

    def mode_var_func(self):

        if self.com_cb2.currentText() == "Repeated Pulses":
            self.pulse_mode = 1
            self.scheduler.job = self.trigger_pulse()
        else:
            self.scheduler.stop()
            self.pulse_mode = 0
            
    def serial_monitor(self, item):

        self.serial_monitor_list.addItem(item)

    def clear_list(self):

        self.serial_monitor_list.clear()

    def trigger_pulse(self):

        self.send(mode = 0x2, msg = 0xAA)
        self.plot_func()

    def pulse_width1_func(self):

        try:
            pulse_width1 = int(self.pulse_width1_txtbox.text())
            self.send_list.append((0x3, hex(pulse_width1)))
        except:
            info_box = QMessageBox.information(self, "WARNING!", "It is not a valid input!")
            return -1
        
    def pulse_width2_func(self):

        try:
            pulse_width2 = int(self.pulse_width2_txtbox.text())
            self.send_list.append((0x4, hex(pulse_width2)))
        except:
            info_box = QMessageBox.information(self, "WARNING!", "It is not a valid input!")
            return -1
        
    def delay_func(self):

        try:
            delay_bt_pulses = int(self.delay_txtbox.text())
            self.send_list.append((0x5, hex(delay_bt_pulses)))
        except:
            info_box = QMessageBox.information(self, "WARNING!", "It is not a valid input!")
            return -1
    
    def delay_pulse1_func(self):
        
        try:
            delay_pulse1 = int(self.delay_p1_txtbox.text())
            self.send_list.append((0x6, hex(delay_pulse1)))
        except:
            info_box = QMessageBox.information(self, "WARNING!", "It is not a valid input!")
            return -1
    
    def rep_func(self):

        try:
            self.freq_rep = float(self.rep_rate_txtbox.text())
            if self.freq_rep > 10.0:
                self.freq_rep = 10.0
            elif self.freq_rep < 0:
                self.freq_rep = 0
            self.scheduler.set_interval(self.freq_rep)
        except:
            info_box = QMessageBox.information(self, "WARNING!", "It is not a valid input!")
            return -1
        
    def receive(self, mode: int, msg: bytes):

        packet_bytes = self.mcu.read(4)
        packet_value = struct.unpack('<I', packet_bytes)[0]
        
        if (self.header == (packet_value & 0xF)):
            if(self.check_func(packet_value) == ((packet_value >> 24) & 0xFF)):
                if(((packet_value >> 4) & self.ack)):
                    self.serial_monitor("MCU received message successfully!")
                else:
                    self.send(self, mode=mode, msg=msg)
            else:
                self.send(self, mode=mode, msg=msg)
        else:
            self.send(self, mode=mode, msg=msg)

    def send(self, mode: int, msg: bytes):

        self.mcu.write(self.message_packet(mode = mode, message = msg))
        self.mcu.flush()

    def set_PWM(self):

        if self.mcu:
            for data in self.send_list:
                self.send(mode = data[0], msg = data[1])
                time.sleep(0.01)
                self.receive(mode = data[0], msg = data[1])
                self.plot_func()
                self.send_list = []
        else:
            info_box = QMessageBox.information(self, "WARNING!", "NO MCU CONNECTION!")

    def connect_MCU(self):

        self.com = self.com_cb.currentText()
        self.clear_list()

        if self.com != "" and self.baud_rate != 0:
            self.mcu = serial.Serial(self.com, self.baud_rate, timeout = 0.3)
            self.send(mode = 0x1, msg = 0x1)
            time.sleep(0.01)
            self.receive(mode = 0x1, msg = 0x1)
            # Buttons and combobox2 are enabled if MCU is connected!
            self.pulse_width1_button.setEnabled(True)
            self.pulse_width2_button.setEnabled(True)
            self.delay_button.setEnabled(True)
            self.delay_p1_button.setEnabled(True)
            self.rep_rate_button.setEnabled(True)
            self.trigger_button.setEnabled(True)
            self.configure_button.setEnabled(True)
            self.com_cb2.setEnabled(True)

    def baudrate_clicked_func(self):

        self.baud_rate = int(self.baudrate_cb.currentText())

    def update_coms_func(self):
        
        self.com_cb.clear()
        com_list = list_ports.comports()

        for port in com_list:
            port = str(port)
            port_name = port.split("-")
            self.com_cb.addItem(port_name[0])
        
    def plot_func(self):

        self.pltObj.plot(x = list(range(0, 101, 1)), y = list(range(0, 30, 1)))
        self.pltObj.plot(x = list(range(0, 101, 1)), y = list(range(60, 101, 1)))
        time.sleep(0.5)
        self.pltObj.clear()
  
    def layouts(self):

        self.main_layout   = QHBoxLayout()
        self.right_layout  = QFormLayout()
        self.left_layout   = QFormLayout()
        self.plot_layout   = QVBoxLayout()
        self.left_Hbox1    = QHBoxLayout()
        self.left_Hbox2    = QHBoxLayout()
        self.left_Hbox3    = QHBoxLayout()
        self.left_Hbox4    = QHBoxLayout()
        self.left_Hbox5    = QHBoxLayout()
        self.right_Vbox    = QVBoxLayout()

        # Left layout
        self.left_layout_group_box = QGroupBox("")
        self.left_Hbox1.addWidget(self.baudrate_list_qlabel)
        self.left_Hbox1.addWidget(self.baudrate_cb)
        self.left_Hbox1.addStretch()
        self.left_Hbox1.addWidget(self.com_qlabel)
        self.left_Hbox1.addWidget(self.com_cb)
        self.left_Hbox1.addStretch()
        self.left_Hbox1.addWidget(self.connect_button)
        self.left_layout.addRow(self.left_Hbox1)

        self.plot_layout.addWidget(self.pltObj)
        self.left_layout.addRow(self.plot_layout)

        self.left_Hbox2.addWidget(self.pulse_width1_qlabel)
        self.left_Hbox2.addWidget(self.pulse_width1_txtbox)
        self.left_Hbox2.addWidget(self.pulse_width1_button)
        self.left_Hbox2.addStretch()
        self.left_Hbox2.addWidget(self.pulse_width2_qlabel)
        self.left_Hbox2.addWidget(self.pulse_width2_txtbox)
        self.left_Hbox2.addWidget(self.pulse_width2_button)
        self.left_Hbox2.addStretch()
        self.left_layout.addRow(self.left_Hbox2)
        
        self.left_Hbox3.addWidget(self.delay_qlabel)
        self.left_Hbox3.addWidget(self.delay_txtbox)
        self.left_Hbox3.addWidget(self.delay_button)
        self.left_Hbox3.addStretch()
        self.left_Hbox3.addWidget(self.delay_p1_qlabel)
        self.left_Hbox3.addWidget(self.delay_p1_txtbox)
        self.left_Hbox3.addWidget(self.delay_p1_button)
        self.left_Hbox3.addStretch()
        self.left_layout.addRow(self.left_Hbox3) 

        self.left_Hbox3.addStretch()
        self.left_Hbox4.addWidget(self.configure_button)
        self.left_layout.addRow(self.left_Hbox4) 

        self.left_Hbox5.addWidget(self.com_cb2)
        self.left_Hbox5.addStretch()
        self.left_Hbox5.addWidget(self.rep_rate_qlabel)
        self.left_Hbox5.addWidget(self.rep_rate_txtbox)
        self.left_Hbox5.addWidget(self.rep_rate_button)
        self.left_Hbox5.addStretch()
        self.left_Hbox5.addWidget(self.trigger_button)
        self.left_Hbox5.addStretch()
        self.left_layout.addRow(self.left_Hbox5) 

        self.left_layout_group_box.setLayout(self.left_layout)

        # Right layout
        self.right_layout_group_box = QGroupBox("Serial Monitor")

        self.right_Vbox.addWidget(self.serial_monitor_list) 
        self.right_Vbox.addWidget(self.clear_serial_monitor)   
        self.right_layout.addRow(self.right_Vbox)

        self.right_layout_group_box.setLayout(self.right_layout)

        self.main_layout.addWidget(self.left_layout_group_box, 60)
        self.main_layout.addWidget(self.right_layout_group_box, 40)
        self.tab1.setLayout(self.main_layout)  

def main():

    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet()) 
    window = Window()
    sys.exit(app.exec_())

if __name__ == "__main__":

    main()