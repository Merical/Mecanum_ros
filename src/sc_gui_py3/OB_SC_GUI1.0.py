import sys
from PyQt5.QtWidgets import QApplication, QWidget, QTableWidgetItem, QMainWindow, QAction, qApp
from PyQt5 import QtCore
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import cv2
import queue
import socket
import re
import json
import time
import numpy as np
from oceanbotech_widgets import *


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('OceanBotech SC')
        self.setWindowIcon(QIcon("oceanbotech.png"))

        self.setGeometry(300, 300, 300, 200)
        self.setFixedWidth(900)
        self.setFixedHeight(481)

        exitAction = QAction(QIcon('heart256.ico'), '&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit app')
        exitAction.triggered.connect(qApp.quit)

        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&file')
        fileMenu.addAction(exitAction)

        self.mode = Mode(self)
        self.setCentralWidget(self.mode)
        self.show()

    def restart(self):
        del self.mode
        self.mode = Mode(self)
        self.setCentralWidget(self.mode)
        self.show()

class Mode(QWidget):
    def __init__(self, mainWindow):
        super().__init__()
        layout = QtWidgets.QHBoxLayout()
        self.mainWindow = mainWindow

        self.button_pad = QtWidgets.QPushButton(u'Normal Mode')
        self.button_pad.setBaseSize(120, 200)
        self.button_pad.setMinimumSize(120, 200)
        layout.addWidget(self.button_pad)
        self.button_pad.clicked.connect(self.mode_pad_on_click)

        self.button_ros = QtWidgets.QPushButton(u'ROS Mode')
        self.button_ros.setBaseSize(120, 200)
        self.button_ros.setMinimumSize(120, 200)
        layout.addWidget(self.button_ros)
        self.button_ros.clicked.connect(self.mode_ros_on_click)

        self.button_dl = QtWidgets.QPushButton(u'Deep Learning')
        self.button_dl.setBaseSize(120, 200)
        self.button_dl.setMinimumSize(120, 200)
        layout.addWidget(self.button_dl)
        self.button_dl.clicked.connect(self.mode_dl_on_click)

        self.setLayout(layout)
        self.show()

    def mode_pad_on_click(self):
        self.mode = "pad"
        self.mainWindow.setCentralWidget(Window_pad(self.mainWindow))

    def mode_ros_on_click(self):
        self.mode = "ros"
        self.mainWindow.setCentralWidget(Window_ros(self.mainWindow))

    def mode_dl_on_click(self):
        self.mode = "dl"
        self.mainWindow.setCentralWidget(Window_dl(self.mainWindow))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = MainWindow()
    sys.exit(app.exec_())
