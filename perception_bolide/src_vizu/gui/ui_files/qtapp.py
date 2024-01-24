# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'uiapp.ui'
##
## Created by: Qt User Interface Compiler version 6.5.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QGridLayout, QHBoxLayout, QLabel, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QSlider,
    QSpacerItem, QSpinBox, QSplitter, QStatusBar,
    QTabWidget, QToolBox, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(324, 686)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_7 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.splitter_9 = QSplitter(self.centralwidget)
        self.splitter_9.setObjectName(u"splitter_9")
        self.splitter_9.setOrientation(Qt.Vertical)
        self.tabWidget = QTabWidget(self.splitter_9)
        self.tabWidget.setObjectName(u"tabWidget")
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.gridLayout_3 = QGridLayout(self.tab)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.toolBox = QToolBox(self.tab)
        self.toolBox.setObjectName(u"toolBox")
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.page.setGeometry(QRect(0, 0, 270, 375))
        self.verticalLayout = QVBoxLayout(self.page)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.splitter_2 = QSplitter(self.page)
        self.splitter_2.setObjectName(u"splitter_2")
        self.splitter_2.setOrientation(Qt.Vertical)
        self.label_6 = QLabel(self.splitter_2)
        self.label_6.setObjectName(u"label_6")
        self.splitter_2.addWidget(self.label_6)
        self.layoutWidget = QWidget(self.splitter_2)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.gridLayout_2 = QGridLayout(self.layoutWidget)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(self.layoutWidget)
        self.label.setObjectName(u"label")

        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)

        self.temporalFilterSlider = QSlider(self.layoutWidget)
        self.temporalFilterSlider.setObjectName(u"temporalFilterSlider")
        self.temporalFilterSlider.setMinimum(1)
        self.temporalFilterSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.temporalFilterSlider, 0, 1, 1, 1)

        self.spinBox_2 = QSpinBox(self.layoutWidget)
        self.spinBox_2.setObjectName(u"spinBox_2")
        self.spinBox_2.setMinimum(1)

        self.gridLayout_2.addWidget(self.spinBox_2, 0, 2, 1, 1)

        self.temporalFilterCheckBox = QCheckBox(self.layoutWidget)
        self.temporalFilterCheckBox.setObjectName(u"temporalFilterCheckBox")

        self.gridLayout_2.addWidget(self.temporalFilterCheckBox, 0, 3, 1, 1)

        self.label_2 = QLabel(self.layoutWidget)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout_2.addWidget(self.label_2, 1, 0, 1, 1)

        self.spatialFilterSlider = QSlider(self.layoutWidget)
        self.spatialFilterSlider.setObjectName(u"spatialFilterSlider")
        self.spatialFilterSlider.setMinimum(1)
        self.spatialFilterSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_2.addWidget(self.spatialFilterSlider, 1, 1, 1, 1)

        self.spinBox = QSpinBox(self.layoutWidget)
        self.spinBox.setObjectName(u"spinBox")
        self.spinBox.setMinimum(1)

        self.gridLayout_2.addWidget(self.spinBox, 1, 2, 1, 1)

        self.spatialFilterCheckBox = QCheckBox(self.layoutWidget)
        self.spatialFilterCheckBox.setObjectName(u"spatialFilterCheckBox")

        self.gridLayout_2.addWidget(self.spatialFilterCheckBox, 1, 3, 1, 1)

        self.splitter_2.addWidget(self.layoutWidget)

        self.verticalLayout.addWidget(self.splitter_2)

        self.splitter = QSplitter(self.page)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Vertical)
        self.label_5 = QLabel(self.splitter)
        self.label_5.setObjectName(u"label_5")
        self.splitter.addWidget(self.label_5)
        self.layoutWidget1 = QWidget(self.splitter)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.gridLayout = QGridLayout(self.layoutWidget1)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.spinBox_3 = QSpinBox(self.layoutWidget1)
        self.spinBox_3.setObjectName(u"spinBox_3")
        self.spinBox_3.setMinimum(-180)
        self.spinBox_3.setMaximum(0)

        self.gridLayout.addWidget(self.spinBox_3, 0, 2, 1, 1)

        self.label_3 = QLabel(self.layoutWidget1)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1)

        self.lidarMinAngleSlider = QSlider(self.layoutWidget1)
        self.lidarMinAngleSlider.setObjectName(u"lidarMinAngleSlider")
        self.lidarMinAngleSlider.setMinimum(-180)
        self.lidarMinAngleSlider.setMaximum(0)
        self.lidarMinAngleSlider.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.lidarMinAngleSlider, 0, 1, 1, 1)

        self.label_4 = QLabel(self.layoutWidget1)
        self.label_4.setObjectName(u"label_4")

        self.gridLayout.addWidget(self.label_4, 1, 0, 1, 1)

        self.lidarMaxAngleSlider = QSlider(self.layoutWidget1)
        self.lidarMaxAngleSlider.setObjectName(u"lidarMaxAngleSlider")
        self.lidarMaxAngleSlider.setMinimum(0)
        self.lidarMaxAngleSlider.setMaximum(180)
        self.lidarMaxAngleSlider.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.lidarMaxAngleSlider, 1, 1, 1, 1)

        self.spinBox_4 = QSpinBox(self.layoutWidget1)
        self.spinBox_4.setObjectName(u"spinBox_4")
        self.spinBox_4.setMinimum(0)
        self.spinBox_4.setMaximum(180)

        self.gridLayout.addWidget(self.spinBox_4, 1, 2, 1, 1)

        self.splitter.addWidget(self.layoutWidget1)

        self.verticalLayout.addWidget(self.splitter)

        self.splitter_3 = QSplitter(self.page)
        self.splitter_3.setObjectName(u"splitter_3")
        self.splitter_3.setOrientation(Qt.Vertical)
        self.label_8 = QLabel(self.splitter_3)
        self.label_8.setObjectName(u"label_8")
        self.splitter_3.addWidget(self.label_8)
        self.gridLayoutWidget = QWidget(self.splitter_3)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayout_4 = QGridLayout(self.gridLayoutWidget)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.lidarDisplayLimSlider = QSlider(self.gridLayoutWidget)
        self.lidarDisplayLimSlider.setObjectName(u"lidarDisplayLimSlider")
        self.lidarDisplayLimSlider.setMinimum(10)
        self.lidarDisplayLimSlider.setMaximum(15000)
        self.lidarDisplayLimSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_4.addWidget(self.lidarDisplayLimSlider, 0, 1, 1, 1)

        self.label_9 = QLabel(self.gridLayoutWidget)
        self.label_9.setObjectName(u"label_9")

        self.gridLayout_4.addWidget(self.label_9, 0, 0, 1, 1)

        self.lidarDisplayLimSpinBox = QDoubleSpinBox(self.gridLayoutWidget)
        self.lidarDisplayLimSpinBox.setObjectName(u"lidarDisplayLimSpinBox")
        self.lidarDisplayLimSpinBox.setDecimals(3)
        self.lidarDisplayLimSpinBox.setMinimum(0.010000000000000)
        self.lidarDisplayLimSpinBox.setMaximum(15.000000000000000)
        self.lidarDisplayLimSpinBox.setSingleStep(0.100000000000000)

        self.gridLayout_4.addWidget(self.lidarDisplayLimSpinBox, 0, 2, 1, 1)

        self.splitter_3.addWidget(self.gridLayoutWidget)

        self.verticalLayout.addWidget(self.splitter_3)

        self.toolBox.addItem(self.page, u"LiDAR")
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
        self.page_2.setGeometry(QRect(0, 0, 270, 329))
        self.verticalLayout_6 = QVBoxLayout(self.page_2)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.enableCameraCheckBox = QCheckBox(self.page_2)
        self.enableCameraCheckBox.setObjectName(u"enableCameraCheckBox")

        self.verticalLayout_6.addWidget(self.enableCameraCheckBox)

        self.splitter_11 = QSplitter(self.page_2)
        self.splitter_11.setObjectName(u"splitter_11")
        self.splitter_11.setOrientation(Qt.Vertical)
        self.label_16 = QLabel(self.splitter_11)
        self.label_16.setObjectName(u"label_16")
        self.splitter_11.addWidget(self.label_16)
        self.splitter_10 = QSplitter(self.splitter_11)
        self.splitter_10.setObjectName(u"splitter_10")
        self.splitter_10.setOrientation(Qt.Vertical)
        self.layoutWidget2 = QWidget(self.splitter_10)
        self.layoutWidget2.setObjectName(u"layoutWidget2")
        self.horizontalLayout_2 = QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label_17 = QLabel(self.layoutWidget2)
        self.label_17.setObjectName(u"label_17")

        self.horizontalLayout_2.addWidget(self.label_17)

        self.redAutoThresholdPushButton = QPushButton(self.layoutWidget2)
        self.redAutoThresholdPushButton.setObjectName(u"redAutoThresholdPushButton")

        self.horizontalLayout_2.addWidget(self.redAutoThresholdPushButton)

        self.splitter_10.addWidget(self.layoutWidget2)
        self.layoutWidget3 = QWidget(self.splitter_10)
        self.layoutWidget3.setObjectName(u"layoutWidget3")
        self.gridLayout_7 = QGridLayout(self.layoutWidget3)
        self.gridLayout_7.setObjectName(u"gridLayout_7")
        self.gridLayout_7.setContentsMargins(0, 0, 0, 0)
        self.label_18 = QLabel(self.layoutWidget3)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setAlignment(Qt.AlignCenter)

        self.gridLayout_7.addWidget(self.label_18, 0, 1, 1, 1)

        self.label_19 = QLabel(self.layoutWidget3)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setAlignment(Qt.AlignCenter)

        self.gridLayout_7.addWidget(self.label_19, 0, 2, 1, 1)

        self.label_20 = QLabel(self.layoutWidget3)
        self.label_20.setObjectName(u"label_20")
        self.label_20.setAlignment(Qt.AlignCenter)

        self.gridLayout_7.addWidget(self.label_20, 0, 3, 1, 1)

        self.label_21 = QLabel(self.layoutWidget3)
        self.label_21.setObjectName(u"label_21")
        self.label_21.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_7.addWidget(self.label_21, 1, 0, 1, 1)

        self.redMinRThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMinRThresholdSpinBox.setObjectName(u"redMinRThresholdSpinBox")
        self.redMinRThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMinRThresholdSpinBox, 1, 1, 1, 1)

        self.redMinGThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMinGThresholdSpinBox.setObjectName(u"redMinGThresholdSpinBox")
        self.redMinGThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMinGThresholdSpinBox, 1, 2, 1, 1)

        self.redMinBThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMinBThresholdSpinBox.setObjectName(u"redMinBThresholdSpinBox")
        self.redMinBThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMinBThresholdSpinBox, 1, 3, 1, 1)

        self.label_22 = QLabel(self.layoutWidget3)
        self.label_22.setObjectName(u"label_22")
        self.label_22.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_7.addWidget(self.label_22, 2, 0, 1, 1)

        self.redMaxRThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMaxRThresholdSpinBox.setObjectName(u"redMaxRThresholdSpinBox")
        self.redMaxRThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMaxRThresholdSpinBox, 2, 1, 1, 1)

        self.redMaxGThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMaxGThresholdSpinBox.setObjectName(u"redMaxGThresholdSpinBox")
        self.redMaxGThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMaxGThresholdSpinBox, 2, 2, 1, 1)

        self.redMaxBThresholdSpinBox = QSpinBox(self.layoutWidget3)
        self.redMaxBThresholdSpinBox.setObjectName(u"redMaxBThresholdSpinBox")
        self.redMaxBThresholdSpinBox.setMaximum(255)

        self.gridLayout_7.addWidget(self.redMaxBThresholdSpinBox, 2, 3, 1, 1)

        self.splitter_10.addWidget(self.layoutWidget3)
        self.layoutWidget4 = QWidget(self.splitter_10)
        self.layoutWidget4.setObjectName(u"layoutWidget4")
        self.horizontalLayout = QHBoxLayout(self.layoutWidget4)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.label_23 = QLabel(self.layoutWidget4)
        self.label_23.setObjectName(u"label_23")

        self.horizontalLayout.addWidget(self.label_23)

        self.greenAutoThresholdPushButton = QPushButton(self.layoutWidget4)
        self.greenAutoThresholdPushButton.setObjectName(u"greenAutoThresholdPushButton")

        self.horizontalLayout.addWidget(self.greenAutoThresholdPushButton)

        self.splitter_10.addWidget(self.layoutWidget4)
        self.layoutWidget_2 = QWidget(self.splitter_10)
        self.layoutWidget_2.setObjectName(u"layoutWidget_2")
        self.gridLayout_8 = QGridLayout(self.layoutWidget_2)
        self.gridLayout_8.setObjectName(u"gridLayout_8")
        self.gridLayout_8.setContentsMargins(0, 0, 0, 0)
        self.label_24 = QLabel(self.layoutWidget_2)
        self.label_24.setObjectName(u"label_24")
        self.label_24.setAlignment(Qt.AlignCenter)

        self.gridLayout_8.addWidget(self.label_24, 0, 1, 1, 1)

        self.label_25 = QLabel(self.layoutWidget_2)
        self.label_25.setObjectName(u"label_25")
        self.label_25.setAlignment(Qt.AlignCenter)

        self.gridLayout_8.addWidget(self.label_25, 0, 2, 1, 1)

        self.label_26 = QLabel(self.layoutWidget_2)
        self.label_26.setObjectName(u"label_26")
        self.label_26.setAlignment(Qt.AlignCenter)

        self.gridLayout_8.addWidget(self.label_26, 0, 3, 1, 1)

        self.label_27 = QLabel(self.layoutWidget_2)
        self.label_27.setObjectName(u"label_27")
        self.label_27.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_8.addWidget(self.label_27, 1, 0, 1, 1)

        self.greenMinRThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMinRThresholdSpinBox.setObjectName(u"greenMinRThresholdSpinBox")
        self.greenMinRThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMinRThresholdSpinBox, 1, 1, 1, 1)

        self.greenMinGThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMinGThresholdSpinBox.setObjectName(u"greenMinGThresholdSpinBox")
        self.greenMinGThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMinGThresholdSpinBox, 1, 2, 1, 1)

        self.greenMinBThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMinBThresholdSpinBox.setObjectName(u"greenMinBThresholdSpinBox")
        self.greenMinBThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMinBThresholdSpinBox, 1, 3, 1, 1)

        self.label_28 = QLabel(self.layoutWidget_2)
        self.label_28.setObjectName(u"label_28")
        self.label_28.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_8.addWidget(self.label_28, 2, 0, 1, 1)

        self.greenMaxRThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMaxRThresholdSpinBox.setObjectName(u"greenMaxRThresholdSpinBox")
        self.greenMaxRThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMaxRThresholdSpinBox, 2, 1, 1, 1)

        self.greenMaxGThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMaxGThresholdSpinBox.setObjectName(u"greenMaxGThresholdSpinBox")
        self.greenMaxGThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMaxGThresholdSpinBox, 2, 2, 1, 1)

        self.greenMaxBThresholdSpinBox = QSpinBox(self.layoutWidget_2)
        self.greenMaxBThresholdSpinBox.setObjectName(u"greenMaxBThresholdSpinBox")
        self.greenMaxBThresholdSpinBox.setMaximum(255)

        self.gridLayout_8.addWidget(self.greenMaxBThresholdSpinBox, 2, 3, 1, 1)

        self.splitter_10.addWidget(self.layoutWidget_2)
        self.splitter_11.addWidget(self.splitter_10)

        self.verticalLayout_6.addWidget(self.splitter_11)

        self.toolBox.addItem(self.page_2, u"Camera")

        self.gridLayout_3.addWidget(self.toolBox, 1, 0, 1, 1)

        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.verticalLayout_3 = QVBoxLayout(self.tab_2)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.toolBox_3 = QToolBox(self.tab_2)
        self.toolBox_3.setObjectName(u"toolBox_3")
        self.page_5 = QWidget()
        self.page_5.setObjectName(u"page_5")
        self.page_5.setGeometry(QRect(0, 0, 270, 375))
        self.verticalLayout_5 = QVBoxLayout(self.page_5)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.gridLayout_6 = QGridLayout()
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.gainVitesseSlider = QSlider(self.page_5)
        self.gainVitesseSlider.setObjectName(u"gainVitesseSlider")
        self.gainVitesseSlider.setMaximum(100)
        self.gainVitesseSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_6.addWidget(self.gainVitesseSlider, 0, 1, 1, 1)

        self.label_15 = QLabel(self.page_5)
        self.label_15.setObjectName(u"label_15")

        self.gridLayout_6.addWidget(self.label_15, 1, 0, 1, 1)

        self.gainDirectionSpinBox = QDoubleSpinBox(self.page_5)
        self.gainDirectionSpinBox.setObjectName(u"gainDirectionSpinBox")
        self.gainDirectionSpinBox.setMaximum(1.000000000000000)
        self.gainDirectionSpinBox.setSingleStep(0.010000000000000)

        self.gridLayout_6.addWidget(self.gainDirectionSpinBox, 1, 2, 1, 1)

        self.gainDirectionSlider = QSlider(self.page_5)
        self.gainDirectionSlider.setObjectName(u"gainDirectionSlider")
        self.gainDirectionSlider.setMaximum(100)
        self.gainDirectionSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_6.addWidget(self.gainDirectionSlider, 1, 1, 1, 1)

        self.gainVitesseSpinBox = QDoubleSpinBox(self.page_5)
        self.gainVitesseSpinBox.setObjectName(u"gainVitesseSpinBox")
        self.gainVitesseSpinBox.setMaximum(1.000000000000000)
        self.gainVitesseSpinBox.setSingleStep(0.010000000000000)

        self.gridLayout_6.addWidget(self.gainVitesseSpinBox, 0, 2, 1, 1)

        self.label_14 = QLabel(self.page_5)
        self.label_14.setObjectName(u"label_14")

        self.gridLayout_6.addWidget(self.label_14, 0, 0, 1, 1)

        self.label_32 = QLabel(self.page_5)
        self.label_32.setObjectName(u"label_32")

        self.gridLayout_6.addWidget(self.label_32, 2, 0, 1, 1)

        self.gainDirectionArgMaxSlider = QSlider(self.page_5)
        self.gainDirectionArgMaxSlider.setObjectName(u"gainDirectionArgMaxSlider")
        self.gainDirectionArgMaxSlider.setMaximum(100)
        self.gainDirectionArgMaxSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_6.addWidget(self.gainDirectionArgMaxSlider, 2, 1, 1, 1)

        self.gainDirectionArgMaxSpinBox = QDoubleSpinBox(self.page_5)
        self.gainDirectionArgMaxSpinBox.setObjectName(u"gainDirectionArgMaxSpinBox")
        self.gainDirectionArgMaxSpinBox.setMaximum(1.000000000000000)
        self.gainDirectionArgMaxSpinBox.setSingleStep(0.010000000000000)

        self.gridLayout_6.addWidget(self.gainDirectionArgMaxSpinBox, 2, 2, 1, 1)


        self.verticalLayout_5.addLayout(self.gridLayout_6)

        self.verticalSpacer_3 = QSpacerItem(20, 255, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_5.addItem(self.verticalSpacer_3)

        self.toolBox_3.addItem(self.page_5, u"Gains")
        self.page_4 = QWidget()
        self.page_4.setObjectName(u"page_4")
        self.verticalLayout_8 = QVBoxLayout(self.page_4)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.gridLayout_9 = QGridLayout()
        self.gridLayout_9.setObjectName(u"gridLayout_9")
        self.label_29 = QLabel(self.page_4)
        self.label_29.setObjectName(u"label_29")
        self.label_29.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_9.addWidget(self.label_29, 0, 0, 1, 1)

        self.FDMSlider = QSlider(self.page_4)
        self.FDMSlider.setObjectName(u"FDMSlider")
        self.FDMSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_9.addWidget(self.FDMSlider, 0, 1, 1, 1)

        self.FDMSpinBox = QDoubleSpinBox(self.page_4)
        self.FDMSpinBox.setObjectName(u"FDMSpinBox")

        self.gridLayout_9.addWidget(self.FDMSpinBox, 0, 2, 1, 1)

        self.label_30 = QLabel(self.page_4)
        self.label_30.setObjectName(u"label_30")
        self.label_30.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_9.addWidget(self.label_30, 1, 0, 1, 1)

        self.FDSlider = QSlider(self.page_4)
        self.FDSlider.setObjectName(u"FDSlider")
        self.FDSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_9.addWidget(self.FDSlider, 1, 1, 1, 1)

        self.FDSpinBox = QDoubleSpinBox(self.page_4)
        self.FDSpinBox.setObjectName(u"FDSpinBox")

        self.gridLayout_9.addWidget(self.FDSpinBox, 1, 2, 1, 1)

        self.label_31 = QLabel(self.page_4)
        self.label_31.setObjectName(u"label_31")
        self.label_31.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout_9.addWidget(self.label_31, 2, 0, 1, 1)

        self.BDMSlider = QSlider(self.page_4)
        self.BDMSlider.setObjectName(u"BDMSlider")
        self.BDMSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_9.addWidget(self.BDMSlider, 2, 1, 1, 1)

        self.BDMSpinBox = QDoubleSpinBox(self.page_4)
        self.BDMSpinBox.setObjectName(u"BDMSpinBox")

        self.gridLayout_9.addWidget(self.BDMSpinBox, 2, 2, 1, 1)


        self.verticalLayout_8.addLayout(self.gridLayout_9)

        self.verticalSpacer_2 = QSpacerItem(20, 240, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_8.addItem(self.verticalSpacer_2)

        self.toolBox_3.addItem(self.page_4, u"Thresholds")

        self.verticalLayout_3.addWidget(self.toolBox_3)

        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.verticalLayout_2 = QVBoxLayout(self.tab_3)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.toolBox_2 = QToolBox(self.tab_3)
        self.toolBox_2.setObjectName(u"toolBox_2")
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"page_3")
        self.page_3.setGeometry(QRect(0, 0, 270, 409))
        self.verticalLayout_4 = QVBoxLayout(self.page_3)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.label_10 = QLabel(self.page_3)
        self.label_10.setObjectName(u"label_10")

        self.verticalLayout_4.addWidget(self.label_10)

        self.gridLayout_5 = QGridLayout()
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.label_12 = QLabel(self.page_3)
        self.label_12.setObjectName(u"label_12")

        self.gridLayout_5.addWidget(self.label_12, 1, 0, 1, 1)

        self.simulationMaxAngleSlider = QSlider(self.page_3)
        self.simulationMaxAngleSlider.setObjectName(u"simulationMaxAngleSlider")
        self.simulationMaxAngleSlider.setMinimum(10)
        self.simulationMaxAngleSlider.setMaximum(35)
        self.simulationMaxAngleSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_5.addWidget(self.simulationMaxAngleSlider, 1, 1, 1, 1)

        self.spinBox_5 = QSpinBox(self.page_3)
        self.spinBox_5.setObjectName(u"spinBox_5")
        self.spinBox_5.setMinimum(10)
        self.spinBox_5.setMaximum(45)

        self.gridLayout_5.addWidget(self.spinBox_5, 1, 2, 1, 1)

        self.simulationMaxSpeedSlider = QSlider(self.page_3)
        self.simulationMaxSpeedSlider.setObjectName(u"simulationMaxSpeedSlider")
        self.simulationMaxSpeedSlider.setMinimum(1)
        self.simulationMaxSpeedSlider.setMaximum(50)
        self.simulationMaxSpeedSlider.setOrientation(Qt.Horizontal)

        self.gridLayout_5.addWidget(self.simulationMaxSpeedSlider, 0, 1, 1, 1)

        self.label_11 = QLabel(self.page_3)
        self.label_11.setObjectName(u"label_11")

        self.gridLayout_5.addWidget(self.label_11, 0, 0, 1, 1)

        self.spinBox_6 = QSpinBox(self.page_3)
        self.spinBox_6.setObjectName(u"spinBox_6")
        self.spinBox_6.setMinimum(1)
        self.spinBox_6.setMaximum(50)

        self.gridLayout_5.addWidget(self.spinBox_6, 0, 2, 1, 1)


        self.verticalLayout_4.addLayout(self.gridLayout_5)

        self.verticalSpacer = QSpacerItem(20, 215, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_4.addItem(self.verticalSpacer)

        self.toolBox_2.addItem(self.page_3, u"Simulation")

        self.verticalLayout_2.addWidget(self.toolBox_2)

        self.tabWidget.addTab(self.tab_3, "")
        self.splitter_9.addWidget(self.tabWidget)
        self.splitter_8 = QSplitter(self.splitter_9)
        self.splitter_8.setObjectName(u"splitter_8")
        self.splitter_8.setOrientation(Qt.Vertical)
        self.label_13 = QLabel(self.splitter_8)
        self.label_13.setObjectName(u"label_13")
        self.splitter_8.addWidget(self.label_13)
        self.splitter_6 = QSplitter(self.splitter_8)
        self.splitter_6.setObjectName(u"splitter_6")
        self.splitter_6.setOrientation(Qt.Horizontal)
        self.saveParamPushButton = QPushButton(self.splitter_6)
        self.saveParamPushButton.setObjectName(u"saveParamPushButton")
        self.splitter_6.addWidget(self.saveParamPushButton)
        self.splitter_8.addWidget(self.splitter_6)
        self.splitter_7 = QSplitter(self.splitter_8)
        self.splitter_7.setObjectName(u"splitter_7")
        self.splitter_7.setOrientation(Qt.Horizontal)
        self.label_7 = QLabel(self.splitter_7)
        self.label_7.setObjectName(u"label_7")
        self.splitter_7.addWidget(self.label_7)
        self.loadParamComboBox = QComboBox(self.splitter_7)
        self.loadParamComboBox.setObjectName(u"loadParamComboBox")
        self.splitter_7.addWidget(self.loadParamComboBox)
        self.loadParamPushButton = QPushButton(self.splitter_7)
        self.loadParamPushButton.setObjectName(u"loadParamPushButton")
        self.splitter_7.addWidget(self.loadParamPushButton)
        self.splitter_8.addWidget(self.splitter_7)
        self.splitter_9.addWidget(self.splitter_8)

        self.verticalLayout_7.addWidget(self.splitter_9)

        self.splitter_5 = QSplitter(self.centralwidget)
        self.splitter_5.setObjectName(u"splitter_5")
        self.splitter_5.setOrientation(Qt.Vertical)
        self.splitter_4 = QSplitter(self.splitter_5)
        self.splitter_4.setObjectName(u"splitter_4")
        self.splitter_4.setOrientation(Qt.Horizontal)
        self.splitter_5.addWidget(self.splitter_4)

        self.verticalLayout_7.addWidget(self.splitter_5)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 324, 24))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.temporalFilterSlider.valueChanged.connect(self.spinBox_2.setValue)
        self.spinBox_2.valueChanged.connect(self.temporalFilterSlider.setValue)
        self.spatialFilterSlider.valueChanged.connect(self.spinBox.setValue)
        self.spinBox.valueChanged.connect(self.spatialFilterSlider.setValue)
        self.lidarMinAngleSlider.valueChanged.connect(self.spinBox_3.setValue)
        self.spinBox_4.valueChanged.connect(self.lidarMaxAngleSlider.setValue)
        self.spinBox_3.valueChanged.connect(self.lidarMinAngleSlider.setValue)
        self.lidarMaxAngleSlider.valueChanged.connect(self.spinBox_4.setValue)
        self.simulationMaxSpeedSlider.valueChanged.connect(self.spinBox_6.setValue)
        self.spinBox_6.valueChanged.connect(self.simulationMaxSpeedSlider.setValue)
        self.simulationMaxAngleSlider.valueChanged.connect(self.spinBox_5.setValue)
        self.spinBox_5.valueChanged.connect(self.simulationMaxAngleSlider.setValue)

        self.tabWidget.setCurrentIndex(1)
        self.toolBox.setCurrentIndex(0)
        self.toolBox_3.setCurrentIndex(0)
        self.toolBox_2.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Filter size", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Temporal", None))
        self.temporalFilterCheckBox.setText("")
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Spatial", None))
        self.spatialFilterCheckBox.setText("")
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Angles range", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Min", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Max", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Visualisation", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Lim", None))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page), QCoreApplication.translate("MainWindow", u"LiDAR", None))
        self.enableCameraCheckBox.setText(QCoreApplication.translate("MainWindow", u"Enable camera", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Calibration", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Red", None))
        self.redAutoThresholdPushButton.setText(QCoreApplication.translate("MainWindow", u"Auto", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"R", None))
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"G", None))
        self.label_20.setText(QCoreApplication.translate("MainWindow", u"B", None))
        self.label_21.setText(QCoreApplication.translate("MainWindow", u"Min", None))
        self.label_22.setText(QCoreApplication.translate("MainWindow", u"Max", None))
        self.label_23.setText(QCoreApplication.translate("MainWindow", u"Green", None))
        self.greenAutoThresholdPushButton.setText(QCoreApplication.translate("MainWindow", u"Auto", None))
        self.label_24.setText(QCoreApplication.translate("MainWindow", u"R", None))
        self.label_25.setText(QCoreApplication.translate("MainWindow", u"G", None))
        self.label_26.setText(QCoreApplication.translate("MainWindow", u"B", None))
        self.label_27.setText(QCoreApplication.translate("MainWindow", u"Min", None))
        self.label_28.setText(QCoreApplication.translate("MainWindow", u"Max", None))
        self.toolBox.setItemText(self.toolBox.indexOf(self.page_2), QCoreApplication.translate("MainWindow", u"Camera", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Perception", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Kd", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"Kv", None))
        self.label_32.setText(QCoreApplication.translate("MainWindow", u"Ka", None))
        self.toolBox_3.setItemText(self.toolBox_3.indexOf(self.page_5), QCoreApplication.translate("MainWindow", u"Gains", None))
        self.label_29.setText(QCoreApplication.translate("MainWindow", u"FDM", None))
        self.label_30.setText(QCoreApplication.translate("MainWindow", u"FD", None))
        self.label_31.setText(QCoreApplication.translate("MainWindow", u"BDM", None))
        self.toolBox_3.setItemText(self.toolBox_3.indexOf(self.page_4), QCoreApplication.translate("MainWindow", u"Thresholds", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Planning", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Max", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"Angle", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Speed", None))
        self.toolBox_2.setItemText(self.toolBox_2.indexOf(self.page_3), QCoreApplication.translate("MainWindow", u"Simulation", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Control", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"Parameters", None))
        self.saveParamPushButton.setText(QCoreApplication.translate("MainWindow", u"Save", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Load ", None))
        self.loadParamPushButton.setText(QCoreApplication.translate("MainWindow", u"Load", None))
    # retranslateUi
