#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: raphaelkhorassani
"""
import os
import cv2
import numpy as np
import rospy
import subprocess

from ui_files.qtapp import Ui_MainWindow  
from std_msgs.msg import Bool
import pickle   # Module used to save parameters and to load them

from PySide6.QtWidgets import QMainWindow, QInputDialog, QMessageBox

# MainWindow is the main class, that we'll be imported and run in the main_gui.py node.

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # The directory will be used to load and save the parameters.
        self.dir = os.path.realpath(__file__).replace("/scripts/widget.py","")

        # We define one dictionary for each type of object : 
        #   - self.checkbox groups all check boxes, which returns a bool value (is ticked or not)
        #   - self.values groups all objects that have a numeric value (int and float)
        #   - self.combobox groups all combo boxes
        #   - self.calibrate groups all objects used for calibration. Note that their type is int, but we decided to not place them in self.values dict.
        
        # Here is how it works : 
        #   - We define a rosparam and choose whatever name we want
        #   - For each param, we define two things : 
        #       - What is the corresponding QObject in the key "object"
        #       - What is the default value in the key "default"
        #   - For comboboxes, we also define what are the different options in the key "choices"

        self.checkbox = {
            "/temporal_filter_bool"         : {"object" : self.ui.temporalFilterCheckBox,           "default" : False},
            "/spatial_filter_bool"          : {"object" : self.ui.spatialFilterCheckBox,            "default" : False},
            "/enable_camera_bool"           : {"object" : self.ui.enableCameraCheckBox,             "default" : True},
            "/green_is_left"                : {"object" : self.ui.GreenIsLeftCheckBox,              "default" : True},
            "/use_dials"                    : {"object" : self.ui.useDialsCheckBox,                 "default" : False},
            "/spaced_dials"                 : {"object" : self.ui.spacedDialsCheckBox,              "default" : True},
            "/anti_jumping_filter_bool"     : {"object" : self.ui.antiJumpingCheckBox,              "default" : False},
            "/use_maximize_threshold"       : {"object" : self.ui.useMaximiseThresholdCheckBox_2,   "default" : False},
        }

        self.values={
            "/temporal_filter_range"        : {"object" : self.ui.temporalFilterSlider,             "default" : 3},
            "/spatial_filter_range"         : {"object" : self.ui.spatialFilterSlider,              "default" : 5},
            "/lidar_min_angle_deg"          : {"object" : self.ui.lidarMinAngleSlider,              "default" : -90},
            "/lidar_max_angle_deg"          : {"object" : self.ui.lidarMaxAngleSlider,              "default" : 90},
            "/lidar_rmax"                   : {"object" : self.ui.lidarDisplayLimSpinBox,           "default" : 10},
            "/simulation_max_speed"         : {"object" : self.ui.simulationMaxSpeedSlider,         "default" : 28},
            "/simulation_max_angle"         : {"object" : self.ui.simulationMaxAngleSlider,         "default" : 25},
            "/gain_vitesse"                 : {"object" : self.ui.gainVitesseSpinBox,               "default" : 0.4},
            "/gain_direction"               : {"object" : self.ui.gainDirectionSpinBox,             "default" : 1.4},
            "/gain_direction_arg_max"       : {"object" : self.ui.gainDirectionArgMaxSpinBox,       "default" : 1.9},

            "/threshold_front_too_close"    : {"object" : self.ui.FrontTooCloseSpinBox,             "default" : 0.35},
            "/threshold_front_far_enough"   : {"object" : self.ui.FrontFarEnoughSpinBox,            "default" : 0.5},
            "/threshold_rear_too_close"     : {"object" : self.ui.RearTooCloseSpinBox,              "default" : 0.2},
            "/navigation_n_dials"           : {"object" : self.ui.numberDialsSlider,                "default" : 11},
            "/color_detection_tolerance"    : {"object" : self.ui.colorDetectionToleranceSpinBox,   "default" : 0.25},
            "/front_dial_ratio"             : {"object" : self.ui.frontRatioSlider,                 "default" : 0.2},

            "/anti_jumping_filter_range"    : {"object" : self.ui.antiJumpingSlider,                "default" : 5},

            "/maximize_threshold"           : {"object" : self.ui.maximizeThresholdSpinBox,         "default" : 5},

        }

        self.combobox = {
            "/navigation_mode"              :  {"object" : self.ui.navModeComboBox,          
                                                "default" : "NDials_Classic",
                                                "choices" : [
                                                    "NDials_Classic",
                                                    "NDials_LeftRightDivision",
                                                    "NDials_Ponderated",
                                                    "NDials_PonderatedWithoutDistanceDivision",
                                                    "NDials_PonderatedRelative"
                                                ]},

            "/navigation_feature"           :  {"object" : self.ui.featureDialsComboBox,
                                                "default" : "mean",
                                                "choices" : [
                                                    "mean",
                                                    "median",
                                                    "min",
                                                    "q1",
                                                    "q3",
                                                    ]},
        }

        self.calibrate = {
            "red" : {
                "R"  : {"object" : self.ui.RredCalibrationSpinBox, "default" : 150},
                "G"  : {"object" : self.ui.GredCalibrationSpinBox, "default" : 0},
                "B"  : {"object" : self.ui.BredCalibrationSpinBox, "default" : 0},
            },
            "green" : {
                "R"  : {"object" : self.ui.RgreenCalibrationSpinBox, "default" : 0},
                "G"  : {"object" : self.ui.GgreenCalibrationSpinBox, "default" : 150},
                "B"  : {"object" : self.ui.BgreenCalibrationSpinBox, "default" : 0},
            }
        }

        # We create a publisher on the topic /param_change_alert.
        # As we don't want nodes to check everytime if the params are changed, we decided to create this topic so that the nodes do a rospy.get_param 
        # only when a parameter is changed in the GUI. We chose arbitrarely a Bool message, we just want to publish the simplest message, it's value doesn't matter
        self.msg_alert = Bool()
        self.msg_alert.data = True
        self.changement_alert = rospy.Publisher('/param_change_alert', Bool, queue_size = 10)


        self.initialize_comboboxes()                    # This function fills the comboboxes with the different choices
        self.connect_sliders_and_double_spin_boxes()    # As sliders are integers, if we want to connect a slider with a double spin box which type is float, we have to do a conversion between integers and floats (integers represents percentages for instance)

        # This push button allows to publish a msg on the topic /param_change_alert so that if we change a rosparam in the terminal, we can send a request for all nodes to check the parameters.
        self.ui.updateParamPushButton.clicked.connect(self.only_publish)

        self.connect()                                  # This is the main method that changes a rosparam to the value selected on it's corresponding QObject
        self.set_parameters(first_use = True)           # This method set all the parameters to the default values defined in the dictionnaries
        self.get_params_names()

        # We connect the Load and Save buttons to the corresponding methods.
        self.ui.loadParamPushButton.clicked.connect(self.load_parameters)   
        self.ui.saveParamPushButton.clicked.connect(self.save_parameters)

        # We create a publisher that sends a request for an auto calibration.
        self.init_calibration = rospy.Publisher("/do_an_auto_calibration", Bool, queue_size = 10)
        # We connect the buttons to do an auto calibration for red walls and green walls
        self.ui.redAutoCalibrationPushButton.clicked.connect(lambda : self.auto_calibration(color = "red"))
        self.ui.greenAutoCalibrationPushButton.clicked.connect(lambda : self.auto_calibration(color = "green"))


    def connect(self):

        for name,info in self.values.items() :
            info["object"].valueChanged.connect(lambda value, key=name: self.change_param(value,key=key))

        for name,info in self.checkbox.items() :
            info["object"].toggled.connect(lambda value, key=name: self.change_param(bool(value),key=key))

        for name,info in self.combobox.items() :
            info["object"].currentTextChanged.connect(lambda value, key=name: self.change_param(value,key=key))

        for color in self.calibrate:
            for name, info in self.calibrate[color].items() :
                info["object"].valueChanged.connect(lambda value, color=color, key=name: self.set_color_calibration(value,color,key=key))

    def auto_calibration(self, color = "no_one", ) :
        rospy.set_param("/color_to_calibrate", color)
        self.color_to_set = color
        self.subscriber_calibration = rospy.Subscriber("/is_auto_calibration_done", Bool, self.update_spinboxes_calibration)
        self.init_calibration.publish(self.msg_alert)
        
    def update_spinboxes_calibration(self, value = True) :
        color = self.color_to_set
        thresholds = rospy.get_param(f"/{color}_RGB")

        self.calibrate[color]["R"]["default"], self.calibrate[color]["G"]["default"], self.calibrate[color]["B"]["default"] = thresholds
        
        for name, info in self.calibrate[color].items() :
            self.calibrate[color][name]["object"].setValue(info["default"])

        if value == True : 
            self.subscriber_calibration.unregister()

    def set_color_calibration(self, value, color, key = "no key"):
        self.calibrate[color][key]["default"] = value
        c = self.calibrate[color]
        rospy.set_param(f"/{color}_RGB",[c["R"]["default"],c["G"]["default"],c["B"]["default"]])
        self.changement_alert.publish(self.msg_alert)


    def connect_sliders_and_double_spin_boxes(self) :
        pairs = {
            1000 : [
                [self.ui.lidarDisplayLimSlider, self.ui.lidarDisplayLimSpinBox]
            ],
            100 : [
                [self.ui.gainVitesseSlider, self.ui.gainVitesseSpinBox],
                [self.ui.gainDirectionSlider, self.ui.gainDirectionSpinBox],
                [self.ui.gainDirectionArgMaxSlider, self.ui.gainDirectionArgMaxSpinBox],
                [self.ui.FrontFarEnoughSlider, self.ui.FrontFarEnoughSpinBox],
                [self.ui.FrontTooCloseSlider, self.ui.FrontTooCloseSpinBox],
                [self.ui.RearTooCloseSlider, self.ui.RearTooCloseSpinBox],
                [self.ui.frontRatioSlider, self.ui.frontRatioSpinBox],
                [self.ui.colorDetectionToleranceSlider, self.ui.colorDetectionToleranceSpinBox]
            ],
        }

        for factor, widgets in pairs.items():
            for slider, spinbox in widgets:
                slider.valueChanged.connect(lambda value, spinbox=spinbox: spinbox.setValue(value / factor))
                spinbox.valueChanged.connect(lambda value, slider=slider: slider.setValue(int(value * factor)))


    def set_parameters(self, first_use = False):
        for name,info in self.values.items() :
            self.values[name]["default"] = rospy.get_param(name, default = info["default"])
            info["object"].setValue(self.values[name]["default"])

        for name,info in self.checkbox.items() :
            self.checkbox[name]["default"] = rospy.get_param(name, default = info["default"])
            info["object"].setChecked(self.checkbox[name]["default"])

        for name,info in self.combobox.items() :
            self.combobox[name]["default"] = rospy.get_param(name, default = info["default"])
            info["object"].setCurrentText(self.combobox[name]["default"])

    def initialize_comboboxes(self) : 
        for name,info in self.combobox.items() :
            info["object"].clear()
            info["object"].addItems(info["choices"])  

    def change_param(self, value, key = None):

        if key in self.checkbox :
            self.checkbox[key]["default"] = value

        if key in self.values :
            self.values[key]["default"] = value
        
        if key in self.combobox :
            self.combobox[key]["default"] = value

        rospy.set_param(key, value)
        self.changement_alert.publish(self.msg_alert)

    def only_publish(self):
        self.changement_alert.publish(self.msg_alert)

    def load_parameters(self) :
        set_name = self.ui.loadParamComboBox.currentText()

        with open(self.dir + f"/params/{set_name}", 'rb') as file:
            to_load = pickle.load(file)

        if "checkbox" in to_load :
            for p in to_load["checkbox"] :
                self.checkbox[p]["default"] = to_load["checkbox"][p]

        if "values" in to_load :
            for p in to_load["values"] :
                self.values[p]["default"] = to_load["values"][p]

        if "calibration" in to_load :
            calibrate_color = to_load["calibration"]
            for color in ["red", "green"]:
                rospy.set_param(f"/{color}_RGB", calibrate_color[color])
                self.color_to_set = color
                self.update_spinboxes_calibration(value = False)

        if "combobox" in to_load :
            for p in to_load["combobox"] :
                self.combobox[p]["default"] = to_load["combobox"][p]


        self.set_parameters()


    def get_params_names(self) :

        self.ui.loadParamComboBox.clear()
        param_folder = self.dir + "/params"
        param_files = [f for f in os.listdir(param_folder) if os.path.isfile(os.path.join(param_folder, f))]
        self.ui.loadParamComboBox.addItems(param_files)

    def save_parameters(self):

        checkbox_defaults = {key: value["default"] for key, value in self.checkbox.items()}
        values_defaults = {key: value["default"] for key, value in self.values.items()}
        red_calibration = rospy.get_param("/red_RGB", default = [0]*3)
        green_calibration = rospy.get_param("/green_RGB", default = [0]*3)
        comboboxes_default = {key: value["default"] for key, value in self.combobox.items()}

        to_save = {
            "checkbox"      : checkbox_defaults,
            "values"        : values_defaults,
            "calibration"   : {"red" : red_calibration, "green" : green_calibration},
            "combobox"      : comboboxes_default,
        }
    #"thresholds": self.thresholds_color,



        # Demander à l'utilisateur le nom du fichier
        new_file_name, ok_pressed = QInputDialog.getText(self, "Sauvegarder", "Nom du fichier:")

        # Si l'utilisateur appuie sur "OK" dans la fenêtre de dialogue
        if ok_pressed:
            # Vérifier si le nom du fichier existe déjà
            complete_file_name = f"{new_file_name}.pkl" if not new_file_name.endswith(".pkl") else new_file_name
            file_path = os.path.join(self.dir + "/params", complete_file_name)

            if os.path.exists(file_path):
                # Afficher un message d'information et demander confirmation pour écraser le fichier existant
                reply = QMessageBox.question(self, "Attention", f"Le fichier '{complete_file_name}' existe déjà. Voulez-vous l'écraser?", QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    return  # L'utilisateur a choisi de ne pas écraser le fichier existant

            # Sauvegarder les données au format pickle
            with open(file_path, 'wb') as file:
                pickle.dump(to_save, file)

            # Facultatif : Mettez à jour le ComboBox avec le nouveau fichier
            self.get_params_names()