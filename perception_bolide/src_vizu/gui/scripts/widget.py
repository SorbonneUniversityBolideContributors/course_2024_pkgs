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
from easydict import EasyDict

from ui_files.qtapp import Ui_MainWindow    # Correspond à la fenêtre principale du GUI
from std_msgs.msg import Bool
# from scripts.calibrate_color import DetectColor
import pickle   # Permet de sauvegarder les paramètres et les charger

from PySide6.QtWidgets import QMainWindow, QInputDialog, QMessageBox

import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.dir = os.path.realpath(__file__).replace("/scripts/widget.py","")

        self.checkbox = {
            "/temporal_filter_bool"     : {"object" : self.ui.temporalFilterCheckBox,   "default" : False},
            "/spatial_filter_bool"      : {"object" : self.ui.spatialFilterCheckBox,    "default" : False},
            "/enable_camera_bool"       : {"object" : self.ui.enableCameraCheckBox,     "default" : True},
            "/green_is_left"            : {"object" : self.ui.GreenIsLeftCheckBox,     "default" : True},

        }

        self.values={
            "/temporal_filter_range"    : {"object" : self.ui.temporalFilterSlider,     "default" : 3},
            "/spatial_filter_range"     : {"object" : self.ui.spatialFilterSlider,      "default" : 5},
            "/lidar_min_angle_deg"      : {"object" : self.ui.lidarMinAngleSlider,      "default" : -90},
            "/lidar_max_angle_deg"      : {"object" : self.ui.lidarMaxAngleSlider,      "default" : 90},
            "/lidar_rmax"               : {"object" : self.ui.lidarDisplayLimSpinBox,   "default" : 10},
            "/simulation_max_speed"     : {"object" : self.ui.simulationMaxSpeedSlider, "default" : 28},
            "/simulation_max_angle"     : {"object" : self.ui.simulationMaxAngleSlider, "default" : 25},
            "/gain_vitesse"             : {"object" : self.ui.gainVitesseSpinBox,       "default" : 0.33},
            "/gain_direction"           : {"object" : self.ui.gainDirectionSpinBox,     "default" : 0.8},
            "/gain_direcition_arg_max"  : {"object" : self.ui.gainDirectionArgMaxSpinBox,"default" : 0.8},
            "/threshold_front_too_close"    : {"object" : self.ui.FrontTooCloseSpinBox,     "default" : 0.1},
            "/threshold_front_far_enough"   : {"object" : self.ui.FrontFarEnoughSpinBox,    "default" : 0.5},
            "/threshold_rear_too_close"     : {"object" : self.ui.RearTooCloseSpinBox,      "default" : 0.2},
        }

        self.calibrate = {
            "red" : {
                "Rmin"  : {"object" : self.ui.redMinRThresholdSpinBox, "default" : 150},
                "Gmin"  : {"object" : self.ui.redMinGThresholdSpinBox, "default" : 0},
                "Bmin"  : {"object" : self.ui.redMinBThresholdSpinBox, "default" : 0},
                "Rmax"  : {"object" : self.ui.redMaxRThresholdSpinBox, "default" : 255},
                "Gmax"  : {"object" : self.ui.redMaxGThresholdSpinBox, "default" : 100},
                "Bmax"  : {"object" : self.ui.redMaxBThresholdSpinBox, "default" : 100},
            },
            "green" : {
                "Rmin"  : {"object" : self.ui.greenMinRThresholdSpinBox, "default" : 0},
                "Gmin"  : {"object" : self.ui.greenMinGThresholdSpinBox, "default" : 150},
                "Bmin"  : {"object" : self.ui.greenMinBThresholdSpinBox, "default" : 0},
                "Rmax"  : {"object" : self.ui.greenMaxRThresholdSpinBox, "default" : 100},
                "Gmax"  : {"object" : self.ui.greenMaxGThresholdSpinBox, "default" : 255},
                "Bmax"  : {"object" : self.ui.greenMaxBThresholdSpinBox, "default" : 100},
            }
        }

        self.connect_sliders_and_double_spin_boxes()

        self.changement_alert = rospy.Publisher('/param_change_alert', Bool, queue_size = 10)

        self.connect()
        self.set_parameters()
        self.get_params_names()

        self.ui.loadParamPushButton.clicked.connect(self.load_parameters)
        self.ui.saveParamPushButton.clicked.connect(self.save_parameters)

        self.msg_alert = Bool()
        self.msg_alert.data = True

        self.init_calibration = rospy.Publisher("/do_an_auto_calibration", Bool, queue_size = 10)
        self.ui.redAutoThresholdPushButton.clicked.connect(lambda : self.auto_calibration(color = "red"))
        self.ui.greenAutoThresholdPushButton.clicked.connect(lambda : self.auto_calibration(color = "green"))


    def connect(self):

        for name,info in self.values.items() :
            info["object"].valueChanged.connect(lambda value, key=name: self.change_param(value,key=key))

        for name,info in self.checkbox.items() :
            info["object"].toggled.connect(lambda value, key=name: self.change_param(bool(value),key=key))

        for color in self.calibrate:
            for name, info in self.calibrate[color].items() :
                info["object"].valueChanged.connect(lambda value, color=color, key=name: self.set_color_threshold(value,color,key=key))



    def auto_calibration(self, color = "no_one") :
        rospy.set_param("/color_to_calibrate", color)
        self.color_to_set = color
        self.subscriber_calibration = rospy.Subscriber("/is_auto_calibration_done", Bool, self.update_spinboxes_calibration)
        self.init_calibration.publish(self.msg_alert)
        
    def update_spinboxes_calibration(self, value = True) :
        color = self.color_to_set
        thresholds = rospy.get_param(f"/{color}_threshold")

        self.calibrate[color]["Bmin"]["default"], self.calibrate[color]["Gmin"]["default"], self.calibrate[color]["Rmin"]["default"] = thresholds[0]
        self.calibrate[color]["Bmax"]["default"], self.calibrate[color]["Gmax"]["default"], self.calibrate[color]["Rmax"]["default"] = thresholds[1]
        
        for name, info in self.calibrate[color].items() :
            self.calibrate[color][name]["object"].setValue(info["default"])

        if value == True : 
            self.subscriber_calibration.unregister()

    def set_color_threshold(self, value, color, key = "no key"):
        self.calibrate[color][key]["default"] = value
        c = EasyDict(self.calibrate[color])
        rospy.set_param(f"/{color}_threshold",[[c.Bmin.default,c.Gmin.default,c.Rmin.default],[c.Bmax.default, c.Gmax.default, c.Rmax.default]])
        self.changement_alert.publish(self.msg_alert)


    def connect_sliders_and_double_spin_boxes(self) :
        self.ui.lidarDisplayLimSlider.valueChanged.connect(lambda value: self.ui.lidarDisplayLimSpinBox.setValue(value /1000))
        self.ui.lidarDisplayLimSpinBox.valueChanged.connect(lambda value: self.ui.lidarDisplayLimSlider.setValue(int(value *1000)))

        self.ui.gainVitesseSlider.valueChanged.connect(lambda value: self.ui.gainVitesseSpinBox.setValue(value /100))
        self.ui.gainVitesseSpinBox.valueChanged.connect(lambda value: self.ui.gainVitesseSlider.setValue(int(value *100)))
        
        self.ui.gainDirectionSlider.valueChanged.connect(lambda value: self.ui.gainDirectionSpinBox.setValue(value /100))
        self.ui.gainDirectionSpinBox.valueChanged.connect(lambda value: self.ui.gainDirectionSlider.setValue(int(value *100)))

        self.ui.gainDirectionArgMaxSlider.valueChanged.connect(lambda value: self.ui.gainDirectionArgMaxSpinBox.setValue(value /100))
        self.ui.gainDirectionArgMaxSpinBox.valueChanged.connect(lambda value: self.ui.gainDirectionArgMaxSlider.setValue(int(value *100)))

        self.ui.FDMSlider.valueChanged.connect(lambda value: self.ui.FDMSpinBox.setValue(value /100))
        self.ui.FDMSpinBox.valueChanged.connect(lambda value: self.ui.FDMSlider.setValue(int(value *100)))

        self.ui.FDSlider.valueChanged.connect(lambda value: self.ui.FDSpinBox.setValue(value /100))
        self.ui.FDSpinBox.valueChanged.connect(lambda value: self.ui.FDSlider.setValue(int(value *100)))

        self.ui.BDMSlider.valueChanged.connect(lambda value: self.ui.BDMSpinBox.setValue(value /100))
        self.ui.BDMSpinBox.valueChanged.connect(lambda value: self.ui.BDMSlider.setValue(int(value *100)))

    def set_parameters(self):
        for name,info in self.values.items() :
            rospy.set_param(name, info["default"])
            info["object"].setValue(info["default"])

        for name,info in self.checkbox.items() :
            rospy.set_param(name, info["default"])
            info["object"].setChecked(info["default"])

    def change_param(self, value, key = None):
        if key in self.checkbox :
            self.checkbox[key]["default"] = value

        if key in self.values :

            # Début de code pour empêcher d'avoir un seuil inférieur à l'autre
            # if key in ["/seuil_FD", "/seuil_FDM"] :
            #     if self.checkbox["/seuil_FD"]["default"] - self.checkbox["/seuil_FDM"] > - 0.05 :
            #         self.checkbox["/seuil_FD"]["default"] =

            self.values[key]["default"] = value

        rospy.set_param(key, value)
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

        """
        if "thresholds" in to_load :
            self.thresholds_color = to_load["thresholds"]
            for color in ["red", "green"]:
                rospy.set_param(f"/{color}_threshold", self.thresholds_color["red"])
                self.color_to_set = color
                self.set_thresholds(value = False)
        """
        self.set_parameters()


    def get_params_names(self) :

        self.ui.loadParamComboBox.clear()
        param_folder = self.dir + "/params"
        param_files = [f for f in os.listdir(param_folder) if os.path.isfile(os.path.join(param_folder, f))]
        self.ui.loadParamComboBox.addItems(param_files)

    def save_parameters(self):

        checkbox_defaults = {key: value["default"] for key, value in self.checkbox.items()}
        values_defaults = {key: value["default"] for key, value in self.values.items()}


        to_save = {
            "checkbox"  : checkbox_defaults,
            "values"    : values_defaults,

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