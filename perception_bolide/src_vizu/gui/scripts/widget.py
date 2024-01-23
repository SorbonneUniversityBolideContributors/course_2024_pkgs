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
            "/enable_camera_bool"       : {"object" : self.ui.enableCameraCheckBox,     "default" : True}
        }

        self.values={
            "/temporal_filter_range"    : {"object" : self.ui.temporalFilterSlider,     "default" : 3},
            "/spatial_filter_range"     : {"object" : self.ui.spatialFilterSlider,      "default" : 5},
            "/lidar_min_angle_deg"      : {"object" : self.ui.lidarMinAngleSlider,      "default" : -90},
            "/lidar_max_angle_deg"      : {"object" : self.ui.lidarMaxAngleSlider,      "default" : 90},
            "/lidar_rmax"               : {"object" : self.ui.lidarDisplayLimSpinBox,    "default" : 10},
            "/simulation_max_speed"     : {"object" : self.ui.simulationMaxSpeedSlider, "default" : 28},
            "/simulation_max_angle"     : {"object" : self.ui.simulationMaxAngleSlider, "default" : 25},
            "/gain_vitesse"             : {"object" : self.ui.gainVitesseSpinBox, "default" : 0.33},
            "/gain_direction"           : {"object" : self.ui.gainDirectionSpinBox, "default" : 0.8},
        }

        self.connect_sliders_and_double_spin_boxes()

        self.connect()
        self.set_parameters()
        self.get_params_names()

        self.ui.loadParamPushButton.clicked.connect(self.load_parameters)
        self.ui.saveParamPushButton.clicked.connect(self.save_parameters)

        self.changement_alert = rospy.Publisher('/param_change_alert', Bool, queue_size = 10)
        self.msg_alert = Bool()
        self.msg_alert.data = True

        self.connect_calibration()
        self.ui.redAutoThresholdPushButton.clicked.connect(lambda : self.auto_calibration(color = "red"))
        self.ui.greenAutoThresholdPushButton.clicked.connect(lambda : self.auto_calibration(color = "green"))


    def connect(self):

        for name,info in self.values.items() :
            info["object"].valueChanged.connect(lambda value, key=name: self.change_param(value,key=key))

        for name,info in self.checkbox.items() :
            info["object"].toggled.connect(lambda value, key=name: self.change_param(bool(value),key=key))


    def auto_calibration(self, color = "no_one") :
        rospy.set_param("/color_to_calibrate", color)

        self.subscriber_calibration = rospy.Subscriber("/is_auto_calibration_done", Bool, self.set_thresholds)

        commande = "rosrun perception_bolide calibrate_color.py"
        subprocess.Popen(commande, shell = True)

    def set_thresholds(self, value) :
        try : 
            red_thresholds = rospy.get_param("/red_threshold")
            Rmin, Gmin, Bmin = red_thresholds[0]
            Rmax, Gmax, Bmax = red_thresholds[1]

            self.ui.redMinBThresholdSpinBox.setValue(Bmin)
            self.ui.redMinGThresholdSpinBox.setValue(Gmin)
            self.ui.redMinRThresholdSpinBox.setValue(Rmin)
            self.ui.redMaxBThresholdSpinBox.setValue(Bmax)
            self.ui.redMaxGThresholdSpinBox.setValue(Gmax)
            self.ui.redMaxRThresholdSpinBox.setValue(Rmax)
        except : pass

        try : 
            green_thresholds = rospy.get_param("/green_threshold")
            Rmin, Gmin, Bmin = green_thresholds[0]
            Rmax, Gmax, Bmax = green_thresholds[1]

            self.ui.greenMinBThresholdSpinBox.setValue(Bmin)
            self.ui.greenMinGThresholdSpinBox.setValue(Gmin)
            self.ui.greenMinRThresholdSpinBox.setValue(Rmin)   
            self.ui.greenMaxBThresholdSpinBox.setValue(Bmax)
            self.ui.greenMaxGThresholdSpinBox.setValue(Gmax)
            self.ui.greenMaxRThresholdSpinBox.setValue(Rmax)
        except : pass

        self.subscriber_calibration.unregister()
        rospy.spin()


    def connect_calibration(self):
        self.ui.redMaxBThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.redMaxGThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.redMaxRThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.greenMaxBThresholdSpinBox.valueChanged.connect(self.set_green_threshold)
        self.ui.greenMaxGThresholdSpinBox.valueChanged.connect(self.set_green_threshold)
        self.ui.greenMaxRThresholdSpinBox.valueChanged.connect(self.set_green_threshold)
        self.ui.redMinBThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.redMinGThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.redMinRThresholdSpinBox.valueChanged.connect(self.set_red_threshold)
        self.ui.greenMinBThresholdSpinBox.valueChanged.connect(self.set_green_threshold)
        self.ui.greenMinGThresholdSpinBox.valueChanged.connect(self.set_green_threshold)
        self.ui.greenMinRThresholdSpinBox.valueChanged.connect(self.set_green_threshold)


    def set_red_threshold(self):
        Rmin, Gmin, Bmin = (
            self.ui.redMinRThresholdSpinBox.value(),
            self.ui.redMinBThresholdSpinBox.value(),
            self.ui.redMinBThresholdSpinBox.value()
        )

        Rmax, Gmax, Bmax = (
            self.ui.redMaxRThresholdSpinBox.value(),
            self.ui.redMaxBThresholdSpinBox.value(),
            self.ui.redMaxBThresholdSpinBox.value()
        )

        rospy.set_param("/red_threshold", [[Bmin,Gmin,Rmin],[Bmax, Gmax, Rmax]])
        self.changement_alert.publish(self.msg_alert)


    def set_green_threshold(self):
        Rmin, Gmin, Bmin = (
            self.ui.greenMinRThresholdSpinBox.value(),
            self.ui.greenMinBThresholdSpinBox.value(),
            self.ui.greenMinBThresholdSpinBox.value()
        )

        Rmax, Gmax, Bmax = (
            self.ui.greenMaxRThresholdSpinBox.value(),
            self.ui.greenMaxBThresholdSpinBox.value(),
            self.ui.greenMaxBThresholdSpinBox.value()
        )

        rospy.set_param("/green_threshold", [[Bmin,Gmin,Rmin],[Bmax, Gmax, Rmax]])
        self.changement_alert.publish(self.msg_alert)

    def connect_sliders_and_double_spin_boxes(self) :
        self.ui.lidarDisplayLimSlider.valueChanged.connect(lambda value: self.ui.lidarDisplayLimSpinBox.setValue(value /1000))
        self.ui.lidarDisplayLimSpinBox.valueChanged.connect(lambda value: self.ui.lidarDisplayLimSlider.setValue(int(value *1000)))

        self.ui.gainVitesseSlider.valueChanged.connect(lambda value: self.ui.gainVitesseSpinBox.setValue(value /100))
        self.ui.gainVitesseSpinBox.valueChanged.connect(lambda value: self.ui.gainVitesseSlider.setValue(int(value *100)))
        
        self.ui.gainDirectionSlider.valueChanged.connect(lambda value: self.ui.gainDirectionSpinBox.setValue(value /100))
        self.ui.gainDirectionSpinBox.valueChanged.connect(lambda value: self.ui.gainDirectionSlider.setValue(int(value *100)))

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