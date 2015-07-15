import os
import rospy
import QtGui

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from pid.srv import GetPIDParameters, SetPIDParameters,Autotune
from autotuner import Autotuner
from identification import Identification

class AutotunerPlugin(Plugin):
    
    def __init__(self, context):
        super(AutotunerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('AutotunerPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'AutotunerPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('AutotunerPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.bUpdateControllerList.clicked.connect(self.UpdateControllerList)
        self._widget.cIdentificationMethod.currentIndexChanged.connect(self.UpdateSynthesisList)

        self._widget.bAutotune.clicked.connect(self.Autotune)

        self.init_identification()

    def UpdateControllerList(self):
        controller_list = Autotuner.get_controller_list()
        self._widget.cPID.clear()
        self._widget.cPID.insertItems(0,controller_list)

    def init_identification(self):
        identification_list = Identification.method_list.keys()
        self._widget.cIdentificationMethod.insertItems(0,identification_list)

    def UpdateSynthesisList(self):
        identification_method = self._widget.cIdentificationMethod.currentText()
        synthesis_list = Identification.method_list[identification_method]
        self._widget.cSynthesisMethod.clear()
        self._widget.cSynthesisMethod.insertItems(0,synthesis_list)

    def Autotune(self):
        identification_method = self._widget.cIdentificationMethod.currentText()
        synthesis_method = self._widget.cSynthesisMethod.currentText()

        path = self._widget.cIdentificationMethod.currentText() +"/autotune"
        rospy.wait_for_service(path)
        autotune = rospy.ServiceProxy('autotune', Autotune)

        autotune(identification_method,synthesis_method)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
