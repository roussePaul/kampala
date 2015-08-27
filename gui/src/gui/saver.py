# Erik Berglund 2015
# A GUI Plugin that sends messages to the recorder node (code for the recorder node can be found in /mocap/scripts) to start and stop recording data for 
# the quadcopters whose checkboxes gets checked and unchecked.

import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal

from mocap.msg import Record




class saverPlugin(Plugin):
    
    def __init__(self, context):
        super(saverPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('saverPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'saver.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('saverUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Creating a publisher on the record topic and the default message to be published.

        self.pub = rospy.Publisher('/recorder', Record, queue_size=10)
        self.msg = Record()

        # Whether the message fields are set to True of False determines if the recorder node should record
        # data for the corresponding quadcopter. Initally, all checkboxes are unchecked and no data is recorded,
        # so all message fields are set to false.

        self.msg.record_iris1 = False
        self.msg.record_iris2 = False
        self.msg.record_iris3 = False
        self.msg.record_iris4 = False
        self.msg.record_iris5 = False
        
        # Connecting slots to signals

        self._widget.iris1Box.stateChanged.connect(self.send_iris1_msg)
        self._widget.iris2Box.stateChanged.connect(self.send_iris2_msg)
        self._widget.iris3Box.stateChanged.connect(self.send_iris3_msg)
        self._widget.iris4Box.stateChanged.connect(self.send_iris4_msg)
        self._widget.iris5Box.stateChanged.connect(self.send_iris5_msg)

        # The following functions are called whenever a checkbox gets checked or unchecked. They alter the message
        # to be published on the recorder topic in the corresponding way and publishes it.
    def send_iris1_msg(self):

        if self._widget.iris1Box.isChecked():
            self.msg.record_iris1 = True
        else:
            self.msg.record_iris1 = False
        self.pub.publish(self.msg)
        
    def send_iris2_msg(self):

        if self._widget.iris2Box.isChecked():
            self.msg.record_iris2 = True
        else:
            self.msg.record_iris2 = False
        self.pub.publish(self.msg)

    def send_iris3_msg(self):

        if self._widget.iris3Box.isChecked():
            self.msg.record_iris3 = True
        else:
            self.msg.record_iris3 = False
        self.pub.publish(self.msg)

    def send_iris4_msg(self):
        
        if self._widget.iris4Box.isChecked():
            self.msg.record_iris4 = True
        else:
            self.msg.record_iris4 = False
        self.pub.publish(self.msg)

    def send_iris5_msg(self):

        if self._widget.iris5Box.isChecked():
            self.msg.record_iris5 = True
        else:
            self.msg.record_iris5 = False
        self.pub.publish(self.msg)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.pub.unregister()

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

    
