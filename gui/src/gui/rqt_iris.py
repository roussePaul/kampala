import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission
from sml_setup import Arming_Quad

import os
import subprocess

class MyPlugin(Plugin):
    
    def __init__(self, context):
        self.pwd = os.environ['PWD']

        self.simulation = rospy.get_param('/simulation','false')

        self.land_permission = Permission()
        self.lander_channel = []

        
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.ConnectButton.clicked.connect(self.Connect)
        self._widget.LANDButton.clicked.connect(self.Land)
        self._widget.ArmButton.clicked.connect(self.Arm)
        self._widget.StartButton.clicked.connect(self.Start)
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3'])

    def Connect(self):
        self.name = self._widget.IrisInputBox.currentText()
        inputstring = "source "+self.pwd+"/devel/setup.bash; roscd scenarios/launch/iris; roslaunch %s.launch simulation:=%s;sleep 10" % (self.name,self.simulation)
        subprocess.Popen(["gnome-terminal","-x","bash","-c", inputstring])


        self.lander_channel = rospy.Publisher('/%s/security_guard/lander'%(self.name),Permission,queue_size=10)
        print self.lander_channel


    def Land(self):
        self.land_permission.permission = True
        self.lander_channel.publish(self.land_permission)

    def Start(self):
        inputstring = "source "+self.pwd+"/devel/setup.bash; roslaunch scenarios %s.launch ns:=%s;sleep 10" % (self._widget.StartInputField.text(),self.name)
        subprocess.Popen(["gnome-terminal","-x","bash","-c", inputstring])

    def Arm(self):
        inputstring = "source "+self.pwd+"/devel/setup.bash; roslaunch scenarios iris_nodes.launch ns:=%s;sleep 10" % (self.name)
        subprocess.Popen(["gnome-terminal","-x","bash","-c", inputstring])


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

    