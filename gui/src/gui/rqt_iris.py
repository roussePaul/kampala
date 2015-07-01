import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission


class MyPlugin(Plugin):
    
    def __init__(self, context):
        lander_channel = rospy.Publisher('security_guard/lander',Permission,queue_size=10)
        land_permission = Permission()
        os.system("gnome-terminal -x bash -c 'cd ~/catkin_ws_px4/src/kampala/gui/scripts;./term-pipe-r.sh pipefile;bash'")
       
        def Connect():
            inputstring = "cd ~/catkin_ws_px4/src/kampala/gui/scripts; echo 'source ~/catkin_ws_px4/setup.bash; roslaunch scenarios %s.launch' > pipefile" % (self._widget.IrisInputBox.currentText())
            os.system(inputstring)

        def Land():
            
            land_permission.permission = True
            lander_channel.publish(land_permission)

        def Start():
            inputstring = "cd ~/catkin_ws_px4/src/kampala/gui/scripts; echo 'source ~/catkin_ws_px4/setup.bash; roslaunch scenarios %s.launch' > pipefile" % (self._widget.StartInputField.text())
            os.system(inputstring)

        def Arm():
            inputstring = "cd ~/catkin_ws_px4/src/kampala/gui/scripts; echo python -c 'from sml_setup import Arming_Quad;Arming_Quad()' > pipefile"
            os.system(inputstring)

        


        
        
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

        self._widget.ConnectButton.clicked.connect(Connect)
        self._widget.LANDButton.clicked.connect(Land)
        self._widget.ArmButton.clicked.connect(Arm)
        self._widget.StartButton.clicked.connect(Start)
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3'])


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

    
