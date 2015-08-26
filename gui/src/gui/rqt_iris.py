# A GUI Plugin used to load parameters for the PID-controller, and connecting to, arming, starting and landing a quadcopter.

import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from controller.msg import Permission
from std_srvs.srv import Empty
from std_msgs.msg import Int32
from gazebo_msgs.srv import DeleteModel

import utils

import os
import subprocess

class MyPlugin(Plugin):
    
    def __init__(self, context):

        # Getting the Path to the current Working Directory.
        self.pwd = os.environ['PWD']

        # Getting a list of all files in the scenarios directory.
        self.filelist = os.listdir(self.pwd+'/src/kampala/scenarios/launch')
        
        # Checking whether it is a simulation or a flight with a real quad.
        self.simulation = rospy.get_param('/simulation','false')

        # Preallocating instance variables
        self.land_pub = []
        self.controller_channel = []
        self.land_permission = Permission()
        self.controller_permission = Permission()
        self.lander_channel = []
        self.name = ''


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

        
        # Connecting slots to signals

        self._widget.ConnectButton.clicked.connect(self.Connect)
        self._widget.LANDButton.clicked.connect(self.Land)
        self._widget.ArmButton.clicked.connect(self.Arm)
        self._widget.StartButton.clicked.connect(self.Start)
        self._widget.ParamButton.clicked.connect(self.Param)
        self._widget.TerminalButton.clicked.connect(self.Terminal)
        self._widget.StartInputField.returnPressed.connect(self.Autocomplete)
        self._widget.FileInputBox.currentIndexChanged.connect(self.FillIn)
   
        # Adding the quadcopters' names and the filenames to their respective lists in the GUI   
        self._widget.IrisInputBox.insertItems(0,['iris1','iris2','iris3','iris4','iris5'])
        self._widget.FileInputBox.insertItems(0,self.filelist)

        # The terminate button is only enabled for simulations.
        if self.simulation:
            self._widget.TerminateButton.clicked.connect(self.Terminate)
        else:
            self._widget.TerminateButton.setEnabled(False)

    def Terminal(self):
        # Opens a new terminal and runs the script term-pipe-r.sh in that terminal. This script creates a pipefile named pipefileirisx 
        # if irisx is the current quadcopter selected, reads from it and executes any commands it finds there in the new terminal.
        self.name = self._widget.IrisInputBox.currentText()
        subprocess.Popen(["gnome-terminal", "-x" , "bash", "-c", 'source '+self.pwd+'/devel/setup.bash;roscd gui/scripts;./term-pipe-r.sh pipefile' + self.name + ';bash'])

    def execute(self,cmd):
        # Takes a command and writes it to the pipefile of the currently selected quadcopter.
        subprocess.Popen(["bash","-c","cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name])

    def executeBlocking(self,cmd):
        # Takes a command and writes it to the pipefile of the currently selected quadcopter and blocks the GUI until it is done 
        # with this.
        os.system("bash -c 'cd "+self.pwd+"/src/kampala/gui/scripts; echo "+cmd+" > pipefile" + self.name + "'")

    def Terminate(self):
        # Kills all nodes associated with the currently selected quadcopter. Tries to delete the model of it in the current
        # gazebo simulation.
        inputstring = 'rosnode kill `rosnode list | grep ' + self.name + '`'
        self.execute(inputstring)
        try:
            delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
            delete_model(self.name)
        except rospy.ServiceException as exc:
            utils.loginfo('Failed to delete model ' + str(exc))


    def Param(self):
        # Launches the irisx file containging the PID parameters used for that quadcopter and loads them
        self.name = self._widget.IrisInputBox.currentText()
        inputstring = "roslaunch scenarios %s.launch simulation:=%s" % (self.name,self.simulation)
        self.executeBlocking(inputstring)
        # A sleep for 0.2 seconds that allows the file to be properly launched before you try to load the parameters. 
        rospy.sleep(2.)
        
        try: 
            params_load = rospy.ServiceProxy("/%s/blender/update_parameters"%(self.name), Empty)
            params_load_PID = rospy.ServiceProxy("/%s/PID_controller/update_parameters"%(self.name), Empty)
            obstacle_params_load = rospy.ServiceProxy("/%s/obstacle_avoidance/update_parameters"%(self.name), Empty)
            params_load()
            params_load_PID()
        except rospy.ServiceException as exc:
            utils.loginfo("PID not reachable " + str(exc))


    def Connect(self):
        # Launches the file connect.launch and starts publishers on the lander and controller channels.
        self.rpi = utils.Get_Parameter('/'+self.name+'/rpi','false')            # raspberry pi parameter for the connection (connect.launch)
        inputstring = "roslaunch scenarios connect.launch simulation:=%s ns:=%s rpi:=%s" % (self.simulation,self.name,self.rpi)
        self.execute(inputstring)
        
        self.land_pub = rospy.Publisher('/%s/gui/land'%(self.name),Permission,queue_size=10)
        self.lander_channel = rospy.Publisher('/%s/security_guard/lander'%(self.name),Permission,queue_size=10)
        self.controller_channel = rospy.Publisher('/%s/security_guard/controller'%(self.name),Int32,queue_size=10)
        


    def Land(self):
        # Publishes permission for the drone to land. 
      #self.lander_channel.publish(Permission(True))
      self.land_pub.publish(Permission(True))
      #land = rospy.ServiceProxy("/%s/SecurityGuard/land"%(self.name), Empty)
      #land()
      


    def Start(self):
        # Launches the file whose name is written in the StartInputField
        inputstring = "roslaunch scenarios %s ns:=%s" % (self._widget.StartInputField.text(),self.name)
        self.execute(inputstring)

    def Arm(self):
        # Launches the file iris_nodes.launch and publishes a message to the lander not to land the drone.
        self.land_pub.publish(Permission(False))
        inputstring = "roslaunch scenarios iris_nodes.launch ns:=%s simulation:=%s" % (self.name,self.simulation)
        self.execute(inputstring)

    def Autocomplete(self):
        # Called when the StartInputField is selected and return is pressed. 
        # Autocompletes the filename in the StartInputField if the string already 
        # written there is the start of exactly one filename in the scenarios directory.
        exists = False
        unique = True
        completed_text=""
        text = self._widget.StartInputField.text()
        textlength = len(text)
        for filename in self.filelist:
            if text == filename[0:textlength]:
                exists = True
                if completed_text != "":
                    unique = False
                completed_text = filename
        if exists and unique:
            self._widget.StartInputField.setText(completed_text)

    def FillIn(self):
        # Sets the text in the start input field to the one displayed in the FileInputBox
        self._widget.StartInputField.setText(self._widget.FileInputBox.currentText())

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value("irisindex", self._widget.IrisInputBox.currentIndex())

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        index = instance_settings.value("irisindex",0)
        self._widget.IrisInputBox.setCurrentIndex(int(index))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
