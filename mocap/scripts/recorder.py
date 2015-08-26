#!/usr/bin/env python

# A ros node that records the position, velocity and acceleration data published by 
# the ros_mocap node for a certain drone when when it receives a message to do so.
import rospy
import subprocess
import os
import signal
from mocap.msg import Record


class Recorder():

    def __init__(self):

        # Initializing the node with the name 'recorder' and subscribing to the recorder topic.
        rospy.init_node('recorder')
        rospy.Subscriber('/recorder', Record, self.callback)

        # Creating a dictionary that enables seeing which quadcopter has which body_id, using the rosparam '/body_array'
        self.irisdict = dict(zip(['iris1','iris2','iris3','iris4','iris5'],rospy.get_param('/body_array',[1,2,3,4,5])))


        # Preallocating values for the instance variables that will later refer to the different recording subprocesses.

        self.p1state = None
        self.p2state = None
        self.p3state = None
        self.p4state = None
        self.p5state = None

        self.p1input = None
        self.p2input = None
        self.p3input = None
        self.p4input = None
        self.p5input = None

        self.p1target = None
        self.p2target = None 
        self.p3target = None 
        self.p4target = None 
        self.p5target = None

        # Creating the default message from the saver plugin of the GUI 

        self.last_msg = Record()
        self.last_msg.record_iris1 = False
        self.last_msg.record_iris2 = False
        self.last_msg.record_iris3 = False
        self.last_msg.record_iris4 = False
        self.last_msg.record_iris5 = False

        # Getting the Path of the current Working Directory

        self.pwd = os.environ['PWD']

        rospy.spin()
        


    def callback(self,data):
        # Called each time a message from the recorder topic is recieved. That message is compared to the last message recieved. 

        # If the field 'record_irisx' has changed to true in the new message, the recorder will start recording from the rostopics 
        # '/body_data/id_X', '/irisx/trajectory_gen/target' and /irisx/mavros/rc/override to the files irisx_state, irisx_target and
        # irisx_input. Time is appended to the filenames to make them unique. The files will be created in the bagfiles directory.

        # If the'record_irisx' field has changed to false in the new message, the processes recording from each topic corresponding to irisx
        # will be terminated.

        if data.record_iris1 != self.last_msg.record_iris1:
            if data.record_iris1:
                self.p1state = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris1_state'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris1'])])
                self.p1target = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris1_target'+str(rospy.get_time()),'/iris1/trajectory_gen/target'])
                self.p1input = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris1_input'+str(rospy.get_time()),'/iris1/mavros/rc/override'])            
            else:
                self.terminate_process_and_children(self.p1state)
                self.terminate_process_and_children(self.p1target)
                self.terminate_process_and_children(self.p1input)

        if data.record_iris2 != self.last_msg.record_iris2:
            if data.record_iris2:
                self.p2state = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris2_state'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris2'])])
                self.p2target = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris2_target'+str(rospy.get_time()),'/iris2/trajectory_gen/target'])
                self.p2input = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris2_input'+str(rospy.get_time()),'/iris2/mavros/rc/override'])            
            else:
                self.terminate_process_and_children(self.p2state)
                self.terminate_process_and_children(self.p2target)
                self.terminate_process_and_children(self.p2input)

        if data.record_iris3 != self.last_msg.record_iris3:
            if data.record_iris3:
                self.p3state = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris3_state'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris3'])])
                self.p3target = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris3_target'+str(rospy.get_time()),'/iris3/trajectory_gen/target'])
                self.p3input = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris3_input'+str(rospy.get_time()),'/iris3/mavros/rc/override'])            
            else:
                self.terminate_process_and_children(self.p3state)
                self.terminate_process_and_children(self.p3target)
                self.terminate_process_and_children(self.p3input)

        if data.record_iris4 != self.last_msg.record_iris4:
            if data.record_iris4:
                self.p4state = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris4_state'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris4'])])
                self.p4target = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris4_target'+str(rospy.get_time()),'/iris4/trajectory_gen/target'])
                self.p4input = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris4_input'+str(rospy.get_time()),'/iris4/mavros/rc/override'])            
            else:
                self.terminate_process_and_children(self.p4state)
                self.terminate_process_and_children(self.p4target)
                self.terminate_process_and_children(self.p4input)

        if data.record_iris5 != self.last_msg.record_iris5:
            if data.record_iris5:
                self.p5state = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris5_state'+str(rospy.get_time()),'/body_data/id_'+str(self.irisdict['iris5'])])
                self.p5target = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris5_target'+str(rospy.get_time()),'/iris5/trajectory_gen/target'])
                self.p5input = subprocess.Popen(['rosbag','record','-O', self.pwd + '/src/kampala/analysis/bagfiles/iris5_input'+str(rospy.get_time()),'/iris5/mavros/rc/override'])            
            else:
                self.terminate_process_and_children(self.p5state)
                self.terminate_process_and_children(self.p5target)
                self.terminate_process_and_children(self.p5input)


        self.last_msg = data

        # This function is copied from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/. It is
        # used to stop the processes recording with rosbag.
    def terminate_process_and_children(self,p):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
        p.terminate()



        
        
if __name__ == "__main__":
    r = Recorder()

   


    
