#!/usr/bin/env python

# Node that provide a service that proviede a trigger called at the beginning of experiments
# it store the initial time of the beginning of experiments

import rospy
from analysis.srv import Trigger


class TriggerNode:
    def __init__():
        rospy.init_node('trigger')

        self.filename = rospy.get_param('logs_filename')
        self.replaceTimestamp = rospy.get_param('logs_replace_timestamp',True)

        self.experiment_timestamp_offset = rospy.get_rostime()
        self.experiment_timestamp_given = False

        rospy.Service('trigger_experiment', Trigger, self.trigger)

        rospy.on_shutdown(myhook)
        rospy.spin()

    def close_cb():
        rospy.logerr('['+rospy.get_node_uri()+'] Experimental time not triggered.')
        file_handle = open(self.filename, 'w')
        file_handle.write(str(self.experiment_timestamp_offset))
        file_handle.close()

    def trigger(self, msg):
        if self.experiment_timestamp_given == False:
            self.experiment_timestamp_offset = rospy.get_rostime()
        else:
            rospy.logerr('['+rospy.get_node_uri()+'] Experimental time already triggered. Not overridding the previous value.')


if __name__ == '__main__':
    TriggerNode()
