# Class and function libraries for points.
# (Used by Blende and the controllers.)


class Point():
    def __init__(self):
        self.found_body=True
        self.x=0
        self.y=0
        self.z=0
        self.yaw=0
        self.pitch=0
        self.roll=0
        self.x_vel=0
        self.y_vel=0
        self.z_vel=0
        self.yaw_vel=0
        self.pitch_vel=0
        self.roll_vel=0
        self.x_acc=0
        self.y_acc=0
        self.z_acc=0
        self.yaw_acc=0
        self.pitch_acc=0
        self.roll_acc=0
        self.time_diff=0
        self.first_point_received=False


    def update_point(self,new_data):
        self.found_body=new_data.found_body
        self.x=new_data.x
        self.y=new_data.y
        self.z=new_data.z
        self.yaw=new_data.yaw
        self.pitch=new_data.pitch
        self.roll=new_data.roll
        self.x_vel=new_data.x_vel
        self.y_vel=new_data.y_vel
        self.z_vel=new_data.z_vel
        self.yaw_vel=new_data.yaw_vel
        self.pitch_vel=new_data.pitch_vel
        self.roll_vel=new_data.roll_vel
        self.x_acc=new_data.x_acc
        self.y_acc=new_data.y_acc
        self.z_acc=new_data.z_acc
        self.yaw_acc=new_data.yaw_acc
        self.pitch_acc=new_data.pitch_acc
        self.roll_acc=new_data.roll_acc
        self.time_diff=new_data.time_diff


class PointExt(Point):
    def __init__(self):
        self.found_body=True
        self.x=0
        self.y=0
        self.z=0
        self.yaw=0
        self.pitch=0
        self.roll=0
        self.x_vel=0
        self.y_vel=0
        self.z_vel=0
        self.yaw_vel=0
        self.pitch_vel=0
        self.roll_vel=0
        self.x_acc=0
        self.y_acc=0
        self.z_acc=0
        self.yaw_acc=0
        self.pitch_acc=0
        self.roll_acc=0
        self.time_diff=0
        self.x_jerk=0
        self.y_jerk=0
        self.z_jerk=0
        self.x_snap=0
        self.y_snap=0
        self.z_snap=0
        self.first_point_received=False


    def update_point(self,new_data):
        self.found_body=new_data.found_body
        self.x=new_data.x
        self.y=new_data.y
        self.z=new_data.z
        self.yaw=new_data.yaw
        self.pitch=new_data.pitch
        self.roll=new_data.roll
        self.x_vel=new_data.x_vel
        self.y_vel=new_data.y_vel
        self.z_vel=new_data.z_vel
        self.yaw_vel=new_data.yaw_vel
        self.pitch_vel=new_data.pitch_vel
        self.roll_vel=new_data.roll_vel
        self.x_acc=new_data.x_acc
        self.y_acc=new_data.y_acc
        self.z_acc=new_data.z_acc
        self.yaw_acc=new_data.yaw_acc
        self.pitch_acc=new_data.pitch_acc
        self.roll_acc=new_data.roll_acc
        self.time_diff=new_data.time_diff
        self.x_jerk=new_data.x_jerk
        self.y_jerk=new_data.y_jerk
        self.z_jerk=new_data.z_jerk
        self.x_snap=new_data.x_snap
        self.y_snap=new_data.y_snap
        self.z_snap=new_data.z_snap


class Instruction():
    def __init__(self):
        self.start=False
        self.permission=True


def get_pos_vel_acc(obj):
    x=(obj.x,obj.y,obj.z,obj.yaw)
    x_vel=(obj.x_vel,obj.y_vel,obj.z_vel,obj.yaw_vel)
    x_acc=(obj.x_acc,obj.y_acc,obj.z_acc,obj.yaw_acc)
    return x,x_vel,x_acc