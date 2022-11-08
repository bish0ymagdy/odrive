import odrive
import rospy
from std_msgs.msg import MultiArrayDimension

odrv0 = odrive.find_any()
odrv0.axis0.motor.config.current_lim = 10
odrv0.axis1.motor.config.current_lim = 10

odrv1 = odrive.find_any()
odrv1.axis0.motor.config.current_lim = 10
odrv1.axis1.motor.config.current_lim = 10

odrv0.axis0.requested_state = "AXIS_STATE_CLOSED_LOOP_CONTROL"
odrv0.axis0.controller.config.control_mode = "CONTROL_MODE_POSITION_CONTROL"
odrv0.axis0.controller.config.input_mode = "INPUT_MODE_TRAP_TRAJ"

odrv0.axis1.requested_state = "AXIS_STATE_CLOSED_LOOP_CONTROL"
odrv0.axis1.controller.config.control_mode = "CONTROL_MODE_POSITION_CONTROL"
odrv0.axis1.controller.config.input_mode = "INPUT_MODE_TRAP_TRAJ"

odrv1.axis0.requested_state = "AXIS_STATE_CLOSED_LOOP_CONTROL"
odrv1.axis0.controller.config.control_mode = "CONTROL_MODE_POSITION_CONTROL"
odrv1.axis0.controller.config.input_mode = "INPUT_MODE_TRAP_TRAJ"

odrv1.axis1.requested_state = "AXIS_STATE_CLOSED_LOOP_CONTROL"
odrv1.axis1.controller.config.control_mode = "CONTROL_MODE_POSITION_CONTROL"
odrv1.axis1.controller.config.input_mode = "INPUT_MODE_TRAP_TRAJ"

odrv0.axis0.trap_traj.config.vel_limit = 10
odrv0.axis0.trap_traj.config.accel_limit = 2
odrv0.axis0.trap_traj.config.decel_limit = 2
odrv0.axis0.controller.config.inertia = 0

odrv0.axis1.trap_traj.config.vel_limit = 10
odrv0.axis1.trap_traj.config.accel_limit = 2
odrv0.axis1.trap_traj.config.decel_limit = 2
odrv0.axis1.controller.config.inertia = 0

odrv1.axis0.trap_traj.config.vel_limit = 10
odrv1.axis0.trap_traj.config.accel_limit = 2
odrv1.axis0.trap_traj.config.decel_limit = 2
odrv1.axis0.controller.config.inertia = 0

odrv1.axis1.trap_traj.config.vel_limit = 10
odrv1.axis1.trap_traj.config.accel_limit = 2
odrv1.axis1.trap_traj.config.decel_limit = 2
odrv1.axis1.controller.config.inertia = 0

def position (data):
    
    odrv0.axis0.controller.input_pos = data.data[0]
    odrv0.axis1.controller.input_pos = data.data[1]
    odrv1.axis0.controller.input_pos = data.data[3]
    odrv1.axis1.controller.input_pos = data.data[4]

def odrive_position():
    rospy.init_node("odrive_position", anonymous=True)
    rospy.Subscriber("speeds", MultiArrayDimension, position)
    rospy.spin()
if __name__ == '__main__':
    odrive_position()