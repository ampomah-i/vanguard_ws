import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from px4_msgs.msg import VehicleOdometry, VehicleCommand, OffboardControlMode, TrajectorySetpoint, Timesync
from threading import Thread
import math
import numpy as np
import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))
from common import coordinate_transforms


#############
#   TO DO   #
#############
# Add QOS 

# Constants
_RATE = 10  # Hz
_MAX_SPEED = 1  # m/s
_MAX_CLIMB_RATE = 0.5  # m/s
_COORDINATE_FRAMES = {'lenu', 'lned', 'bu', 'bd', 'dc', 'fc'}

# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

class TranslationController(Node):
    """
    Controls drone with open-loop translational motion, not rotations.
    """

    def __init__(self, control_reference_frame='bu'):
        super().__init__('translation_controller')
        
        if control_reference_frame not in _COORDINATE_FRAMES:
            raise ValueError("Invalid control reference frame: " + control_reference_frame)

        self.control_reference_frame = control_reference_frame

        # A subscriber to the topic '/fmu/out/vehicle_odometry'. self.pos_sub_cb is called when a message of type 'PoseStamped' is received 
        self.pos_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.pos_sub_cb, 10)
        # Quaternion representing the rotation of the drone's body frame (bu) in the LENU frame. 
        # Initialize to identity quaternion, as bu is aligned with lenu when the drone starts up.
        self.quat_bu_lenu = (0, 0, 0, 1)

        # A subscriber to the topic '/fmu/in/vehicle_command'. self.state_sub_cb is called when a message of type 'VehicleCommand' is received
        self.state_sub = self.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', self.state_sub_cb, 10)
        # Flight mode of the drone ('OFFBOARD', 'POSCTL', 'MANUAL', etc.)
        self.mode = VehicleCommand().mode

        # A publisher which will publish the desired linear and angular velocity to the topic '/fmu/in/offboard_control_mode'
        self.velocity_pub = self.create_publisher(Twist, '/fmu/in/offboard_control_mode', 10)
        # Initialize linear setpoint velocities
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Publishing rate
        self.rate = self.create_rate(_RATE)

        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False

    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def pos_sub_cb(self, msg):
        """
        Updates the orientation the drone (the bu frame) related to the lenu frame
        Args: 
            - msg = ROS VehicleOdometry message
        """
        self.quat_bu_lenu = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])

    def state_sub_cb(self, msg):
        """
        Callback function which is called when a new message of type VehicleCommand is received by self.state_subscriber to update the drone's mode (MANUAL, POSCTL, or OFFBOARD)
        Args:
            - msg = px4_msgs VehicleCommand message
        """
        self.mode = msg.command

    #############
    # STREAMING #
    #############
    def start(self): 
        """
        Start thread to stream velocity commands.
        """
        self.offboard_command_streaming_thread = Thread(target=self.stream_offboard_velocity_setpoints)
        self.offboard_command_streaming_thread.start()

    def stop(self):
        """
        Stop streaming thread.
        """
        self.stopped = True
        try:
            self.offboard_command_streaming_thread.join()
        except AttributeError:
            pass

    def stream_offboard_velocity_setpoints(self):
        """
        Continually publishes Twist commands in the local lenu reference frame. Our desired velocities (in the local frame) are in self.vx, self.vy, self.vz (linear velocity) 
        and self.wx, self.wy, self.wz (rotational velocities around their respective axes)
        """
        # Create twist message for velocity setpoint represented in lenu coordinates
        velsp__lenu = Twist()

        # Continually publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return

            # Set linear velocity (convert command velocity from control_reference_frame to lenu)
            vx, vy, vz = coord_transforms.get_v__lenu((self.vx, self.vy, self.vz), 
                                                      self.control_reference_frame, 
                                                      self.quat_bu_lenu)
            velsp__lenu.linear.x = vx
            velsp__lenu.linear.y = vy
            velsp__lenu.linear.z = vz

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0:
                raise Exception("_MAX_SPEED and _MAX_CLIMB_RATE must be positive")
            velsp__lenu.linear.x = min(max(velsp__lenu.linear.x,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(velsp__lenu.linear.y,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(velsp__lenu.linear.z,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()

    ########
    # WAIT #
    ########
    def wait(self):
        """
        If the drone is not in OFFBOARD mode, loops until the drone is put into OFFBOARD mode. If the drone is in OFFBOARD mode,
        loops until the drone is taken out of OFFBOARD mode.
        """
        # Wait till drone is put into OFFBOARD mode
        if self.mode != 'OFFBOARD':
            self.get_logger().info('Open loop controller: Waiting to enter OFFBOARD mode')
            while self.mode != 'OFFBOARD' and not rclpy.ok():
                self.rate.sleep()
            self.get_logger().info('Open loop controller: {} mode ...'.format(self.mode))
    
        # Wait till drone is taken out of OFFBOARD mode
        else:
            self.get_logger().info('Open loop controller: Waiting to exit OFFBOARD mode')
            while self.mode == 'OFFBOARD' and not rclpy.ok():
                self.rate.sleep()
            self.get_logger().info('Open loop controller: {} mode ...'.format(self.mode))

    ###############
    # TRANSLATION #
    ###############
    def translate(self, displacement, speed):
        """
        Given a displacement vector (dx, dy, dz) and a speed, sets the command velocities so that the drone moves from its current
        location (x, y, z) to the point (x+dx, y+dy, z+dz),
        Args:
            - displacement = (dx, dy, dz) (m)
            - speed = (m/s), must be positive
        """
        # Clip speed at _MAX_SPEED
        speed = min(speed, _MAX_SPEED)
        # Raise error if speed is 0 or not positive
        if speed <= 0:
            raise ValueError("Speed must be positive")
        
        dx, dy, dz = displacement
        # Magnitude of displacement
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        # Amount of time the velocity message will be published for
        move_time = distance/speed

        # Set command velocities (vi = di/time)
        self.vx = dx/move_time
        self.vy = dy/move_time
        self.vz = dz/move_time
        
        self.get_logger().info('Open loop controller: Time of translation: {:.2f}'.format(move_time))
        self.get_logger().info('Open loop controller: Displacement vector: {}'.format(displacement))
        
        # Wait for us to finish publishing velocities
        rclpy.sleep(move_time)

        # Reset command velocities to 0
        self.vx = self.vy = self.vz = 0
        self.get_logger().info('Open loop controller: Done')


def main(args=None):
    rclpy.init(args=args)

    # Create Controller instance
    cframe = 'bu'  # Reference frame commands are given in
    controller = TranslationController(control_reference_frame=cframe)
    # Start streaming setpoint velocities so we can switch into OFFBOARD mode 
    controller.start()
    # Wait to begin executing commands until we switch into OFFBOARD mode
    controller.wait()

    speed = .3
    controller.translate((0,0,-1), speed)
    controller.translate((0.33,0,0.25), speed)
    controller.translate((-0.33,0,0.25), speed)
    controller.translate((0.33,0,0.25), speed)
    controller.translate((-0.33,0,0.25), speed)
    controller.translate((0.66,0,0), speed)
    controller.translate((0.33,0,-1), speed)
    controller.translate((0.33,0,0.5), speed)
    controller.translate((0.33,0,-0.5), speed)
    controller.translate((0.33,0,1), speed)

    controller.wait()
    controller.stop()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
