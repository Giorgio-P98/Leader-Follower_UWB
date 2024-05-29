import numpy as np
from transforms3d.euler import quat2euler
from collections import deque


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,  VehicleAttitude
from mymsg_msgs.msg import Uwb


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        # Subscriber to the simulated sensor's data
        self.get_range_aoa = self.create_subscription(
            Uwb, '/my_topic/range_aoa', self.uwb_callback, 10)
        
        
        # Get parameters by ROS
        self.declare_parameter('gains', rclpy.Parameter.Type.DOUBLE_ARRAY)
        
        gains = self.get_parameter('gains').value
        
        # GAINS
        self.kp = gains[0]                              # 2.5 Proportional gain for PID controller
        self.kd = gains[1]                              # 0.4 Derivative gain for PID controller
        self.ki = gains[2]                              # 2.0 Integral gain for PID controller
        
        # # GAINS
        # self.kp = 2.0                                # 2.5 Proportional gain for PID controller
        # self.kd = 0.3                                # 0.4 Derivative gain for PID controller
        # self.ki = 1.0                                # 2.0 Integral gain for PID controller
        
        # self.kp_h = 5.0                              # Proportional gain for PI height controller
        # self.ki_h = 2.0                              # Integral gain for PI heihgt controller
        
        # following distance
        self.following_distance = 1.2                # following distance in x-y plane [meters]
        
        # Time instant duration
        self.dt = 0.1
        
        # array initialization
        self.position_error = 0.0
        self.past_position_error = 0.0
        self.int_error = 0.0
        
        self.control_yaw = 0.0
        
        # int-error boolean
        self.int_err_yes = False
        
        # moving avrg aoa
        self.aoa_moving= deque([], 5)
        
        # check new UWb message vars
        self.past_range = self.following_distance
        self.equal_range_countr = 0

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.Uwb_meas = Uwb()
        self.takeoff_height = -0.4
        self.target_height = -0.45
        self.dr_tag_h = 0.24
        self.takeoff = 0
        
        # Timer for the callback
        self.principl_timer = self.create_timer(self.dt, self.principal_comand_callback)
        

    ## FUNCTIONS DEFINITION

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self, pos=False, vel=False):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = pos
        msg.velocity = vel
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info("Publishing takeoff setpoints {[x, y, z]}")
        
    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), self.takeoff_height]
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing control velocity: {vx, vy, vz} and : {yaw}")
        self.get_logger().info(f"Yaw: {yaw}")
        
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def uwb_callback(self, Uwb):
        """Get Uwb measures"""
        self.Uwb_meas = Uwb

    def principal_comand_callback(self) -> None:
        """Publish the trajectory setpoint."""
        
        # Calculating the control velocity and position
        if self.takeoff == 0 and self.vehicle_local_position.z - self.takeoff_height < 0.1:
            self.takeoff = 1
            self.offboard_setpoint_counter = 0
            self.get_logger().info("ho finito il takeoff")
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter < 11:
            self.publish_offboard_control_heartbeat_signal(pos=True)
            self.offboard_setpoint_counter += 1
            
        if self.takeoff == 0 and self.offboard_setpoint_counter > 10:
            self.publish_offboard_control_heartbeat_signal(pos=True)
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        if self.takeoff == 1:
            self.publish_offboard_control_heartbeat_signal(vel=True)
            drone_orientation_estimate = np.array(quat2euler(self.vehicle_attitude.q))
            yaw_estim = drone_orientation_estimate[2]
            
            
            if abs(self.past_range - self.Uwb_meas.range) < 1e-4:
                self.equal_range_countr += 1
            else:
                self.equal_range_countr = 0
            self.past_range = self.Uwb_meas.range
            
            ## CONTROL VARS CALCULATION
            
            # control_yaw is the target angle (direction of target movments), calculated using the px4 estimated yaw and the aoa given by the sensor
            
            self.aoa_moving.append(self.Uwb_meas.aoa)
            
            if self.offboard_setpoint_counter % 3 == 0:
            	mean_aoa = np.mean(self.aoa_moving)
            	self.control_yaw = yaw_estim + self.Uwb_meas.aoa
            else:
                self.control_yaw = yaw_estim
            
            # update control_yaw value
            #mean_aoa = np.mean(self.aoa_moving)
            #self.control_yaw = yaw_estim + self.Uwb_meas.aoa
            
            # subtract the height component from the range measure, using pitagora
            planar_range = np.sqrt((self.Uwb_meas.range)**2 - (self.vehicle_local_position.z - self.target_height + self.dr_tag_h)**2)
            
            # RANGE DIFFERENCE 
            self.past_position_error = self.position_error                             # save past position_error for the derivative PD component
            self.position_error = planar_range - self.following_distance               # Position error evaluation 
            
            # INTEGRAL CONTROLLER UPDATE
            if not(self.int_err_yes) and abs(self.position_error) <= 0.05:
                self.int_err_yes = True
            
            if self.int_err_yes:
                self.int_error += self.position_error * self.dt
                # self.get_logger().info(f'integral error= {self.int_error}')
            
            # DERIVATIVE CONTROLLER UPDATE
            der_pos_error = (self.position_error - self.past_position_error) / self.dt
            
            # Planar position PID controller
            # control_velocity = self.kp * self.position_error
            control_velocity = self.kp * self.position_error + self.kd * der_pos_error + self.ki * self.int_error
            
            if self.equal_range_countr >= 5 and self.equal_range_countr < 30:
                control_velocity_vec = [0.0,0.0]
                self.control_yaw = yaw_estim
                self.int_error = 0.0
            elif self.equal_range_countr >= 30:
                self.land()
                self.disarm()
            else:
                control_velocity_vec = control_velocity * np.array([np.cos(self.control_yaw), np.sin(self.control_yaw)])
            
            # Pubblish the x-y velocity together with the yaw control (the height is directly controlled using a series of z setpoints)
            self.publish_velocity_setpoint(control_velocity_vec[0], control_velocity_vec[1], float('nan'), self.control_yaw)
            self.offboard_setpoint_counter += 1
            
            
    

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
