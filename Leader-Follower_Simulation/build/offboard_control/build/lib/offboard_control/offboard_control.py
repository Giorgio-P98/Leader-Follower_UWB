import numpy as np
from transforms3d.euler import quat2euler
import matplotlib.pyplot as plt
import os

# ROS IMPORTS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus,  VehicleAttitude
from mymsg_msgs.msg import Uwb
from gazebo_msgs.msg import LinkStates


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

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
        
        # gazebo tag subscriber
        self.gazebo_states = self.create_subscription(
            LinkStates, '/gazebo_topic/link_states', self.link_states_callback, 10)
        self.gazebo_states  # prevent unused variable warning        
        
        # Subscriber to the simulated sensor's data
        self.get_range_aoa = self.create_subscription(
            Uwb, '/my_topic/range_aoa', self.principal_comand_callback, 10)
        
        # Following distance in x-y plane [meters]
        self.following_distance = 1.5
        
        ## INITIALIZATION POSITION CONTROL PID VARS
        self.position_error = 0.0
        self.past_position_error = 0.0
        self.int_error = 0.0
        
        # control gains
        self.kp = 1.5                                # Proportional gain for PID position controller 2.5
        self.kd = 0.3                                # Derivative gain for PID position controller   0.4
        self.ki = 0.6                                # Integral gain for PID position controller     2.0
        
        # Time instant duration            
        self.dt = 0.1             
        self.step = 0.0
        
        # Initialize vectors to collect and plot data
        self.step_all = []
        self.drone_height_px4 = [] 
        self.meas_tag_pos_all = []
        self.real_tag_pos_all = []
        
        # Plot Path in the home directory
        HOME = os.path.expanduser( '~' )
        self.PLOT_PATH = HOME+'/UAV_Follower_Plot/'
        
        # Create the PLOT_PATH directory if it does not already exist
        if not(os.path.isdir(self.PLOT_PATH)):
            os.makedirs(self.PLOT_PATH)  
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.link_states = LinkStates()
        self.takeoff_height = -2.0
        self.takeoff = 0
        
        # yaw zeroing for simulation
        self.yaw_zero = np.pi/2
        
        self.timer = self.create_timer(95, self.plot_px4)

    ## FUNCTIONS DEFINITION
    
    def link_states_callback(self, link_states):
        """ Callback funtion for gazebo Link_states topic subscriber"""
        self.link_states = link_states

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
        msg.yaw = self.yaw_zero
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info("\n\nPublishing position setpoints {[x, y, z]}\n")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), self.takeoff_height]
        msg.velocity = [vx, vy, vz]
        msg.yaw = self.yaw_zero - yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f'Publishing control velocity: {vx, vy, vz} and yaw : {yaw}')
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", float('nan'))
        msg.param2 = params.get("param2", float('nan'))
        msg.param3 = params.get("param3", float('nan'))
        msg.param4 = params.get("param4", float('nan'))
        msg.param5 = params.get("param5", float('nan'))
        msg.param6 = params.get("param6", float('nan'))
        msg.param7 = params.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        
    def principal_comand_callback(self, msg) -> None:
        """Publish the trajectory setpoint."""
        
        # take the initial drone pose in gobal reference frame
        if self.offboard_setpoint_counter == 0:
            self.initial_drone_pose = np.array([self.link_states.pose[2].position.x, self.link_states.pose[2].position.y])
        
        # Calculate drone estimated yaw
        drone_orientation_estimate = np.array(quat2euler(self.vehicle_attitude.q))
        yaw_estim = self.yaw_zero - drone_orientation_estimate[2] 
        
        # Calculating the control velocity and position
        if self.takeoff == 0 and self.vehicle_local_position.z - self.takeoff_height < 0.05:
            self.takeoff = 1
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            
        if self.takeoff == 0:
            self.publish_offboard_control_heartbeat_signal(pos=True)
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        
        if self.takeoff == 1:
            self.publish_offboard_control_heartbeat_signal(vel=True)
            
            ## CONTROL VARS CALCULATION
            
            # control_yaw is the target angle (direction of target movments), calculated using the px4 estimated yaw and the aoa given by the sensor
            control_yaw = yaw_estim + msg.aoa 
            
            # The desired position is expressed by transforming a simple following distance (a.k.a. the desired range) into his x-y TARGET projection using theta. the same is done even with the actual_pos (a.k.a. using the actual range).
            
            range_diff = msg.range - self.following_distance
            self.past_position_error = self.position_error                              # save past position_error for the derivative PD component
            
            # Update Position, Integral and Derivative control component
            self.position_error = range_diff                                            # Position error update
            self.int_error += range_diff * self.dt                                      # Integral error update
            der_pos_error = (self.position_error - self.past_position_error) / self.dt  # Derivative error update
            
            # Planar position PID controller
            control_velocity = self.kp * self.position_error + self.kd * der_pos_error + self.ki * self.int_error
            control_velocity_vec = control_velocity * np.array([np.cos(control_yaw), np.sin(control_yaw)]) 
            
            self.publish_velocity_setpoint(control_velocity_vec[1], control_velocity_vec[0], float('nan'), control_yaw)
        
            
        # Save height values for plot purposes
        local_height = self.vehicle_local_position.z
        self.drone_height_px4.append(- local_height)
    
        # update step
        self.step_all.append(self.step)
        self.step += self.dt
            
        self.offboard_setpoint_counter += 1 
        
        # TRUE POSITION ESTIMATE
        theta = (msg.aoa + yaw_estim)
        meas_tag_pos = msg.range * np.array([np.cos(theta),np.sin(theta)])           
        
        drone_pos = np.array([self.vehicle_local_position.y,self.vehicle_local_position.x]) + self.initial_drone_pose
        # drone_pos = np.array([self.link_states.pose[2].position.x, self.link_states.pose[2].position.y])
        tag_pos = np.array([self.link_states.pose[11].position.x, self.link_states.pose[11].position.y])
        self.meas_tag_pos_all.append(meas_tag_pos + drone_pos)
        self.real_tag_pos_all.append(tag_pos)
    
    def plot_px4(self):
        
        # DRONE HEIGHT
        time = np.array(self.step_all)
        drone_height_px4 = np.array(self.drone_height_px4)
        
        # REAL & MEASURED TAG POS ERROR
        meas_tag_pos = np.array(self.meas_tag_pos_all)
        real_tag_pos = np.array(self.real_tag_pos_all)
        
        # CALCULATE ERRORS IN TARGET POS ESTIMATION
        tag_pos_errx = meas_tag_pos[:,0] - real_tag_pos[:,0]
        tag_pos_erry = meas_tag_pos[:,1] - real_tag_pos[:,1]
        
        # SAVE THE ARRAYS IN CSV FILES
        np.savetxt(self.PLOT_PATH+'target_real_pos_off.csv', real_tag_pos, delimiter=",")
        np.savetxt(self.PLOT_PATH+'target_meas_pos_off.csv', meas_tag_pos, delimiter=",")
        np.savetxt(self.PLOT_PATH+'times_off.csv', time, delimiter=",")
        np.savetxt(self.PLOT_PATH+'drone_height_off.csv', drone_height_px4, delimiter=",")
        
        
        # PLOT AND SAVE FIGURES
        plt.figure(6)
        plt.plot(time, drone_height_px4)
        plt.axhline(y=-self.takeoff_height, color='r', linestyle='--', linewidth=2)
        plt.ylabel('height[m]')
        plt.xlabel('Time [s]')
        # plt.title('Drone height',fontweight='bold')
        plt.savefig(self.PLOT_PATH+'height_from_px4.png', dpi=300)
        
        plt.figure(7,figsize=(10,3))
        plt.plot(time, tag_pos_errx)
        plt.ylabel('x error [m]')
        plt.xlabel('Time [s]')
        # plt.title('Target estimation error in x',fontweight='bold')
        plt.savefig(self.PLOT_PATH+'tag_estimation_errx.png', dpi=300)
        
        plt.figure(8,figsize=(10,3))
        plt.plot(time, tag_pos_erry)
        plt.ylabel('y error [m]')
        plt.xlabel('Time [s]')
        # plt.title('Target estimation error in y',fontweight='bold')
        plt.savefig(self.PLOT_PATH+'tag_estimation_erry.png', dpi=300)
        
        plt.figure(9)
        plt.plot(meas_tag_pos[:,0], meas_tag_pos[:,1], label='Estimated')
        plt.plot(real_tag_pos[:,0], real_tag_pos[:,1], label='True')
        plt.ylabel('y [m]')
        plt.xlabel('x [m]')
        plt.legend()
        # plt.title('Target position estimat vs real position',fontweight='bold')
        plt.savefig(self.PLOT_PATH+'tag_estimation_vs_real.png', dpi=300)
        
        plt.figure(10)
        plt.hist(tag_pos_errx,41)
        # plt.title('x error distribution')
        plt.savefig(self.PLOT_PATH+'tag_estimation_errx_hist.png', dpi=300)
        
        plt.figure(11)
        plt.hist(tag_pos_erry,41)
        # plt.title('y error distribution')
        plt.savefig(self.PLOT_PATH+'tag_estimation_erry_hist.png', dpi=300)
        plt.show()
        plt.close()
        

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