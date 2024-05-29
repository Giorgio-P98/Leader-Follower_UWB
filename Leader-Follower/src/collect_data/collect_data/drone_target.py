import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mymsg_msgs.msg import Uwb

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition,  VehicleAttitude

from transforms3d.euler import quat2euler

import os
from datetime import datetime
from matplotlib import pyplot as plt

class drone_target(Node):

    def __init__(self):
        super().__init__('Save_data')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # subscribe to Mocap topic
        self.Drone_subscriber  = self.create_subscription(PoseStamped, '/Drone/pose', self.drone_callback, 10)
        self.Target_subscriber = self.create_subscription(PoseStamped, '/Target/pose', self.target_callback, 10)
        
        # subscribe to UWB topic
        self.Uwb_range_aoa_subscriber = self.create_subscription(Uwb, '/my_topic/range_aoa', self.uwb_ra_callback, 10)
        
        # subscribe to Flight Controller topic
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        # Define message type
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_local_position = VehicleLocalPosition()
        self.Drone = PoseStamped()
        self.Target = PoseStamped()
        self.Uwb_ra = Uwb()
        
        
        # Create vector
        self.all_dronepos = []
        self.all_targetpos = []
        self.all_range = []
        self.all_angle = []
        self.all_yaw_estim = []
        self.all_dronepos_estim = []
        self.time = []
        
        self.Uwb_range = []
        self.Uwb_aoa = []
        
        #some constant
        self.takeoff_height = -0.4
        self.following_distance = 1.5
        
        # timer to get values
        self.dt = 0.1
        self.step = 0.0
        self.timer = self.create_timer(self.dt, self.get_value_callback)
        
        # Save data timer
        self.save_data = self.create_timer(200, self.get_data_callback)
        
        # save data PATH
        HOME = os.path.expanduser( '~' )
        self.DATA_PATH = HOME+'/UAV_Follower_realdata/'
        
        # Create the DATA_PATH directory
        if not(os.path.isdir(self.DATA_PATH)):
            os.makedirs(self.DATA_PATH)
        

    def drone_callback(self, drone):
        """Callback function for dronetopic subscriber."""
        self.Drone = drone
        
    def target_callback(self, target):
        """Callback function for targete topic subscriber."""
        self.Target = target
        
    def uwb_ra_callback(self, Uwb):
        """Callback function for Uwb topic subscriber."""
        self.Uwb_ra = Uwb
        
    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
    def get_value_callback(self):
        Dronepos = np.array([self.Drone.pose.position.x, self.Drone.pose.position.y, self.Drone.pose.position.z])
        Targetpos = np.array([self.Target.pose.position.x, self.Target.pose.position.y, self.Target.pose.position.z])
        
        Droneorientation = np.array(quat2euler(np.array([self.Drone.pose.orientation.w, self.Drone.pose.orientation.x, self.Drone.pose.orientation.y, self.Drone.pose.orientation.z])))
        droneyaw = Droneorientation[2]
        
        rangee = np.linalg.norm(Dronepos[:2] - Targetpos[:2])
        angle = (np.arctan2(Targetpos[1]-Dronepos[1], Targetpos[0]-Dronepos[0]) - droneyaw)*180/np.pi 
        
        # Take the estimated yaw and x,y drone position
        drone_orientation_estimate = np.array(quat2euler(self.vehicle_attitude.q))
        yaw_estim = drone_orientation_estimate[2]
        
        Dronepos_estim = np.array([self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z])
        
        # save Drone-Target positions range and angle
        self.all_dronepos.append(Dronepos)
        self.all_targetpos.append(Targetpos)
        self.all_range.append(rangee)
        self.all_angle.append(angle)
        self.time.append(self.step)
        
        self.Uwb_range.append(self.Uwb_ra.range)
        self.Uwb_aoa.append(self.Uwb_ra.aoa*180/np.pi)
        
        self.all_yaw_estim.append(yaw_estim)
        self.all_dronepos_estim.append(Dronepos_estim)
        
        # Increase step time
        self.step += self.dt
        
        self.get_logger().info('Time {0} s'.format(self.step))
        
    def get_data_callback(self):
        all_dronepos = np.array(self.all_dronepos)
        all_targetpos = np.array(self.all_targetpos)
        all_range = np.array(self.all_range)
        all_angle = np.array(self.all_angle)
        time = np.array(self.time)
        
        Uwb_range = np.array(self.Uwb_range)
        Uwb_aoa = np.array(self.Uwb_aoa)
        
        yaw_estim = np.array(self.all_yaw_estim)
        dronepos_estim = np.array(self.all_dronepos_estim)
        
        # Get the current date time
        current_datetime = str(datetime.now().replace(microsecond=0))
        folder = self.DATA_PATH+current_datetime
        os.mkdir(folder)
        
        # save the data as CSV
        np.savetxt(folder+'/drone_xy_pos.csv', all_dronepos ,delimiter=',')
        np.savetxt(folder+'/target_xy_pos.csv', all_targetpos ,delimiter=',')
        np.savetxt(folder+'/ranges.csv', all_range ,delimiter=',')
        np.savetxt(folder+'/angles.csv', all_angle ,delimiter=',')
        np.savetxt(folder+'/time_vec.csv', time ,delimiter=',')
        
        np.savetxt(folder+'/Uwb_range.csv', Uwb_range ,delimiter=',')
        np.savetxt(folder+'/Uwb_aoa.csv', Uwb_aoa ,delimiter=',')
        
        np.savetxt(folder+'/yaw_estim.csv', yaw_estim,delimiter=',')
        np.savetxt(folder+'/drone_xy_pos_estim.csv', dronepos_estim,delimiter=',')
        
        
        plt.figure(1)
        plt.plot(all_dronepos[:,0], all_dronepos[:,1], label='Drone')
        plt.plot(all_targetpos[:,0], all_targetpos[:,1], label='Target')
        plt.xlabel('x Position [m]')
        plt.ylabel('y Position [m]')
        plt.legend()
        plt.title('Drone & Target x-y position',fontweight='bold')
        
        plt.figure(2)
        plt.plot(time, all_dronepos[:,2])
        plt.axhline(y=-self.takeoff_height, color='r', linestyle='--', linewidth=2)
        plt.ylabel('Height [m]')
        plt.xlabel('Time [s]')
        plt.title('Drone height',fontweight='bold')
        
        plt.figure(3)
        plt.plot(time, all_range, label='Mocap')
        plt.plot(time, Uwb_range, label='Uwb')
        plt.axhline(y=self.following_distance, color='r', linestyle='--', linewidth=2)
        plt.ylabel('range [m]')
        plt.xlabel('Time [s]')
        plt.legend()
        plt.title('Range between drone and target',fontweight='bold')

        plt.figure(4)
        plt.plot(time, all_angle, label='Mocap')
        plt.plot(time, Uwb_aoa, label='Uwb')
        plt.ylabel('theta [deg]')
        plt.xlabel('Time [s]')
        plt.legend()
        plt.title('angle between drone and target',fontweight='bold')
        
        plt.show()
        plt.close()
        
        
        
        



        

def main(args=None):
    rclpy.init(args=args)

    dronetarget = drone_target()

    rclpy.spin(dronetarget)
    
    
    dronetarget.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
