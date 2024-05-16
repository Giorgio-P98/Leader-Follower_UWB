import numpy as np
import matplotlib.pyplot as plt
import os
import datetime
from datetime import datetime

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from transforms3d.euler import quat2euler
from mymsg_msgs.msg import Uwb

class SensorSim(Node):

    def __init__(self):
        super().__init__('Sensor_sim')
        
        # Create subscription.
        self.gazebo_states = self.create_subscription(
            LinkStates, '/gazebo_topic/link_states', self.get_value_callback, 10)
        self.gazebo_states  # prevent unused variable warning
        
        # Create publisher.
        self.range_aoa_publisher = self.create_publisher(
            Uwb, '/my_topic/range_aoa', 10)

        # takeoff height and following distance
        self.takeoff_height = -2.5
        self.following_distance = 2.5
        
        # sensor measurement noises
        self.range_std = 0*0.02                        # range simulated measure standard deviation
        self.aoa_std = 0*3.0*np.pi/180                 # aoa simulated measure standard deviation
        
        # save initialize lists for plot purposes
        self.target_pos_all = []
        self.drone_pos_all = []
        self.drone_yaw_all = []
        self.range_all = []
        self.theta_all = []
        self.step_all = []
        
        # step time
        self.dt = 0.1                               # time step duration
        self.step = 0.0
        
        # Timer for the plot method
        self.timer = self.create_timer(60, self.plot)
        
        # Plot Path in the home directory
        HOME = os.path.expanduser( '~' )
        self.PLOT_PATH = HOME+'/UAV_Follower_Plot/'
        
        # Create the PLOT_PATH directory
        if not(os.path.isdir(self.PLOT_PATH)):
            os.makedirs(self.PLOT_PATH)     
            
        # Target trajectory type str
        traj_type = 1                               # 0 = square, 1 = sin-like   
        if traj_type == 0:
            self.traj_str = 'square'  
        elif traj_type == 1:
            self.traj_str = 'sin-like'


    def get_value_callback(self, msg):
        
        # Extract drone and target position and drone orientation frome the LinkStates gazebo msg
        drone_position = np.array([msg.pose[2].position.x, msg.pose[2].position.y, msg.pose[2].position.z])
        target_position = np.array([msg.pose[11].position.x, msg.pose[11].position.y, msg.pose[11].position.z])
        drone_orientation = np.array(quat2euler((msg.pose[2].orientation.w, msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z)))
        
        # Calculate range and angle between drone and target, using their positions
        range = np.linalg.norm((target_position[0:2] - drone_position[0:2]))
        theta = np.arctan2(target_position[1]-drone_position[1], target_position[0]-drone_position[0])
        
        # Yaw angle is the rotations about z axis, so the third component from quat2euler drone_orientation
        yaw_g = drone_orientation[2]
        
        # rotazione oraria negativa, antioraria positiva
        aoa = theta - yaw_g
        
        # Publish the simulated sensor msg adding some noise
        uwb = Uwb()
        uwb.range = range + np.random.normal(0.0, self.range_std)
        uwb.aoa = aoa + np.random.normal(0.0, self.aoa_std)
        self.range_aoa_publisher.publish(uwb)
        
        # Save target-drone position, drone yaw, range and theta of the step for plot purposes
        self.target_pos_all.append(target_position)
        self.drone_pos_all.append(drone_position)
        self.drone_yaw_all.append(yaw_g)
        self.range_all.append(uwb.range)
        self.theta_all.append(uwb.aoa)
        
        # save time instants
        self.step_all.append(self.step)
        self.step += self.dt
        
    def plot(self):
        
        # convert list to array
        target_pos = np.array(self.target_pos_all)
        drone_pos = np.array(self.drone_pos_all)
        time = np.array(self.step_all)
        yaw = np.array(self.drone_yaw_all)
        range = np.array(self.range_all)
        theta = np.array(self.theta_all)
        
        # Get the current date time
        current_datetime = str(datetime.now().replace(microsecond=0))
        folder = self.PLOT_PATH+current_datetime+"__range std: "+str(self.range_std)+", aoa std: "+str(self.aoa_std)+"__traj: "+self.traj_str
        os.mkdir(folder)
        
        plt.figure(1)
        plt.plot(drone_pos[:,0], drone_pos[:,1], label='Drone')
        plt.plot(target_pos[:,0], target_pos[:,1], label='Target')
        plt.xlabel('x Position [m]')
        plt.ylabel('y Position [m]')
        plt.legend()
        plt.title('Drone & Target x-y position',fontweight='bold')
        plt.savefig(folder+"/Drone_Target x-y Position with range std: "+str(self.range_std)+", aoa std: "+str(self.aoa_std)+".png", dpi=500)
        
        plt.figure(2)
        plt.plot(time, drone_pos[:,2])
        plt.axhline(y=-self.takeoff_height, color='r', linestyle='--', linewidth=2)
        plt.ylabel('Height [m]')
        plt.xlabel('Time [s]')
        plt.title('Drone height',fontweight='bold')
        
        
        plt.figure(3)
        plt.plot(time, yaw)
        plt.ylabel('yaw [rad]')
        plt.xlabel('Time [s]')
        plt.title('Drone yaw',fontweight='bold')
        plt.savefig(folder+"/Drone_Yaw with range std: "+str(self.range_std)+", aoa std: "+str(self.aoa_std)+".png", dpi=500)
        
        mean_error = np.mean(abs(range[340:440] - self.following_distance))
        plt.figure(4)
        plt.plot(time, range)
        plt.axhline(y=self.following_distance, color='r', linestyle='--', linewidth=2)
        plt.ylabel('range [m]')
        plt.xlabel('Time [s]')
        plt.title('Range between drone and target, mean error= '+str(mean_error),fontweight='bold') 
        plt.savefig(folder+"/Range measure with range std: "+str(self.range_std)+", aoa std: "+str(self.aoa_std)+".png", dpi=500)

        plt.figure(5)
        plt.plot(time, theta)
        plt.ylabel('theta [rad]')
        plt.xlabel('Time [s]')
        plt.title('angle between drone and target',fontweight='bold')  
        plt.savefig(folder+"/AoA measure with range std: "+str(self.range_std)+", aoa std: "+str(self.aoa_std)+".png", dpi=500)
        
        plt.show()
        plt.close()



def main(args=None):
    rclpy.init(args=args)

    Sensorsim = SensorSim()
    rclpy.spin(Sensorsim)
    Sensorsim.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)