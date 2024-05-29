import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class range_aoa(Node):

    def __init__(self):
        super().__init__('R_A')

        self.doubleUWB_subscriber  = self.create_subscription(PoseStamped, '/doubleUWB/pose', self.double_callback, 10)
        
        self.singleUWB_subscriber = self.create_subscription(PoseStamped, '/singleUWB/pose', self.single_callback, 10)
        
        self.singleUWB = PoseStamped()
        self.doubleUWB = PoseStamped()
        
        self.timer = self.create_timer(0.1, self.get_value_callback)
        

    def single_callback(self, singUWB):
        self.singleUWB = singUWB
        
    def double_callback(self, doubUWB):
        self.doubleUWB = doubUWB  
        
    def get_value_callback(self):
        singleUWBpos = np.array([self.singleUWB.pose.position.x, self.singleUWB.pose.position.y, self.singleUWB.pose.position.z])
        doubleUWBpos = np.array([self.doubleUWB.pose.position.x, self.doubleUWB.pose.position.y, self.doubleUWB.pose.position.z])
        
        rangee = np.linalg.norm(doubleUWBpos[:2] - singleUWBpos[:2])*100
        
        angle = np.arctan2(singleUWBpos[1]-doubleUWBpos[1], singleUWBpos[0]-doubleUWBpos[0])*180/np.pi
        
        self.get_logger().info('range: {0}, aoa: {1}'.format(round(rangee,2),round(angle,4)))
        
        # self.get_logger().info('{0}'.format(singleUWBpos))



        

def main(args=None):
    rclpy.init(args=args)

    rangeaoa = range_aoa()

    rclpy.spin(rangeaoa)
    
    
    rangeaoa.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
