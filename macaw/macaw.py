import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Empty,UInt8

class Macaw(Node):
    """
    ROS2 node to translate from Mavlink to ROS2
    and vice versa
    """

    def __init__(self):
        # ROS groundwork
        self.node_name = 'macaw'
        super().__init__(self.node_name)
        # set up inbound MAVlink subscribers
        self.mav_subscribers = {'HEARTBEAT': self.mav_heartbeat_callback}
        self.mav_messages = list(self.mav_subscribers.keys())
        self.get_logger().info(f'Ready for MAVlink messages: {self.mav_messages}')
        # MAVlink system ID
        self.declare_parameter('mavlink_sysid',1)
        self.sysid = self.get_parameter('mavlink_sysid').value
        self.get_logger().info(f'Will talk to SYSID {self.sysid}')
        # set up outbound ROS publishers
        self.ros_publishers = {}
        self.add_ros_publisher(UInt8,'status')
        # connect MAVlink
        self.declare_parameter('mavlink_connect_str','tcp:127.0.0.1:5760')
        connect_str = self.get_parameter('mavlink_connect_str')
        self.get_logger().info(f'Connecting to {connect_str.value}')
        self.mav = mavutil.mavlink_connection(connect_str.value)
        # timer for inbound MAVlink handling
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # ROS subscribers
        self.add_ros_subscriber(Empty,'arm',self.ros_arm_callback)

    def add_ros_publisher(self,ros_type,topic):
        topic_root = f'macaw/sysid{self.sysid}/'
        new_pub = self.create_publisher(ros_type,topic_root + topic,10)
        self.ros_publishers[topic] = new_pub

    def add_ros_subscriber(self,ros_type,topic,callback):
        topic_root = f'macaw/sysid{self.sysid}/'
        new_sub = self.create_subscription(ros_type,topic_root + topic,callback,10)

    def timer_callback(self):
        # process inbound MAVlink messages
        mav_msg = self.mav.recv_match(type=self.mav_messages, blocking=False)
        if mav_msg:
            mav_msg_type = mav_msg.get_type()
            self.mav_subscribers[mav_msg_type](mav_msg)

    def mav_heartbeat_callback(self,mav_msg):
        ros_msg = UInt8()
        ros_msg.data = mav_msg.system_status
        self.ros_publishers['status'].publish(ros_msg)

    def ros_arm_callback(self,ros_msg):
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0,True,0,0,0,0,0,0)

def main(args=None):
    rclpy.init(args=args)
    macaw = Macaw()
    rclpy.spin(macaw)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    macaw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
