import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty,Bool,UInt8,Float64,String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Point, Vector3
from pymavlink import mavutil

class Macaw(Node):
    """
    ROS2 node to translate from Mavlink to ROS2
    and vice versa
    """

    def __init__(self):
        # ROS groundwork
        self.node_name = 'macaw'
        super().__init__(self.node_name)
        # identify target MAVlink system ID
        self.declare_parameter('mavlink_sysid',1)
        self.sysid = self.get_parameter('mavlink_sysid').value
        self.get_logger().info(f'Will talk to SYSID {self.sysid}')
        # set up outbound ROS publishers
        self.ros_publishers = {}
        self.add_ros_publisher(UInt8,'status')
        self.add_ros_publisher(Bool,'is_armed')
        self.add_ros_publisher(NavSatFix,'gps')
        self.add_ros_publisher(Float64,'altitude_asl')
        self.add_ros_publisher(Point,'local_position')
        self.add_ros_publisher(Vector3,'local_velocity')
        # connect MAVlink
        self.declare_parameter('mavlink_connect_str','tcp:127.0.0.1:5760')
        connect_str = self.get_parameter('mavlink_connect_str')
        self.get_logger().info(f'Connecting to {connect_str.value}')
        self.mav = mavutil.mavlink_connection(connect_str.value)
        # set up inbound MAVlink subscribers
        self.mav_subscribers = {}
        self.add_mav_subscriber('HEARTBEAT', self.mav_heartbeat_callback)
        self.add_mav_subscriber('STATUSTEXT', self.mav_text_callback)
        self.add_mav_subscriber('PARAM_VALUE', self.mav_param_callback)
        self.add_mav_subscriber('GPS_RAW_INT', self.mav_gps_callback, interval=1e6)
        self.add_mav_subscriber('LOCAL_POSITION_NED', self.mav_local_pos_callback, interval=1e6)
        self.get_logger().info(f'Ready for MAVlink messages: {self.mav_subscribers.keys()}')
        # timer for inbound MAVlink handling
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # ROS subscribers
        self.add_ros_subscriber(Empty,'arm',self.ros_arm_callback)
        self.add_ros_subscriber(Float64,'takeoff',self.ros_takeoff_callback)
        self.add_ros_subscriber(Empty,'land',self.ros_land_callback)
        self.add_ros_subscriber(String,'cmd_mode',self.ros_mode_callback)
        self.add_ros_subscriber(Twist,'cmd_vel',self.ros_vel_callback)

    def add_ros_publisher(self,ros_type,topic):
        topic_root = f'macaw/sysid{self.sysid}/'
        new_pub = self.create_publisher(ros_type,topic_root + topic,10)
        self.ros_publishers[topic] = new_pub

    def add_ros_subscriber(self,ros_type,topic,callback):
        topic_root = f'macaw/sysid{self.sysid}/'
        new_sub = self.create_subscription(ros_type,topic_root + topic,callback,10)

    def add_mav_subscriber(self,mav_type,callback,interval=None):
        mav_type_num = getattr(mavutil.mavlink,f'MAVLINK_MSG_ID_{mav_type}')
        self.mav_subscribers[mav_type] = callback
        if interval:
            self.mav.mav.command_long_send(self.sysid,
                                        1,
                                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                        0,
                                        mav_type_num,
                                        1e6,
                                        0,0,0,0,0,0)

    def timer_callback(self):
        # process inbound MAVlink messages
        mav_msg = self.mav.recv_match()
        if mav_msg:
            mav_msg_type = mav_msg.get_type()
            mav_msg_sender = mav_msg.get_srcSystem()
            if mav_msg_type in self.mav_subscribers:
                self.get_logger().info(f'Got {mav_msg_type} from {mav_msg_sender}')
                if mav_msg_sender==self.sysid:
                    self.mav_subscribers[mav_msg_type](mav_msg)
            else:
                self.get_logger().info(f'Ignoring {mav_msg_type} from {mav_msg_sender}')

    def mav_heartbeat_callback(self,mav_msg):
        ros_msg = UInt8()
        ros_msg.data = mav_msg.system_status
        self.ros_publishers['status'].publish(ros_msg)
        ros_msg = Bool()
        if mav_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            ros_msg.data = True
        else:
            ros_msg.data = False
        self.ros_publishers['is_armed'].publish(ros_msg)

    def mav_text_callback(self,mav_msg):
        msg_sender = mav_msg.get_srcSystem()
        self.get_logger().info(f'STATUSTEXT from {msg_sender}: {mav_msg.text}')

    def mav_gps_callback(self,mav_msg):
        ros_msg = NavSatFix()
        ros_msg.latitude = mav_msg.lat/1e7
        ros_msg.longitude = mav_msg.lon/1e7
        ros_msg.altitude = mav_msg.alt_ellipsoid/1e3
        self.ros_publishers['gps'].publish(ros_msg)
        ros_msg = Float64()
        ros_msg.data = mav_msg.alt/1e3
        self.ros_publishers['altitude_asl'].publish(ros_msg)

    def mav_param_callback(self,mav_msg):
        param_name = f'macaw/sysid{self.sysid}/{mav_msg.param_id}'
        param_value = mav_msg.param_value
        self.get_logger().info(f'Setting {param_name} to {param_value}')
        try:
            _ = self.get_parameter(param_name)
        except:
            self.declare_parameter(param_name)
        ros_param = Parameter(param_name,Parameter.Type.DOUBLE,param_value)
        self.set_parameters([ros_param])

    def mav_local_pos_callback(self,mav_msg):
        ros_msg = Point()
        ros_msg.x = mav_msg.x
        ros_msg.y = mav_msg.y
        ros_msg.z = mav_msg.z
        self.ros_publishers['local_position'].publish(ros_msg)
        ros_msg = Vector3()
        ros_msg.x = mav_msg.vx
        ros_msg.y = mav_msg.vy
        ros_msg.z = mav_msg.vz
        self.ros_publishers['local_velocity'].publish(ros_msg)

    def ros_arm_callback(self,ros_msg):
        self.get_logger().info('Arming')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0,True,0,0,0,0,0,0)

    def ros_takeoff_callback(self,ros_msg):
        takeoff_alt = ros_msg.data
        self.get_logger().info(f'Take off to altitude {takeoff_alt}')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                       0,0,0,0,0,0,0,
                                       takeoff_alt)

    def ros_land_callback(self,ros_msg):
        self.get_logger().info('Land')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_NAV_LAND,
                                       0,0,0,0,0,0,0,0)

    def ros_mode_callback(self,ros_msg):
        new_mode_name = ros_msg.data
        if new_mode_name in self.mav.mode_mapping():
            new_mode_num = self.mav.mode_mapping()[new_mode_name]
            self.get_logger().info(f'Changing to {new_mode_name} mode ({new_mode_num})')
            self.mav.mav.set_mode_send(self.sysid,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       new_mode_num)
        else:
            self.get_logger().info(f'Unknown mode {new_mode_name}')

    def ros_vel_callback(self,ros_msg):
        self.mav.mav.set_position_target_global_int_send(
            0, # ms since boot
            self.sysid, 1,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE #|
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), lat_int=0, lon_int=0, alt=0, # (x, y WGS84 frame pos), z [m]
            vx=ros_msg.linear.x,
            vy=ros_msg.linear.y,
            vz=ros_msg.linear.z, # velocities in NED frame [m/s] 
            afx=0, afy=0, afz=0, yaw=0,
            yaw_rate=ros_msg.angular.z
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )

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
