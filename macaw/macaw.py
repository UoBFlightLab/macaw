import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterNotDeclaredException
from std_msgs.msg import Empty, Bool, UInt8, Float64, String, UInt64
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, Twist, Point, Vector3, Quaternion, TransformStamped, Pose
from tf2_ros import TransformBroadcaster
from pymavlink import mavutil
import math
from transforms3d.euler import euler2quat, quat2euler
import threading

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
        self.declare_parameter('mavlink_sysid', 1)
        self.sysid = int(self.get_parameter('mavlink_sysid').value)
        self.get_logger().info(f'Will talk to SYSID {self.sysid}')
        # set up outbound ROS publishers
        self.ros_publishers = {}
        self.add_ros_publisher(UInt8, 'status')
        self.add_ros_publisher(UInt8, 'mode')
        self.add_ros_publisher(UInt64, 'heartbeat_count')
        self.add_ros_publisher(Bool, 'is_armed')
        self.add_ros_publisher(NavSatFix, 'gps_raw')
        self.add_ros_publisher(NavSatFix, 'global_pos')
        self.add_ros_publisher(NavSatFix, 'global_target')
        self.add_ros_publisher(Float64, 'altitude_asl')
        self.add_ros_publisher(Float64, 'altitude_rel_home')
        self.add_ros_publisher(Float64, 'heading')
        self.add_ros_publisher(Point, 'local_position')
        self.add_ros_publisher(Point, 'local_target')
        self.add_ros_publisher(Vector3, 'local_velocity')
        self.add_ros_publisher(Quaternion, 'attitude')
        self.add_ros_publisher(Vector3, 'angular_velocity')
        self.add_ros_publisher(BatteryState, 'battery_state')
        #my new publisher for vision position estimate
        self.add_ros_publisher(PoseStamped, 'vision_position')
        self.tf_broadcaster = TransformBroadcaster(self)
        # connect MAVlink
        self.declare_parameter('mavlink_connect_str', '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00')
        #self.declare_parameter('mavlink_connect_str', 'udpin:127.0.0.1:14540')
        connect_str = self.get_parameter('mavlink_connect_str')
        self.get_logger().info(f'Connecting to {connect_str.value}')
        self.mav = mavutil.mavlink_connection(connect_str.value, 
                                              source_system=253,
                                              source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.get_logger().info(f'Waiting for heartbeat')
        self.mav.wait_heartbeat()
        self.num_heartbeats = 1
        self.get_logger().info(f'Got heartbeat from ID {self.mav.target_system} component {self.mav.target_component}')
        #print(help(self.mav.mav.vision_position_estimate_send))
        # set up inbound MAVlink subscribers
        self.mav_subscribers = {}
        self.last_mav_msgs = {}
        self.add_mav_subscriber('HEARTBEAT', self.mav_heartbeat_callback)
        self.add_mav_subscriber('STATUSTEXT', self.mav_text_callback)
        self.add_mav_subscriber('PARAM_VALUE', self.mav_param_callback)
        self.add_mav_subscriber('GPS_RAW_INT', self.mav_gps_callback, interval=1e6)
        self.add_mav_subscriber('LOCAL_POSITION_NED', self.mav_local_pos_callback, interval=1e6)
        self.add_mav_subscriber('ATTITUDE', self.mav_attitude_callback, interval=1e6)
        self.add_mav_subscriber('COMMAND_ACK', self.mav_cmd_ack_callback)
        self.add_mav_subscriber('SYS_STATUS', self.mav_sys_status_callback, interval=1e6)
        self.add_mav_subscriber('GLOBAL_POSITION_INT', self.mav_global_pos_callback, interval=1e6)
        self.add_mav_subscriber('POSITION_TARGET_GLOBAL_INT', self.mav_global_target_callback, interval=1e6)
        self.add_mav_subscriber('POSITION_TARGET_LOCAL_NED', self.mav_local_target_callback, interval=1e6)
        self.add_ros_subscriber(Pose, 'cmd_pose_local', self.ros_local_pos_callback)
        self.add_ros_subscriber(NavSatFix, 'cmd_pos_global', self.ros_global_pos_callback)
        # --- setpoint streaming for PX4 offboard mode ---
        # PX4 requires SET_POSITION_TARGET_LOCAL_NED at >2Hz continuously
        # or it will exit offboard mode as a failsafe. We decouple the
        # incoming ROS rate from the outgoing mavlink rate using a timer
        # that always resends the last known target.
        self.local_pos_target_lock = threading.Lock()
        self.local_pos_target = None  # dict with x, y, z, yaw, or None until first msg
        setpoint_rate_hz = 20.0
        self.local_pos_setpoint_timer = self.create_timer(
            1.0 / setpoint_rate_hz, self.send_local_pos_setpoint_callback)
        
        self.get_logger().info(f'Ready for MAVlink messages: {self.mav_subscribers.keys()}')
        # timer for inbound MAVlink handling
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # ROS subscribers
        self.add_ros_subscriber(Empty, 'arm', self.ros_arm_callback)
        self.add_ros_subscriber(Float64, 'takeoff', self.ros_takeoff_callback)
        self.add_ros_subscriber(Empty, 'land', self.ros_land_callback)
        self.add_ros_subscriber(String, 'cmd_mode', self.ros_mode_callback)
        self.add_ros_subscriber(Twist, 'cmd_vel', self.ros_body_vel_callback)
        self.add_ros_subscriber(Twist, 'cmd_vel_global', self.ros_global_vel_callback)
        self.add_ros_subscriber(Pose, 'cmd_pose_local', self.ros_local_pos_callback)
        self.add_ros_subscriber(NavSatFix, 'cmd_pos_global', self.ros_global_pos_callback)
        #my new nodes
        self.add_ros_subscriber(PoseStamped, '/vicon/ActiveWand/ActiveWand', self.ros_vision_transform_callback)
        self.add_ros_subscriber(PoseStamped, 'macaw_vision_pose', self.ros_vision_update_callback)

    def add_ros_publisher(self, ros_type, topic):
        topic_root = f'macaw/sysid{self.sysid}/'
        new_pub = self.create_publisher(ros_type,
                                        topic_root + topic,
                                        10)
        self.ros_publishers[topic] = new_pub

# I want to add a node that subscribes to my vicon bridge, and then publishes mavlink messages in a form that the autopilot will
#interpret as being part of its state estimation. It looks like a mavlink message called "VICON_POSITION ESTIMATE" would be sufficient.
# Look at a command that seens position information in reverse, i.e. from mavlink to ros. i.e. mav_local_position_callback.

    def publish_ros(self,topic,msg,msg_type=None):
        if topic not in self.ros_publishers:
            if msg_type==None:
                msg_type = type(msg)
            self.add_ros_publisher(msg_type, topic)
        self.ros_publishers[topic].publish(msg)

    def add_ros_subscriber(self, ros_type, topic, callback):
        if topic[0]=='/':
            topic_root = ""
        else:
            topic_root = f'macaw/sysid{self.sysid}/'
        new_sub = self.create_subscription(ros_type,
                                           topic_root + topic,
                                           callback,
                                           10)
        print(topic_root + topic)
        return new_sub

    def add_mav_subscriber(self, mav_type, callback, interval=None):
        mav_type_num = getattr(mavutil.mavlink, f'MAVLINK_MSG_ID_{mav_type}')
        self.mav_subscribers[mav_type] = callback
        if interval:
            self.mav.mav.command_long_send(self.sysid,
                                           1,
                                           mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                           0,
                                           mav_type_num,
                                           interval,
                                           0, 0, 0, 0, 0, 0)
            self.get_logger().info(f'Requesting {mav_type} (#{mav_type_num}) at interval {interval}')

    def timer_callback(self):
        # process inbound MAVlink messages
        mav_msg = self.mav.recv_match()
        if mav_msg:
            mav_msg_type = mav_msg.get_type()
            mav_msg_sender = mav_msg.get_srcSystem()
            if mav_msg_type in self.mav_subscribers:
                self.get_logger().debug(f'Got {mav_msg_type} from {mav_msg_sender}')
                self.last_mav_msgs[mav_msg_type] = mav_msg
                if mav_msg_sender == self.sysid:
                    self.mav_subscribers[mav_msg_type](mav_msg)
            else:
                self.get_logger().debug(f'Ignoring {mav_msg_type} from {mav_msg_sender}')

    def mav_heartbeat_callback(self, mav_msg):
        # status
        ros_msg = UInt8()
        ros_msg.data = mav_msg.system_status
        self.ros_publishers['status'].publish(ros_msg)
        # armed status
        ros_msg = Bool()
        if mav_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            ros_msg.data = True
        else:
            ros_msg.data = False
        self.ros_publishers['is_armed'].publish(ros_msg)
        # current mode
        ros_msg = UInt8()
        ros_msg.data = mav_msg.custom_mode
        self.ros_publishers['mode'].publish(ros_msg)
        # time stamp
        self.num_heartbeats = self.num_heartbeats + 1
        ros_msg = UInt64()
        ros_msg.data = self.num_heartbeats
        self.ros_publishers['heartbeat_count'].publish(ros_msg)
        #TODO: Find a way to get this to fire once, during initialistion. Will quickly crowd out terminal.
        #self.get_logger().info(f'Ze Autpilot issssss {self.last_mav_msgs["HEARTBEAT"].autopilot}')

    def mav_text_callback(self, mav_msg):
        msg_sender = mav_msg.get_srcSystem()
        self.get_logger().info(f'STATUSTEXT from {msg_sender}: {mav_msg.text}')

    def mav_cmd_ack_callback(self, mav_msg):
        msg_sender = mav_msg.get_srcSystem()
        self.get_logger().info(f'COMMAND_ACK from {msg_sender}: Command was {mav_msg.command} with result {mav_msg.result}.')    

    def mav_gps_callback(self, mav_msg):
        ros_msg = NavSatFix()
        ros_msg.latitude = mav_msg.lat/1e7
        ros_msg.longitude = mav_msg.lon/1e7
        ros_msg.altitude = mav_msg.alt_ellipsoid/1e3
        self.ros_publishers['gps_raw'].publish(ros_msg)

    def mav_global_pos_callback(self, mav_msg):
        ros_msg = NavSatFix()
        ros_msg.latitude = mav_msg.lat/1e7
        ros_msg.longitude = mav_msg.lon/1e7
        ros_msg.altitude = -1.0 # no altitude as WGS84 not provided in this MAVLINK message
        self.ros_publishers['global_pos'].publish(ros_msg)
        ros_msg = Float64()
        ros_msg.data = mav_msg.alt/1e3
        self.ros_publishers['altitude_asl'].publish(ros_msg)
        ros_msg = Float64()
        ros_msg.data = mav_msg.relative_alt/1e3
        self.ros_publishers['altitude_rel_home'].publish(ros_msg)
        ros_msg = Float64()
        ros_msg.data = mav_msg.hdg/1e2
        self.ros_publishers['heading'].publish(ros_msg)

    def mav_global_target_callback(self, mav_msg):
        ros_msg = NavSatFix()
        ros_msg.latitude = mav_msg.lat_int/1e7
        ros_msg.longitude = mav_msg.lon_int/1e7
        ros_msg.altitude = -1.0 # no altitude as WGS84 not provided in this MAVLINK message
        self.ros_publishers['global_target'].publish(ros_msg)

    def mav_param_callback(self, mav_msg):
        param_name = f'macaw/sysid{self.sysid}/{mav_msg.param_id}'
        param_value = mav_msg.param_value
        self.get_logger().info(f'Setting {param_name} to {param_value}')
        try:
            _ = self.get_parameter(param_name)
        except ParameterNotDeclaredException:
            self.declare_parameter(param_name, 0.0)
        ros_param = Parameter(param_name,
                              Parameter.Type.DOUBLE,
                              param_value)
        self.set_parameters([ros_param])

    def mav_local_pos_callback(self, mav_msg):
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
        ros_msg = Float64()
        ros_msg.data = mav_msg.vz*(-0.3048)*60
        self.publish_ros('vertical_speed_fpm',ros_msg)
        self.mav_tf_callback()

    def mav_local_target_callback(self, mav_msg):
        ros_msg = Point()
        ros_msg.x = mav_msg.x
        ros_msg.y = mav_msg.y
        ros_msg.z = mav_msg.z
        self.ros_publishers['local_target'].publish(ros_msg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'home{self.sysid}_ned'
        t.child_frame_id = f'target{self.sysid}_ned'
        t.transform.translation.x = mav_msg.x
        t.transform.translation.y = mav_msg.y
        t.transform.translation.z = mav_msg.z
        q = euler2quat(mav_msg.yaw, 0.0, 0.0, 'rzyx')
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.get_logger().debug('Sending target TF')
        self.tf_broadcaster.sendTransform(t)

    def mav_attitude_callback(self, mav_msg):
        """The attitude in the aeronautical frame
        (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic)."""
        q = euler2quat(mav_msg.yaw, mav_msg.pitch, mav_msg.roll, 'rzyx')
        ros_msg = Quaternion()
        ros_msg.w = q[0]
        ros_msg.x = q[1]
        ros_msg.y = q[2]
        ros_msg.z = q[3]
        self.ros_publishers['attitude'].publish(ros_msg)
        ros_msg = Vector3()
        ros_msg.x = mav_msg.rollspeed
        ros_msg.y = mav_msg.pitchspeed
        ros_msg.z = mav_msg.yawspeed
        self.ros_publishers['angular_velocity'].publish(ros_msg)
        self.mav_tf_callback()

    def mav_tf_callback(self):
        if 'ATTITUDE' not in self.last_mav_msgs:
            return
        if 'LOCAL_POSITION_NED' not in self.last_mav_msgs:
            return
        att = self.last_mav_msgs['ATTITUDE']
        pos = self.last_mav_msgs['LOCAL_POSITION_NED']
        if att.time_boot_ms == pos.time_boot_ms:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f'home{self.sysid}_ned'
            t.child_frame_id = f'macaw{self.sysid}_ned'
            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = pos.z
            q = euler2quat(att.yaw, att.pitch, att.roll, 'rzyx')
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            t.transform.rotation.w = q[0]
            self.get_logger().debug('Sending TF')
            self.tf_broadcaster.sendTransform(t)

    def mav_sys_status_callback(self, mav_msg):
        ros_msg = BatteryState()
        ros_msg.voltage = mav_msg.voltage_battery/1e3
        ros_msg.current = mav_msg.current_battery/1e2
        ros_msg.percentage = mav_msg.battery_remaining/100
        self.ros_publishers['battery_state'].publish(ros_msg)
        
    def ros_arm_callback(self, ros_msg):
        self.get_logger().info('Arming')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, True,
                                       0, 0, 0, 0, 0, 0)

    def ros_takeoff_callback(self, ros_msg):
        takeoff_alt = ros_msg.data
        self.get_logger().info(f'Take off to altitude {takeoff_alt}')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                       0, 0, 0, 0, 0, math.nan, math.nan,
                                       takeoff_alt)

    def ros_land_callback(self, ros_msg):
        self.get_logger().info('Land')
        self.mav.mav.command_long_send(self.sysid,
                                       1,
                                       mavutil.mavlink.MAV_CMD_NAV_LAND,
                                       0, 0, 0, 0, 0, 0, 0, 0)

#TODO: Change this to be agnsostic between PX4 and Ardupilot. Use mavutil to query the software stack during initiation and then use the appropriate command to change modes..
    def ros_mode_callback(self, ros_msg):
        new_mode_name = ros_msg.data
        if new_mode_name in self.mav.mode_mapping():
            new_mode_num = self.mav.mode_mapping()[new_mode_name]
            
            # Safely unpack the tuple (29, 6, 0) into individual floats
            base_mode = float(new_mode_num[0])  # This will be 29.0
            main_mode = float(new_mode_num[1])  # This will be 6.0
            sub_mode  = float(new_mode_num[2])  # This will be 0.0

            self.mav.mav.command_long_send(self.sysid,
                                           1,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                           0,
                                           base_mode,
                                           main_mode,
                                           sub_mode,
                                           0, 0, 0, 0)
        else:
            self.get_logger().info(f'Unknown mode {new_mode_name}')

    def ros_global_vel_callback(self, ros_msg):
        # command velocity in global aligned frame
        self.mav.mav.set_position_target_global_int_send(
            0, # ms since boot
            self.sysid, 1,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except 3D velocity and yaw rate
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

    def ros_global_pos_callback(self, ros_msg):
        # command position in global frame (lat/lon only)
        sea_level_ellipsoid = self.last_mav_msgs['GPS_RAW_INT'].alt_ellipsoid - self.last_mav_msgs['GPS_RAW_INT'].alt
        alt_sl_cmd = ros_msg.altitude - sea_level_ellipsoid
        self.mav.mav.set_position_target_global_int_send(
            0, # ms since boot
            self.sysid, 1,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except latitude and longitude, altitude
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            lat_int=int(1e7*ros_msg.latitude),
            lon_int=int(1e7*ros_msg.longitude),
            alt=alt_sl_cmd, # (x, y WGS84 frame pos), z [m]
            vx=0,
            vy=0,
            vz=0, # velocities in NED frame [m/s]
            afx=0, afy=0, afz=0,
            yaw=0,
            yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )    

    def ros_body_vel_callback(self, ros_msg):
        # command velocity in body aligned frame
        self.mav.mav.set_position_target_local_ned_send(
            0, # ms since boot
            self.sysid, 1,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask=( # ignore everything except 3D velocity and yaw rate
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
            ), x=0, y=0, z=0, # 
            vx=ros_msg.linear.x,
            vy=ros_msg.linear.y,
            vz=ros_msg.linear.z, # velocities in body NED frame [m/s]
            afx=0, afy=0, afz=0, yaw=0,
            yaw_rate=ros_msg.angular.z
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )

    def ros_local_pos_callback(self, ros_msg):
        # command position in NED frame relative to home
        # this only updates the stored target - actual sending happens
        # in send_local_pos_setpoint_callback, on a timer, so that PX4
        # gets a continuous stream regardless of this topic's rate
        yaw, pitch, roll = quat2euler((ros_msg.orientation.w,
                                       ros_msg.orientation.x,
                                       ros_msg.orientation.y,
                                       ros_msg.orientation.z), 'rzyx')
        if roll**2 > 1e-9:
            self.get_logger().info(f'Target roll {roll} too much - ignoring position command')
            return
        if pitch**2 > 1e-9:
            self.get_logger().info(f'Target pitch {pitch} too much - ignoring position command')
            return

        with self.local_pos_target_lock:
            self.local_pos_target = {
                'x': ros_msg.position.x,
                'y': ros_msg.position.y,
                'z': ros_msg.position.z,
                'yaw': yaw,
            }

    def ros_vision_update_callback(self, VisionEstimate):
        self.mav.mav.vision_position_estimate_send(
            int(self.get_clock().now().nanoseconds / 1e3),  # timestamp
            VisionEstimate.x,
            VisionEstimate.y,
            VisionEstimate.z,
            VisionEstimate.yaw,
            VisionEstimate.pitch,
            VisionEstimate.roll,
            covariance=[float('nan')] + [0.0] * 20)

    def ros_vision_transform_callback(self, ros_msg):
        # command position in NED frame relative to home
        # this only updates the stored target - actual sending happens
        # in send_local_pos_setpoint_callback, on a timer, so that PX4
        # gets a continuous stream regardless of this topic's rate
        self.get_logger().info("TRANSFORM CALLBACK FIRED")
        self.get_logger().info(f'the incoming ros msg from vicon bridge is: {ros_msg.pose.orientation}')
        yaw = 0
        pitch = 0
        roll = 0
        yaw, pitch, roll = quat2euler((ros_msg.pose.orientation.x,
                                       ros_msg.pose.orientation.y,
                                       ros_msg.pose.orientation.z,
                                       ros_msg.pose.orientation.w), 'rzyx')
        x = ros_msg.pose.position.x
        y = ros_msg.pose.position.y
        z = ros_msg.pose.position.z

        class VisionEstimate:
            pass

        VisionEstimate.x = x
        VisionEstimate.y = y
        VisionEstimate.z = z
        VisionEstimate.yaw = yaw
        VisionEstimate.pitch = pitch
        VisionEstimate.roll = roll

        self.get_logger().info(f'Received vision position: X Position: {x}, Y Position: {y}, Z Position: {z}, Yaw Attitude: {yaw}, Pitch Attitude: {pitch}, Roll Attitude: {roll}')
        self.ros_vision_update_callback(VisionEstimate)
        

#TODO: This needs to be made angostic of ardupilot or PX4. Use mavutil as a good and query the software stack during itiation.
    def send_local_pos_setpoint_callback(self):
        # fires on a timer, independent of ROS message arrival, to keep
        # PX4 in offboard mode - resends the last valid target each time
        with self.local_pos_target_lock:
            target = self.local_pos_target

        if target is None:
            return  # nothing received yet, don't send garbage

        self.mav.mav.set_position_target_local_ned_send(
            0, # ms since boot
            self.sysid, 1,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=( # ignore everything except 3D position and yaw
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            x=target['x'],
            y=target['y'],
            z=target['z'], # positions in NED
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=target['yaw'],
            yaw_rate=0
        )

##ARDUPILOT COMMANDING LOCAL POSITION - 
            #def ros_local_pos_callback(self, ros_msg):
    #    # command position in NED frame relative to home
    #    yaw,pitch,roll = quat2euler((ros_msg.orientation.w,
    #                                 ros_msg.orientation.x,
    #                                 ros_msg.orientation.y,
    #                                 ros_msg.orientation.z),'rzyx')
    #    if roll**2>1e-9:
    #        self.get_logger().info(f'Target roll {roll} too much - ignoring position command')
    #    elif pitch**2>1e-9:
    #        self.get_logger().info(f'Target pitch {pitch} too much - ignoring position command')
    #    else:
    #        self.mav.mav.set_position_target_local_ned_send(
    #            0, # ms since boot
    #            self.sysid, 1,
    #            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    #            type_mask=( # ignore everything except 3D position and yaw
    #                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    #                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    #                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    #                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
    #                # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    #                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    #            ),
    #            x=ros_msg.position.x, 
    #            y=ros_msg.position.y, 
    #            z=ros_msg.position.z, # positions in NED
    #            vx=0, vy=0, vz=0, # velocities in body NED frame [m/s]
    #            afx=0, afy=0, afz=0,
    #            yaw=yaw,
    #            yaw_rate=0
    #            # accelerations in NED frame [N], yaw, yaw_rate
    #            #  (all not supported yet, ignored in GCS Mavlink)
    #        )
     
     #ARDUPILOT MODE SWITCHING - 
        #def ros_mode_callback(self, ros_msg):
     #   new_mode_name = ros_msg.data
      #  if new_mode_name in self.mav.mode_mapping():
       #     new_mode_num = self.mav.mode_mapping()[new_mode_name]
        #    self.get_logger().info(f'Changing to {new_mode_name} mode ({new_mode_num})')
         #   self.mav.mav.set_mode_send(self.sysid,
          #                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
           #                            new_mode_num)
        #else:
        #    self.get_logger().info(f'Unknown mode {new_mode_name}')

        
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
