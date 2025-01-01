import cv2
import numpy as np
import time
import lcm
import zlib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import message_filters
import bot_core  # import LCM message types

class RealSensePublisher:
    def __init__(self):
        # Initialize ROS parameters
        self.lcm_channel = rospy.get_param('~output_lcm_channel', 'OPENNI_FRAME')  # LCM channel
        self.compress_rgb = rospy.get_param('~compress_rgb', True)  # RGB compression flag
        self.compress_depth = rospy.get_param('~compress_depth', True)  # Depth compression flag
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 85)  # JPEG quality
        self.debug = rospy.get_param('~debug_print_statements', True)  # Debug flag
        
        # Initialize LCM
        self.lcm = lcm.LCM()
        
        # Initialize message containers for RGB and Depth images
        self.rgb_lcm_msg = bot_core.image_t()
        self.depth_lcm_msg = bot_core.image_t()
        self.i = 0
        self.image_buf = bytearray(1024 * 1024)  # Buffer for compressed images
        self.depth_compress_buf = bytearray(1024 * 1024)  # Buffer for depth compression

        # Initialize ROS node and CvBridge
        rospy.init_node("realsense_lcm_publisher")
        self.bridge = CvBridge()

        # Set up RealSense pipeline for RGBD stream
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color and depth streams
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(config)

        # Set up ROS subscribers for RGB and depth topics
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        # rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        # rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

        rospy.loginfo(f"RGB Topic: {self.rgb_topic}")
        rospy.loginfo(f"Depth Topic: {self.depth_topic}")
        # print(self.rgb_topic, self.depth_topic)
        
        # 使用message_filters同步RGB和深度图像
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, rgb_msg, depth_msg):
        """
        Synchronized RGB and depth image callback method.
        """
        try:
            # Convert the ROS Image messages to OpenCV format
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")  # 16-bit depth image
            
            timestamp = int(time.time() * 1000000)  # Current time in microseconds

            if self.debug:
                rospy.loginfo(f"RGB and Depth images acquired at {timestamp}")
            
            # Check RGB encoding (BGR8 assumed)
            if rgb_msg.encoding == 'bgr8':
                pass  # BGR8 encoding, no conversion needed
            else:
                rospy.logerr(f"Unexpected RGB encoding: {rgb_msg.encoding}")
                return

            # Check Depth encoding (16UC1 or 32FC1)
            if depth_msg.encoding == '16UC1':
                pass  # 16-bit depth, no conversion needed
            elif depth_msg.encoding == '32FC1':
                depth = (depth * 1000).astype(np.uint16)  # Convert 32FC1 to 16UC1 if needed
            else:
                rospy.logerr(f"Unexpected Depth encoding: {depth_msg.encoding}")
                return

            # Publish the synchronized data
            self.publish_lcm(timestamp, rgb, depth)
            
        except Exception as e:
            rospy.logerr(f"Error in synchronized callback: {e}")

    def publish_lcm(self, timestamp, rgb, depth):
        # Create RGB LCM message
        if rgb is not None:
            self.rgb_lcm_msg.utime = timestamp
            self.rgb_lcm_msg.width = rgb.shape[1]
            self.rgb_lcm_msg.height = rgb.shape[0]
            self.rgb_lcm_msg.row_stride = rgb.shape[1] * 3
            self.rgb_lcm_msg.nmetadata = 0
            self.rgb_lcm_msg.size = rgb.shape[0] * rgb.shape[1] * 3
            self.rgb_lcm_msg.data = list(rgb.flatten())  # Flatten to a 1D list

            if not self.compress_rgb:
                self.rgb_lcm_msg.pixelformat = 861030210  # PIXEL_FORMAT_BGR
            else:
               # Compress RGB using OpenCV JPEG
                ret, encoded_image = cv2.imencode('.jpg', rgb, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if not ret:
                    rospy.logerr("JPEG compression failed")
                    return
                self.rgb_lcm_msg.data = list(encoded_image.flatten())  # Flatten the encoded image
                self.rgb_lcm_msg.size = len(encoded_image)  # Set the compressed size
                self.rgb_lcm_msg.pixelformat = bot_core.image_t.PIXEL_FORMAT_MJPEG

        # Create Depth LCM message
        if depth is not None:
            self.depth_lcm_msg.utime = timestamp
            self.depth_lcm_msg.width = depth.shape[1]
            self.depth_lcm_msg.height = depth.shape[0]
            self.depth_lcm_msg.row_stride = depth.shape[1] * 2
            self.depth_lcm_msg.nmetadata = 0

            if not self.compress_depth:
                self.depth_lcm_msg.size = depth.shape[0] * depth.shape[1] * 2
                self.depth_lcm_msg.data = list(depth.flatten())  # Flatten to 1D list
                self.depth_lcm_msg.pixelformat = 357  # PIXEL_FORMAT_BE_GRAY16
            else:
                # Compress Depth using zlib
                # uncompressed_size = depth.shape[0] * depth.shape[1] * 2
                # compressed_size = len(self.depth_compress_buf)
                # zlib.compress(depth.tobytes(), level=9, out=self.depth_compress_buf)
                # self.depth_lcm_msg.data = list(self.depth_compress_buf[:compressed_size])
                # self.depth_lcm_msg.size = compressed_size
                compressed_depth = zlib.compress(depth.tobytes(), level=9)
                self.depth_lcm_msg.data = list(compressed_depth)
                self.depth_lcm_msg.size = len(compressed_depth)
                self.depth_lcm_msg.pixelformat = -2  # PIXEL_FORMAT_DEPTH_MM_ZIPPED

        # Create Image message and publish
        images_msg = bot_core.images_t()
        images_msg.utime = self.rgb_lcm_msg.utime
        images_msg.n_images = 2
        images_msg.images = [self.rgb_lcm_msg, self.depth_lcm_msg]
        images_msg.image_types = [0, 4] if not self.compress_depth else [0, 6]  # 4: DEPTH_MM, 6: DEPTH_MM_ZIPPED

        self.lcm.publish(self.lcm_channel, images_msg)

        self.i += 1
        if self.debug:
            rospy.loginfo(f"Frame {self.i} published")

    def stop(self):
        # Stop the pipeline when we're done
        self.pipeline.stop()

if __name__ == "__main__":
    try:
        # Initialize the RealSensePublisher
        publisher = RealSensePublisher()
        
        # Spin ROS to keep listening for messages
        rospy.spin()

    except Exception as e:
        rospy.logerr(f"Error in RealSensePublisher: {e}")
    finally:
        publisher.stop()
