import cv2
import numpy as np
import time
import lcm
import zlib
import pyrealsense2 as rs
import bot_core  # import LCM message types

class RealSensePublisher:
    def __init__(self):
        self.lcm_channel = 'OPENNI_FRAME'  # LCM channel
        self.compress_rgb = True  # RGB compression flag
        self.compress_depth = True  # Depth compression flag
        self.jpeg_quality = 85  # JPEG quality
        self.debug = True  # Debug flag
        
        # Initialize LCM
        self.lcm = lcm.LCM()
        
        # Initialize message containers for RGB and Depth images
        self.rgb_lcm_msg = bot_core.image_t()
        self.depth_lcm_msg = bot_core.image_t()
        self.i = 0

        # Set up RealSense pipeline for RGBD stream
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color and depth streams
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(config)

    def collect_data(self):
        while True:
            frames = self.pipeline.wait_for_frames()

            # Get RGB and Depth frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Convert color_frame to numpy array (RGB format)
            rgb_image = np.asanyarray(color_frame.get_data())
            # Convert depth_frame to numpy array (Depth format)
            depth_image = np.asanyarray(depth_frame.get_data())

            publish_time = int(time.time() * 1000000)  # Current time in microseconds

            if (self.debug):
                print(f"RGB and Depth images acquired at {publish_time}")
            
            self.publish_lcm(publish_time, rgb_image, depth_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

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
                    # rospy.logerr("JPEG compression failed")
                    print("JPEG compression failed")
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

        # Encode the images_t message to bytes before publishing
        try:
            encoded_msg = images_msg.encode()  # This serializes the message to a byte string
            self.lcm.publish(self.lcm_channel, encoded_msg)  # Publish the encoded message
        except Exception as e:
            print(f"Error encoding and publishing message: {e}")

        self.i += 1
        if self.debug:
            # rospy.loginfo(f"Frame {self.i} published")
            print(f"Frame {self.i} published")

    def stop(self):
        # Stop the pipeline when we're done
        self.pipeline.stop()

if __name__ == "__main__":
    try:
        # Initialize the RealSensePublisher
        publisher = RealSensePublisher()

        # Collect RGB and Depth data
        publisher.collect_data()

    except Exception as e:
        print(f"Error in RealSensePublisher: {e}")
    finally:
        publisher.stop()
