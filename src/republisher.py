import numpy as np
import lcm
import cv2
import zlib
import jpeg
import time

class ImagePublisher:
    def __init__(self, lcm_channel, compress_rgb=False, compress_depth=False, jpeg_quality=90, debug=False):
        self.lcm = lcm.LCM()
        self.lcm_channel = lcm_channel
        self.compress_rgb = compress_rgb
        self.compress_depth = compress_depth
        self.jpeg_quality = jpeg_quality
        self.debug = debug
        
        # Initialize message containers for RGB and Depth images
        self.rgb_lcm_msg = bot_core.image_t()
        self.depth_lcm_msg = bot_core.image_t()
        self.i = 0
        self.image_buf = bytearray(1024*1024)  # Buffer for compressed images
        self.depth_compress_buf = bytearray(1024*1024)  # Buffer for depth compression
        
    def publish_lcm(self, timestamp, rgb, depth):
        # Create RGB LCM message
        self.rgb_lcm_msg.utime = timestamp
        self.rgb_lcm_msg.width = rgb.shape[1]
        self.rgb_lcm_msg.height = rgb.shape[0]
        self.rgb_lcm_msg.row_stride = rgb.shape[1] * 3
        self.rgb_lcm_msg.nmetadata = 0
        
        # Set pixel format and size for RGB
        self.rgb_lcm_msg.size = rgb.shape[0] * rgb.shape[1] * 3
        self.rgb_lcm_msg.data = list(rgb.flatten())  # Flatten to a 1D list
        
        if not self.compress_rgb:
            self.rgb_lcm_msg.pixelformat = 861030210  # PIXEL_FORMAT_BGR
        else:
            # Compress RGB using JPEG
            compressed_size = len(self.image_buf)
            compression_status = jpeg.compress_8u_rgb(rgb, self.jpeg_quality, self.image_buf, compressed_size)
            if compression_status != 0:
                print("JPEG compression failed...")
            self.rgb_lcm_msg.data = list(self.image_buf[:compressed_size])
            self.rgb_lcm_msg.size = compressed_size
            self.rgb_lcm_msg.pixelformat = bot_core.image_t.PIXEL_FORMAT_MJPEG
        
        # Create Depth LCM message
        self.depth_lcm_msg.utime = self.rgb_lcm_msg.utime
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
            uncompressed_size = depth.shape[0] * depth.shape[1] * 2
            compressed_size = len(self.depth_compress_buf)
            zlib.compress(depth.tobytes(), level=9, out=self.depth_compress_buf)
            self.depth_lcm_msg.data = list(self.depth_compress_buf[:compressed_size])
            self.depth_lcm_msg.size = compressed_size
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
            print(f"Frame {self.i} published")

# Usage Example
if __name__ == "__main__":
    # Initialize the publisher with desired LCM channel
    publisher = ImagePublisher(lcm_channel="image_channel", compress_rgb=True, compress_depth=True, jpeg_quality=85, debug=True)
    
    # Create a dummy RGB and Depth image using OpenCV (You can replace this with your own images)
    rgb = cv2.imread("rgb_image.png")  # Read your RGB image
    depth = cv2.imread("depth_image.png", cv2.IMREAD_UNCHANGED)  # Read your depth image
    
    timestamp = int(time.time() * 1000000)  # Get current timestamp in microseconds
    publisher.publish_lcm(timestamp, rgb, depth)
