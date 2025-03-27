import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)

        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.fourcc = self.get_parameter('fourcc').get_parameter_value().string_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(self.camera_device)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc)) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.publisher = self.create_publisher(Image, 'camera/image', self.fps)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.bridge = CvBridge()

        self.get_logger().info(f"Camera Node Initialized. Device: {self.camera_device}, Width: {self.frame_width}, Height: {self.frame_height}, FPS: {self.fps}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().error("Failed to capture frame from camera")

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()