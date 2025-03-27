import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray  # 用于发布相对坐标
from cv_bridge import CvBridge
import cv2
import numpy as np

# 定义find_red_center函数，用于检测红色物体的中心点
def find_red_center(frame):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 定义红色在HSV空间中的范围（分为两部分）
    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    # 创建红色的掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 使用形态学操作（腐蚀和膨胀）去除噪声
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cX, cY = -1, -1  # 初始化中心点坐标
    if contours:
        # 找到面积最大的轮廓
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)  # 计算轮廓的矩
        if M["m00"] != 0:
            # 计算轮廓的质心
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        # 在图像上绘制轮廓和质心
        cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 7, (255, 0, 0), 5)
        cv2.putText(frame, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    # 获取图像的宽高
    height, width = frame.shape[:2]
    # 将质心坐标归一化到[0, 1]范围
    normalized_cX = cX / width * 2 -1 if cX != -1 else -1
    normalized_cY = cY / height * 2 -1 if cY != -1 else -1
    return frame, normalized_cX, normalized_cY

# 定义DetectNode类
class DetectNode(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('detect_node')
        # 创建一个订阅者，订阅名为'camera/image'的topic，接收Image消息
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10)
        # 创建一个发布者，发布名为'detect/center'的topic，发布Float32MultiArray消息
        self.publisher = self.create_publisher(Float32MultiArray, 'detect/center', 10)
        # 初始化cv_bridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 调用find_red_center函数，获取处理后的图像和归一化后的质心坐标
        _, normalized_cX, normalized_cY = find_red_center(frame)
        # 如果检测到红色物体的质心
        if normalized_cX != -1 and normalized_cY != -1:
            # 创建一个Float32MultiArray消息
            msg = Float32MultiArray()
            # 将归一化后的质心坐标填充到消息中
            msg.data = [normalized_cX, normalized_cY]
            # 发布消息
            self.publisher.publish(msg)

def main(args=None):
    # 初始化ROS 2节点
    rclpy.init(args=args)
    # 创建DetectNode节点实例
    detect_node = DetectNode()
    # 开始自旋，等待回调函数触发
    rclpy.spin(detect_node)
    # 销毁节点
    detect_node.destroy_node()
    # 关闭ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()