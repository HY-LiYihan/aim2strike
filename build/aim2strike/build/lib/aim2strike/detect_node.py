import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray  # 用于发布相对坐标
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 定义find_red_center函数，用于检测红色物体的中心点
def find_red_center(frame):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # # 定义红色在HSV空间中的范围（分为两部分）
    lower_red1 = np.array([0, 70, 40])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 40])
    upper_red2 = np.array([180, 255, 255])

    
    
    # 创建红色的掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 使用形态学操作（腐蚀和膨胀）去除噪声
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # --- 修正：在这里初始化返回值 ---
    cX, cY = -1, -1
    normalized_cX = -1.0
    normalized_cY = -1.0
    processed_frame = frame # 默认返回原始帧

    if contours:
        c = max(contours, key=cv2.contourArea)
        # 添加一个最小面积过滤可能更好
        # if cv2.contourArea(c) > 100: # 示例：面积大于100像素才处理
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            height, width = frame.shape[:2]
            if width > 0:
                normalized_cX = float(cX) / width
            if height > 0:
                normalized_cY = float(cY) / height

            # --- 绘制部分移到这里，确保 cX, cY 有效 ---
            # 使用 frame.copy() 避免在原始图像上绘制，如果你不想修改它
            processed_frame = frame # 或者 frame.copy()
            cv2.drawContours(processed_frame, [c], -1, (0, 255, 0), 2)
            cv2.circle(processed_frame, (cX, cY), 5, (0, 0, 255), -1) # 红色实心点
            cv2.putText(processed_frame, f"({normalized_cX:.2f}, {normalized_cY:.2f})", (cX + 10, cY + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 返回处理后的帧和归一化坐标
    # 即使没有找到轮廓，也会返回初始化的 -1.0, -1.0 和原始帧
    return processed_frame, normalized_cX, normalized_cY



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
        self.visualization_publisher = self.create_publisher(Image, 'detect/image_processed', 10)

    def image_callback(self, msg):
        self.get_logger().info("Image callback triggered!", throttle_duration_sec=2.0)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # --- 注意：调用修改后的函数 ---
        processed_frame, normalized_cX, normalized_cY = find_red_center(frame.copy()) # 使用 copy

        # --- 添加日志 ---
        self.get_logger().info(f"find_red_center returned: norm_x={normalized_cX:.3f}, norm_y={normalized_cY:.3f}", throttle_duration_sec=1.0)

        # 如果检测到红色物体的质心 (检查是否 >= 0)
        if normalized_cX >= 0 and normalized_cY >= 0:
            center_msg = Float32MultiArray()
            center_msg.data = [normalized_cX, normalized_cY]
            self.publisher.publish(center_msg)
            self.get_logger().info("Published center data.", throttle_duration_sec=1.0)
        # else: # 可选：如果没找到，也打印日志
        #     self.get_logger().info("Red center not found.", throttle_duration_sec=1.0)

        # --- 发布可视化图像的代码需要移到 try...except 块中防止错误 ---
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_publisher.publish(vis_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert processed frame to Image msg: {e}')
        except Exception as e_pub:
            self.get_logger().error(f'Failed to publish visualization image: {e_pub}')

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