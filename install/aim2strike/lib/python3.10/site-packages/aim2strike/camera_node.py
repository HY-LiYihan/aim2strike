# -*- coding: utf-8 -- # 确保文件编码为 UTF-8

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys
import os
import time
import traceback # 用于打印更详细的错误信息

# --- 关键修改：导入 Hikvision 库 ---
# 假设 hikvision_capture.py 位于 aim2strike 包的某个可导入位置
# 例如，如果它在 'aim2strike/include/' 并且 setup.py 正确配置:
from aim2strike.include.hikvision_capture import HikvisionCamera, HikvisionCameraError
# 或者，如果它在 'aim2strike/src/' 并且 setup.py 正确配置:
# from aim2strike.src.hikvision_capture import HikvisionCamera, HikvisionCameraError
#
# !! 根据你的项目结构调整下面的导入语句 !!
# 如果你的测试脚本是有效的，并且项目根目录是 aim2strike，
# 并且 hikvision_capture.py 在 src/ 子目录下，
# ROS 2 的标准做法是在 setup.py 中声明这个包/模块。
# # 这里我们先假设它可以通过包名直接访问：
# try:
#     from aim2strike.hikvision_capture import HikvisionCamera, HikvisionCameraError
#     print("成功导入 HikvisionCamera 库。")
# except ImportError as e:
#     print(f"导入 HikvisionCamera 库出错: {e}")
#     # 尝试像测试脚本那样添加路径（这在 ROS 节点中通常不推荐，应通过 setup.py 处理）
#     try:
#         project_root = os.path.join(os.path.dirname(__file__).split('install')[0], 'src', 'aim2strike') # 尝试猜测项目源路径
#         src_path = os.path.dirname(project_root) # 退到 src 目录
#         print(f"尝试将 '{src_path}' 添加到 sys.path")
#         sys.path.insert(0, src_path) # 插入到路径开头
#         # 注意：下面的导入现在是相对于 src_path 的
#         from src.hikvision_capture import HikvisionCamera, HikvisionCameraError
#         print("通过修改 sys.path 成功导入 HikvisionCamera 库。")
#         # 恢复 sys.path，避免潜在副作用
#         sys.path.pop(0)
#     except ImportError as e_fallback:
#          print(f"尝试备用导入方法也失败了: {e_fallback}")
#          print("请确保 'hikvision_capture.py' 位于正确的 Python 路径下，并且你的 ROS 2 包 setup.py 配置正确。")
#          sys.exit("退出: 无法加载 HikvisionCamera 库。")
# except Exception as e_generic:
#     print(f"导入 HikvisionCamera 时发生意外错误: {e_generic}")
#     print(traceback.format_exc())
#     sys.exit("退出: 无法加载 HikvisionCamera 库。")


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Hikvision Camera Node 初始化开始...")

        # --- 修改后的参数声明 ---
        self.declare_parameter('hik_device_index', 0) # Hikvision 相机索引
        self.declare_parameter('hik_verbose', False) # 是否启用 Hikvision 库的详细日志
        # 分辨率和 FPS 可能由相机或 SDK 自动确定，或通过 SDK 特定方法设置
        # 这里暂时保留参数声明，但不主动去 set，而是读取实际值
        self.declare_parameter('frame_width', 640) # 期望宽度 (可能无效)
        self.declare_parameter('frame_height', 480) # 期望高度 (可能无效)
        self.declare_parameter('fps', 300) # 期望发布速率 (定时器使用)

        # --- 获取参数值 ---
        self.hik_device_index = self.get_parameter('hik_device_index').get_parameter_value().integer_value
        self.hik_verbose = self.get_parameter('hik_verbose').get_parameter_value().bool_value
        # 注意：下面的参数可能不会直接影响相机设置，主要用于信息和定时器
        self.expected_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.expected_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.publish_fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.cam = None # 初始化相机对象为 None
        self.camera_initialized = False # 标记相机是否成功初始化和启动

        try:
            # --- 关键修改：使用 HikvisionCamera ---
            self.get_logger().info(f"尝试打开 Hikvision 相机，索引: {self.hik_device_index}")
            # 注意：这里不再使用 with 语句，需要在 cleanup 中手动 release
            self.cam = HikvisionCamera(device_index=self.hik_device_index, verbose=self.hik_verbose)

            if not self.cam.is_open():
                # 使用 raise 抛出异常，让外层 try...except 捕获并阻止后续初始化
                raise HikvisionCameraError("无法打开相机。检查连接和 MVS 客户端。")

            # 获取实际分辨率
            actual_width, actual_height = self.cam.get_resolution()
            self.get_logger().info(f"相机成功打开。实际分辨率: {actual_width}x{actual_height}")

            # 启动数据抓取
            self.get_logger().info("尝试启动相机数据抓取...")
            if not self.cam.start():
                # 同样抛出异常
                raise HikvisionCameraError("无法启动数据抓取。")

            self.get_logger().info("相机数据抓取已启动。")
            self.camera_initialized = True # 标记成功

        except HikvisionCameraError as e:
            self.get_logger().error(f"初始化 Hikvision 相机时发生错误: {e}")
            self.get_logger().error("节点将继续运行，但无法发布图像。")
            # 不再 raise，允许节点继续运行，但 timer_callback 会跳过处理
            if self.cam: # 尝试关闭已部分打开的相机
                 try:
                     self.cam.release()
                     self.get_logger().info("已尝试释放相机资源。")
                 except Exception as release_e:
                     self.get_logger().error(f"释放相机资源时出错: {release_e}")
                 self.cam = None # 确保对象为 None
        except Exception as e:
            self.get_logger().fatal(f"初始化过程中发生意外错误: {e}")
            self.get_logger().fatal(traceback.format_exc())
            if self.cam: # 尝试关闭
                 try:
                     self.cam.release()
                 except Exception as release_e:
                     self.get_logger().error(f"释放相机资源时出错: {release_e}")
            self.cam = None
            # 重新抛出异常可能导致节点启动失败，由 main 处理
            # raise e # 取决于是否希望节点在这种情况下退出

        # --- ROS 相关设置 (仅在相机初始化基本成功后进行) ---
        if self.camera_initialized:
            self.publisher = self.create_publisher(Image, 'camera/image', 10) # QoS 设为 10
            self.timer = self.create_timer(1.0 / self.publish_fps, self.timer_callback) # 使用参数指定的发布速率
            self.bridge = CvBridge()
            self.get_logger().info(f"图像将在 'camera/image' 话题上以 ~{self.publish_fps} FPS 发布。")
        else:
            self.get_logger().warn("由于相机初始化失败，图像发布功能未启动。")
            self.publisher = None
            self.timer = None
            self.bridge = None

        self.get_logger().info("Camera Node 初始化完成。")

    def timer_callback(self):
        # 仅在相机成功初始化后才尝试读取
        if not self.camera_initialized or self.cam is None:
            # self.get_logger().warn("相机未初始化，跳过帧捕获。", throttle_duration_sec=10) # 减少日志噪音
            return

        try:
            # --- 关键修改：从 Hikvision 对象读取 ---
            ret, frame = self.cam.read()
            if ret:
                # 将 OpenCV 图像 (BGR) 转换为 ROS Image 消息
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                # 添加时间戳 (可选但推荐)
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame" # 或其他合适的 frame_id
                self.publisher.publish(msg)
            else:
                # read() 返回 False 通常表示超时或其他读取错误
                self.get_logger().warn("未能从 Hikvision 相机读取到帧 (可能超时)。", throttle_duration_sec=5)

        except HikvisionCameraError as e:
             self.get_logger().error(f"读取相机帧时发生 Hikvision 错误: {e}", throttle_duration_sec=5)
             # 这里可以考虑是否需要停止定时器或尝试重新初始化相机
        except Exception as e:
             self.get_logger().error(f"处理相机帧时发生意外错误: {e}", throttle_duration_sec=5)
             self.get_logger().error(traceback.format_exc(), throttle_duration_sec=5)

    def cleanup(self):
        """节点关闭前的清理工作"""
        self.get_logger().info("节点清理开始...")
        # 停止定时器，防止在清理过程中还调用回调
        if self.timer is not None:
            self.timer.cancel()
            self.get_logger().info("定时器已取消。")

        # --- 关键修改：释放 Hikvision 相机资源 ---
        if self.cam is not None:
            self.get_logger().info("正在释放 Hikvision 相机资源...")
            try:
                self.cam.release() # 停止抓取并关闭相机
                self.get_logger().info("Hikvision 相机资源已释放。")
            except HikvisionCameraError as e:
                self.get_logger().error(f"释放 Hikvision 相机时出错: {e}")
            except Exception as e:
                self.get_logger().error(f"释放 Hikvision 相机时发生意外错误: {e}")
                self.get_logger().error(traceback.format_exc())
            finally:
                self.cam = None # 确保对象设为 None
        else:
             self.get_logger().info("相机对象不存在，无需释放。")

        # SDK 的 finalize 放在 main 函数的 finally 中确保执行

        self.get_logger().info("节点清理完成。")

    def destroy_node(self):
        """重写 destroy_node 以执行清理"""
        self.cleanup()
        super().destroy_node()
        self.get_logger().info("CameraNode 已销毁。")

def main(args=None):
    # --- 关键修改：管理 SDK 生命周期 ---
    sdk_initialized = False
    try:
        print("尝试初始化 Hikvision MVS SDK...")
        HikvisionCamera.initialize_sdk()
        sdk_initialized = True
        print("Hikvision MVS SDK 初始化成功。")

        rclpy.init(args=args)
        camera_node = None # 初始化为 None
        try:
            camera_node = CameraNode()
            rclpy.spin(camera_node)
        except KeyboardInterrupt:
            print("\n检测到键盘中断。")
        except Exception as e:
            if camera_node:
                camera_node.get_logger().fatal(f"节点运行时发生未捕获的异常: {e}")
                camera_node.get_logger().fatal(traceback.format_exc())
            else:
                print(f"节点初始化或运行时发生未捕获的异常: {e}")
                print(traceback.format_exc())
        finally:
            if camera_node:
                # destroy_node 会调用 cleanup 来释放相机对象
                camera_node.destroy_node()
            # 关闭 ROS 2
            if rclpy.ok():
                rclpy.shutdown()
                print("ROS 2 已关闭。")

    except HikvisionCameraError as e:
        print(f"初始化 Hikvision SDK 时失败: {e}")
        print("请确保 SDK 已正确安装并配置。")
    except Exception as e_sdk:
         print(f"初始化 Hikvision SDK 时发生意外错误: {e_sdk}")
         print(traceback.format_exc())
    finally:
        # --- 关键修改：确保 SDK 被最终化 ---
        if sdk_initialized:
            try:
                print("正在最终化 Hikvision MVS SDK...")
                HikvisionCamera.finalize_sdk()
                print("Hikvision MVS SDK 已最终化。")
            except HikvisionCameraError as e:
                print(f"最终化 Hikvision SDK 时出错: {e}")
            except Exception as e_finalize:
                print(f"最终化 Hikvision SDK 时发生意外错误: {e_finalize}")
                print(traceback.format_exc())
        else:
             print("SDK 未成功初始化，无需最终化。")

    print("应用程序主函数结束。")


if __name__ == '__main__':
    main()