# -*- coding: utf-8 --

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import serial
import sys
import os
import traceback

# --- 尝试导入依赖 ---
# 假设这些文件位于 aim2strike/include/
try:
    from aim2strike.include.uart_servo import UartServoManager
    # 注意：data_table 似乎在原始代码中被注释掉了，但 UartServoManager 可能内部需要它
    # 如果 UartServoManager 需要它，请取消下面的注释
    # from aim2strike.include.data_table import *
    print("成功导入 UartServoManager。")
except ImportError as e:
    print(f"导入 UartServoManager 出错: {e}")
    print("请确保 'aim2strike/include/uart_servo.py' 存在且路径正确。")
    sys.exit("退出: 初始化需要 UartServoManager。")

# 假设 gimbal_servo_lib.py 也位于 aim2strike/include/
try:
    from aim2strike.include.gimbal_servo_lib import GimbalServo
    print("成功导入 GimbalServo 库。")
except ImportError as e:
    print(f"导入 GimbalServo 库出错: {e}")
    print("请确保 'aim2strike/include/gimbal_servo_lib.py' 存在且路径正确。")
    sys.exit("退出: 需要 GimbalServo 库。")


# --- PID 控制器类 ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        """
        初始化 PID 控制器。

        参数:
        Kp: 比例增益
        Ki: 积分增益
        Kd: 微分增益
        setpoint: 目标值 (默认是 0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time() # 初始化上一次时间

    def update(self, feedback_value):
        """
        更新 PID 控制器并计算输出。

        参数:
        feedback_value: 当前反馈值

        返回:
        控制器输出
        """
        current_time = time.time()
        dt = current_time - self.last_time
        # 防止 dt 为零导致除零错误
        if dt <= 0:
            dt = 1e-6 # 使用一个很小的时间间隔
        self.last_time = current_time

        # 计算误差
        error = self.setpoint - feedback_value

        # 计算积分部分
        self.integral += error * dt
        # 可以添加积分抗饱和逻辑 (可选)
        # integral_limit = 1.0 # 示例限制值
        # self.integral = max(min(self.integral, integral_limit), -integral_limit)


        # 计算微分部分
        derivative = (error - self.previous_error) / dt

        # 计算 PID 输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 保存误差以备下次计算微分部分
        self.previous_error = error

        return output

# --- ROS 2 节点 ---
class AimNode(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('aim_node')
        self.get_logger().info("瞄准节点 AimNode 初始化开始...")

        # --- 参数声明 ---
        self.declare_parameter('servo_port_name', '/dev/ttyUSB0')
        self.declare_parameter('servo_baudrate', 115200)
        # 舵机ID固定为 yaw=1, pitch=2
        self.declare_parameter('servo_config.yaw.id', 1)
        self.declare_parameter('servo_config.yaw.mid_pos', 2047)
        self.declare_parameter('servo_config.yaw.min_dev', -1400)
        self.declare_parameter('servo_config.yaw.max_dev', 1400)
        self.declare_parameter('servo_config.pitch.id', 2)
        self.declare_parameter('servo_config.pitch.mid_pos', 2047)
        self.declare_parameter('servo_config.pitch.min_dev', -650)
        self.declare_parameter('servo_config.pitch.max_dev', 1024)
        self.declare_parameter('pid_gains.yaw.Kp',20.0) # PID参数可能需要根据实际效果大幅调整
        self.declare_parameter('pid_gains.yaw.Ki', 0.01) # 0.01
        self.declare_parameter('pid_gains.yaw.Kd', 0.5) # 0.3
        self.declare_parameter('pid_gains.pitch.Kp', 15.0)
        self.declare_parameter('pid_gains.pitch.Ki', 0.005)
        self.declare_parameter('pid_gains.pitch.Kd', 0.3)
        self.declare_parameter('target.x', 0.5) # 目标归一化X坐标
        self.declare_parameter('target.y', 0.5) # 目标归一化Y坐标


        # --- 获取参数值 ---
        self.servo_port_name = self.get_parameter('servo_port_name').get_parameter_value().string_value
        self.servo_baudrate = self.get_parameter('servo_baudrate').get_parameter_value().integer_value

        self.yaw_servo_config = {
            'id': self.get_parameter('servo_config.yaw.id').get_parameter_value().integer_value,
            'mid_pos': self.get_parameter('servo_config.yaw.mid_pos').get_parameter_value().integer_value,
            'min_dev': self.get_parameter('servo_config.yaw.min_dev').get_parameter_value().integer_value,
            'max_dev': self.get_parameter('servo_config.yaw.max_dev').get_parameter_value().integer_value
        }
        self.pitch_servo_config = {
            'id': self.get_parameter('servo_config.pitch.id').get_parameter_value().integer_value,
            'mid_pos': self.get_parameter('servo_config.pitch.mid_pos').get_parameter_value().integer_value,
            'min_dev': self.get_parameter('servo_config.pitch.min_dev').get_parameter_value().integer_value,
            'max_dev': self.get_parameter('servo_config.pitch.max_dev').get_parameter_value().integer_value
        }
        self.servo_ids = [self.yaw_servo_config['id'], self.pitch_servo_config['id']]

        self.pid_gains_yaw = {
            'Kp': self.get_parameter('pid_gains.yaw.Kp').get_parameter_value().double_value,
            'Ki': self.get_parameter('pid_gains.yaw.Ki').get_parameter_value().double_value,
            'Kd': self.get_parameter('pid_gains.yaw.Kd').get_parameter_value().double_value
        }
        self.pid_gains_pitch = {
            'Kp': self.get_parameter('pid_gains.pitch.Kp').get_parameter_value().double_value,
            'Ki': self.get_parameter('pid_gains.pitch.Ki').get_parameter_value().double_value,
            'Kd': self.get_parameter('pid_gains.pitch.Kd').get_parameter_value().double_value
        }
        
        self.target_x = self.get_parameter('target.x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target.y').get_parameter_value().double_value

        self.get_logger().info(f"参数加载完成：端口={self.servo_port_name}, 波特率={self.servo_baudrate}")
        self.get_logger().info(f"偏航舵机(ID:{self.yaw_servo_config['id']}) 配置: mid={self.yaw_servo_config['mid_pos']}, range=[{self.yaw_servo_config['min_dev']}, {self.yaw_servo_config['max_dev']}]")
        self.get_logger().info(f"俯仰舵机(ID:{self.pitch_servo_config['id']}) 配置: mid={self.pitch_servo_config['mid_pos']}, range=[{self.pitch_servo_config['min_dev']}, {self.pitch_servo_config['max_dev']}]")
        self.get_logger().info(f"偏航 PID: Kp={self.pid_gains_yaw['Kp']}, Ki={self.pid_gains_yaw['Ki']}, Kd={self.pid_gains_yaw['Kd']}")
        self.get_logger().info(f"俯仰 PID: Kp={self.pid_gains_pitch['Kp']}, Ki={self.pid_gains_pitch['Ki']}, Kd={self.pid_gains_pitch['Kd']}")
        self.get_logger().info(f"目标中心点: X={self.target_x}, Y={self.target_y}")

        # --- 初始化串口和舵机 ---
        self.uart = None
        self.uservo = None
        self.yaw_servo = None
        self.pitch_servo = None
        self.servos_initialized = False

        try:
            self.get_logger().info(f"尝试打开串口 {self.servo_port_name}...")
            self.uart = serial.Serial(port=self.servo_port_name, baudrate=self.servo_baudrate,
                                      parity=serial.PARITY_NONE, stopbits=1,
                                      bytesize=8, timeout=0.1) # 使用 0.1 秒超时

            self.get_logger().info(f"为 ID {self.servo_ids} 初始化 UartServoManager...")
            # 注意：如果 UartServoManager 需要显式扫描或ping，可能需要在这里调用
            self.uservo = UartServoManager(self.uart, servo_id_list=self.servo_ids)

            self.get_logger().info("初始化偏航舵机对象...")
            self.yaw_servo = GimbalServo(
                servo_id=self.yaw_servo_config['id'],
                mid_pos=self.yaw_servo_config['mid_pos'],
                min_dev=self.yaw_servo_config['min_dev'],
                max_dev=self.yaw_servo_config['max_dev'],
                uservo_manager=self.uservo # 传递共享的管理器实例
            )

            self.get_logger().info("初始化俯仰舵机对象...")
            self.pitch_servo = GimbalServo(
                servo_id=self.pitch_servo_config['id'],
                mid_pos=self.pitch_servo_config['mid_pos'],
                min_dev=self.pitch_servo_config['min_dev'],
                max_dev=self.pitch_servo_config['max_dev'],
                uservo_manager=self.uservo # 传递共享的管理器实例
            )

            self.servos_initialized = True
            self.get_logger().info("串口和舵机对象初始化成功。")

        except serial.SerialException as e:
            self.get_logger().error(f"打开串口 {self.servo_port_name} 时发生致命错误: {e}")
            self.get_logger().error("舵机控制将不可用。请检查连接和权限。")
            # 可以在这里决定是否让节点继续运行或退出
            # raise e # 重新抛出异常，让 ROS 系统知道节点启动失败
        except NameError as e:
             self.get_logger().error(f"致命错误: 未找到 UartServoManager 或 GimbalServo 类。请检查导入。({e})")
             self.get_logger().error("舵机控制将不可用。")
             # raise e
        except Exception as e:
            self.get_logger().error(f"在串口/舵机设置期间发生意外错误: {e}")
            self.get_logger().error(traceback.format_exc())
            self.get_logger().error("舵机控制将不可用。")
            # raise e


        # --- 创建订阅者 ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'detect/center',  # 订阅检测到的中心点坐标
            self.center_callback,
            10) # QoS 配置文件，10 是队列大小
        self.get_logger().info(f"已创建 'detect/center' 话题的订阅者。")

        # --- 初始化 PID 控制器 ---
        # 注意：设定点是误差的目标值，对于追踪图像中心，误差目标为0
        # 而PID的输入是 (目标值 - 当前值)，所以PID的设定点是 0
        # 但我们用PID输出直接调整偏差，所以设定点是目标坐标0.5，输入是当前坐标
        self.pid_yaw = PID(self.pid_gains_yaw['Kp'], self.pid_gains_yaw['Ki'], self.pid_gains_yaw['Kd'], setpoint=self.target_x)
        self.pid_pitch = PID(self.pid_gains_pitch['Kp'], self.pid_gains_pitch['Ki'], self.pid_gains_pitch['Kd'], setpoint=self.target_y)
        self.get_logger().info("PID 控制器已初始化。")


        # --- 初始化舵机状态 ---
        self.current_yaw_deviation = 0.0
        self.current_pitch_deviation = 0.0

        if self.servos_initialized:
            self.get_logger().info("正在将舵机设置到初始位置（居中）...")
            try:
                self.yaw_servo.set_deviation(self.current_yaw_deviation)
                time.sleep(0.05) # 短暂延时确保命令发送
                self.pitch_servo.set_deviation(self.current_pitch_deviation)
                time.sleep(0.5) # 等待舵机到达初始位置
                self.get_logger().info("舵机已居中。")
            except Exception as e:
                 self.get_logger().error(f"设置舵机初始位置时出错: {e}")
                 self.get_logger().error(traceback.format_exc())
                 # 即使居中失败，也可能需要继续尝试运行
        else:
            self.get_logger().warn("舵机未成功初始化，无法设置初始位置。")

        self.get_logger().info("AimNode 初始化完成。")


    def center_callback(self, msg):
        """处理接收到的中心点坐标的回调函数"""
        if not self.servos_initialized:
            self.get_logger().warn("舵机未初始化，跳过中心点回调处理。", throttle_duration_sec=5) # 节流日志输出
            return

        # 获取红色物体的归一化中心坐标 (假设 msg.data 是 [x, y])
        if len(msg.data) >= 2:
            normalized_cX, normalized_cY = msg.data[0], msg.data[1]
        else:
            self.get_logger().warn("收到的消息数据格式不正确，需要至少两个元素。", throttle_duration_sec=5)
            return

        # 更新 PID 控制器并计算输出 (输出是偏差的调整量)
        # 注意 PID 类的 update 现在内部处理 dt
        output_yaw = self.pid_yaw.update(normalized_cX)
        output_pitch = self.pid_pitch.update(normalized_cY)

        # 更新舵机目标偏差
        # 注意：PID输出的符号和舵机运动方向的关系需要根据实际情况调整
        # 如果摄像头看到的物体向右移动（normalized_cX 增大），需要偏航舵机也向右转动（假设偏差增大是右转）
        # 如果摄像头看到的物体向下移动（normalized_cY 增大），需要俯仰舵机向下转动（假设偏差减小是下转）
        self.current_yaw_deviation += output_yaw
        self.current_pitch_deviation -= output_pitch # Y轴方向通常是反的，偏差减小对应画面目标向下移动

        # self.get_logger().debug(f"PID 输出: Yaw={output_yaw:.4f}, Pitch={output_pitch:.4f}")
        # self.get_logger().debug(f"目标偏差: Yaw={self.current_yaw_deviation:.2f}, Pitch={self.current_pitch_deviation:.2f}")

        # 设置舵机偏差 (GimbalServo 类内部会进行范围限制)
        try:
            self.yaw_servo.set_deviation(self.current_yaw_deviation)
            # 不需要每次都延时，除非通信非常不稳定
            # time.sleep(0.01)
            self.pitch_servo.set_deviation(self.current_pitch_deviation)
        except Exception as e:
            self.get_logger().error(f"设置舵机偏差时出错: {e}", throttle_duration_sec=5)
            self.get_logger().error(traceback.format_exc(), throttle_duration_sec=5)


        # 打印舵机角度（可选，用于调试）
        # 注意：获取当前实际位置可能需要调用 uservo 的 read_data 方法，GimbalServo 可能没有直接暴露
        # 暂时只打印目标偏差
        # self.get_logger().info(f'目标舵机偏差 - 偏航: {self.current_yaw_deviation:.1f}, 俯仰: {self.current_pitch_deviation:.1f}')


    def cleanup(self):
        """节点关闭前的清理工作"""
        self.get_logger().info("节点清理开始...")
        if self.servos_initialized and self.uart is not None and self.uart.is_open:
            self.get_logger().info("正在居中舵机...")
            try:
                if self.yaw_servo:
                    self.yaw_servo.center() # 使用 GimbalServo 的 center 方法
                    time.sleep(0.1) # 短暂延时
                if self.pitch_servo:
                    self.pitch_servo.center()
                    time.sleep(0.1)
                self.get_logger().info("舵机已发送居中命令。")
            except Exception as e:
                self.get_logger().error(f"关闭时居中舵机出错: {e}")
                self.get_logger().error(traceback.format_exc())
            finally:
                self.get_logger().info("正在关闭串口...")
                self.uart.close()
                self.get_logger().info("串口已关闭。")
        elif self.uart and self.uart.is_open:
             self.get_logger().warn("舵机对象未完全初始化，但串口是打开的。尝试关闭串口...")
             self.uart.close()
             self.get_logger().info("串口已关闭。")
        else:
            self.get_logger().info("串口未打开或未初始化，无需关闭。")
        self.get_logger().info("节点清理完成。")

    def destroy_node(self):
        """重写 destroy_node 以执行清理"""
        self.cleanup()
        super().destroy_node()
        self.get_logger().info("AimNode 已销毁。")

def main(args=None):
    # 初始化 ROS 2 节点
    rclpy.init(args=args)
    aim_node = None # 初始化为 None
    try:
        # 创建 AimNode 节点实例
        aim_node = AimNode()
        # 开始自旋，等待回调函数触发
        rclpy.spin(aim_node)
    except KeyboardInterrupt:
        print("\n检测到键盘中断。")
    except Exception as e:
        if aim_node:
            aim_node.get_logger().fatal(f"节点运行时发生未捕获的异常: {e}")
            aim_node.get_logger().fatal(traceback.format_exc())
        else:
            print(f"节点初始化或运行时发生未捕获的异常: {e}")
            print(traceback.format_exc())
    finally:
        # 销毁节点（如果已成功创建）
        if aim_node:
            # destroy_node 会调用 cleanup
            aim_node.destroy_node()
        # 关闭 ROS 2
        if rclpy.ok():
            rclpy.shutdown()
            print("ROS 2 已关闭。")

if __name__ == '__main__':
    main()