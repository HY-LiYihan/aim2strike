# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import threading
import time
import serial
import sys
import tty # 用於非阻塞鍵盤讀取
import termios # 用於非阻塞鍵盤讀取
from aim2strike.include.uart_servo import UartServoManager
# from rclpy.parameter import Parameter # Parameter 已被導入，但 descriptor 需要從 msg 導入
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterType # 導入 ParameterType 以便使用枚举

# 控制步长 (舵機位置值的變化量，需要根據實際情況調整)
# 考慮到 AimNode 的 angle 是偏移量，這裡的 STEP_SIZE 也應理解為角度偏移量的變化
STEP_SIZE = 50 # 保持不變，但意義明確為角度偏移變化量
# 控制间隔 (秒)
CONTROL_INTERVAL = 0.1 # 保持不變

# --- 移除全域變數 servo_angles, servo_manager, stop_control ---

def get_key():
    """读取用户的单个按键输入 (非阻塞)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno()) # 設置為原始模式，立即讀取單個字符
        ch = sys.stdin.read(1)
        # 處理方向鍵（通常發送3個字符的序列，如 \x1b[A）
        if ch == '\x1b': # ESC - 可能表示方向鍵開始
            next1 = sys.stdin.read(1)
            if next1 == '[':
                next2 = sys.stdin.read(1)
                if next2 == 'A': return 'w' # 上
                elif next2 == 'B': return 's' # 下
                elif next2 == 'C': return 'd' # 右
                elif next2 == 'D': return 'a' # 左
    except Exception as e:
        # print(f"读取按键时发生错误: {e}") # 正常情況下不需要打印
        ch = None
    finally:
        # 無論如何都要恢復終端設置
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# --- 修改: 鍵盤線程接收 node 實例 ---
def keyboard_control_thread(node: Node):
    """键盘控制线程. 修改節點的 servo_angles 和 stop_control."""
    while not node.stop_control: # --- 修改: 使用 node.stop_control ---
        key = get_key()
        if key is None:
            time.sleep(0.01)
            continue

        # --- 修改: 直接修改 node.servo_angles ---
        # 注意：這裡的索引 0 對應 AimNode 的 angle1 (通常是偏航 Yaw), 索引 1 對應 angle2 (通常是俯仰 Pitch)
        # 鍵盤 W/S 通常控制俯仰 (Pitch)，A/D 通常控制偏航 (Yaw)
        # 假設 servo_id[0] 是偏航舵機 (ID 1 in AimNode), servo_id[1] 是俯仰舵機 (ID 2 in AimNode)
        # AimNode: angle1 控制 ID 1 (X), angle2 控制 ID 2 (Y)
        # 這裡：servo_angles[0] 對應 ID=node.servo_id[0], servo_angles[1] 對應 ID=node.servo_id[1]

        if key == 'w': # 向上 (Pitch Up - 通常減小 Y 角度)
            # 假設 ID 2 (索引 1) 是俯仰舵機
            node.servo_angles[1] -= STEP_SIZE # AimNode 中 angle2 減小
            print(f"俯仰 (ID {node.servo_id[1]}) 目标角度偏移: {node.servo_angles[1]:.1f}")
        elif key == 's': # 向下 (Pitch Down - 通常增大 Y 角度)
             # 假設 ID 2 (索引 1) 是俯仰舵機
            node.servo_angles[1] += STEP_SIZE # AimNode 中 angle2 增大
            print(f"俯仰 (ID {node.servo_id[1]}) 目标角度偏移: {node.servo_angles[1]:.1f}")
        elif key == 'a': # 向左 (Yaw Left - 通常減小 X 角度)
            # 假設 ID 1 (索引 0) 是偏航舵機
            node.servo_angles[0] -= STEP_SIZE # AimNode 中 angle1 減小
            print(f"偏航 (ID {node.servo_id[0]}) 目标角度偏移: {node.servo_angles[0]:.1f}")
        elif key == 'd': # 向右 (Yaw Right - 通常增大 X 角度)
            # 假設 ID 1 (索引 0) 是偏航舵機
            node.servo_angles[0] += STEP_SIZE # AimNode 中 angle1 增大
            print(f"偏航 (ID {node.servo_id[0]}) 目标角度偏移: {node.servo_angles[0]:.1f}")
        elif key == 'q':
            node.stop_control = True # --- 修改: 使用 node.stop_control ---
            print("请求停止控制...")
            break

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # --- 修改: 參數宣告與 AimNode 一致 ---
        self.declare_parameter('servo_port_name', '/dev/ttyUSB0')
        self.declare_parameter('servo_baudrate', 115200)
        self.declare_parameter('servo_id', [1, 2], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        # --- 修改: mid 宣告與 AimNode 一致 (列表，索引對應 ID，可能需要填充) ---
        # 假設 ID 從 1 開始，索引 0 不使用或用於特殊目的
        # Default value needs padding if max(servo_id) > len(default) - 1
        # Example: if servo_id=[1, 3], mid needs at least 4 elements [pad, mid1, mid2, mid3]
        # Here, default assumes IDs 1 and 2 are used.
        self.declare_parameter('mid', [0, 2047, 2047], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)) # [pad, mid_for_id_1, mid_for_id_2]
        # --- 修改: angle_range 宣告與 AimNode 一致 (嵌套列表) ---
        # [[pad_min, pad_max], [min_id1, max_id1], [min_id2, max_id2]]
        self.declare_parameter('angle_range', [0, 0, -1400, 1400, -650, 1024], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)) # 使用整數數組列表描述符
        # --- 修改: initial_angles 仍然是簡單列表，長度與 servo_id 相同 ---
        self.declare_parameter('initial_angles', [0.0, 0.0], ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))

        # --- 獲取參數值 ---
        self.servo_port_name = self.get_parameter('servo_port_name').get_parameter_value().string_value
        self.servo_baudrate = self.get_parameter('servo_baudrate').get_parameter_value().integer_value
        self.servo_id = self.get_parameter('servo_id').get_parameter_value().integer_array_value
        # --- 修改: 獲取 mid 和 angle_range ---
        self.mid = self.get_parameter('mid').get_parameter_value().integer_array_value
        # angle_range 參數讀取的是整數數組列表，需要手動轉換
        angle_range_param = self.get_parameter('angle_range').get_parameter_value()
        if angle_range_param.type == ParameterType.PARAMETER_INTEGER_ARRAY:
             # Assuming it's flattened: [pad0, pad0, min1, max1, min2, max2, ...]
             int_array = angle_range_param.integer_array_value
             # Reconstruct the nested list structure based on servo_id length + padding
             # We expect len(int_array) >= 2 * (max(self.servo_id) + 1) if padded from 0
             # Or len(int_array) >= 2 * max(self.servo_id) if padding implicit
             # Let's assume structure [[0,0], [min1, max1], [min2, max2]] as default shows
             max_id = max(self.servo_id) if self.servo_id else 0
             self.angle_range = []
             if len(int_array) >= 2 * (max_id + 1): # Check if enough elements exist for ID-based indexing
                 for i in range(max_id + 1):
                     self.angle_range.append([int_array[i*2], int_array[i*2+1]])
             else:
                 self.get_logger().error(f"angle_range 參數格式或長度 ({len(int_array)}) 不足以支持最大的 servo_id ({max_id})!")
                 # Handle error - maybe use a default or shutdown
                 self.angle_range = [[0, 0]] * (max_id + 1) # Fallback to zero ranges
                 # rclpy.shutdown()
                 # sys.exit(1)
        else:
             # Handle incorrect type if necessary
             self.get_logger().error(f"angle_range 參數類型不是 INTEGER_ARRAY!")
             self.angle_range = [[0, 0]] * (max(self.servo_id) + 1) # Fallback

        self.initial_angles = self.get_parameter('initial_angles').get_parameter_value().double_array_value

        # --- 修改: 使用 self.servo_angles 作為成員變數 ---
        self.servo_angles = list(self.initial_angles)

        # --- 修改: 使用 self.stop_control 作為成員變數 ---
        self.stop_control = False

        # 檢查參數有效性
        # --- 修改: 調整檢查以匹配新的參數結構 ---
        max_servo_id = max(self.servo_id) if self.servo_id else 0
        if len(self.mid) <= max_servo_id:
            self.get_logger().error(f"mid 參數長度 ({len(self.mid)}) 必須至少為 {max_servo_id + 1} 以包含最大的 servo_id ({max_servo_id})!")
            rclpy.shutdown()
            sys.exit(1)
        if len(self.angle_range) <= max_servo_id:
             self.get_logger().error(f"angle_range 參數格式解析錯誤或長度 ({len(self.angle_range)}) 不足以包含最大的 servo_id ({max_servo_id})!")
             rclpy.shutdown()
             sys.exit(1)
        if len(self.initial_angles) != len(self.servo_id):
             self.get_logger().error(f"initial_angles 參數長度 ({len(self.initial_angles)}) 必須等於 servo_id 數量 ({len(self.servo_id)})！")
             rclpy.shutdown()
             sys.exit(1)

        # --- 初始化串口和舵機管理器 (使用 self.servo_manager) ---
        try:
            self.uart = serial.Serial(port=self.servo_port_name, baudrate=self.servo_baudrate,
                                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS, timeout=0.1)
            # --- 修改: 使用 self.servo_manager ---
            self.servo_manager = UartServoManager(self.uart, servo_id_list=list(self.servo_id))

            self.get_logger().info(f"串口 {self.servo_port_name} 打開成功。")
            self.get_logger().info(f"初始化舵機 ID: {self.servo_id}")
            # --- 修改: 打印新的參數結構 ---
            self.get_logger().info(f"舵機中位值 (索引對應ID): {self.mid}")
            self.get_logger().info(f"舵機角度範圍 (索引對應ID): {self.angle_range}")
            self.get_logger().info(f"設定初始角度偏移: {self.servo_angles}")

            # --- 修改: 設定初始角度 - 使用新的 set_angle 方法 ---
            for i, angle_offset in enumerate(self.servo_angles):
                servo_actual_id = self.servo_id[i] # 獲取對應的舵機 ID
                self.set_angle(servo_actual_id, angle_offset) # 使用 ID 和 角度偏移量
            time.sleep(0.5) # 等待舵机就位
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口 {self.servo_port_name}: {e}")
            rclpy.shutdown()
            sys.exit(1)
        except Exception as e:
             self.get_logger().error(f"初始化舵機管理器時發生錯誤: {e}")
             rclpy.shutdown()
             sys.exit(1)

        # --- 創建定時器和啟動線程 ---
        self.control_timer = self.create_timer(CONTROL_INTERVAL, self.control_servo)

        # --- 修改: 啟動鍵盤線程並傳遞 self ---
        self.keyboard_thread = threading.Thread(target=keyboard_control_thread, args=(self,), daemon=True)
        self.keyboard_thread.start()

        # 打印控制說明 (保持不變)
        print("\n---------------------------\n")
        print("键盘控制云台 (Keyboard Gimbal Control)\n")
        print("---------------------------\n")
        print("'w'/'↑': 俯仰 (Pitch) 向上 (ID: {})\n".format(self.servo_id[1] if len(self.servo_id)>1 else 'N/A')) # 假設 ID 2 (索引 1) 是俯仰
        print("'s'/'↓': 俯仰 (Pitch) 向下 (ID: {})\n".format(self.servo_id[1] if len(self.servo_id)>1 else 'N/A'))
        print("'a'/'←': 偏航 (Yaw) 向左 (ID: {})\n".format(self.servo_id[0] if len(self.servo_id)>0 else 'N/A')) # 假設 ID 1 (索引 0) 是偏航
        print("'d'/'→': 偏航 (Yaw) 向右 (ID: {})\n".format(self.servo_id[0] if len(self.servo_id)>0 else 'N/A'))
        print("'q': 退出 (Quit)\n")
        print("---------------------------\n")
        print(f"使用串口: {self.servo_port_name}, 波特率: {self.servo_baudrate}\n")
        print(f"控制舵機 ID: {self.servo_id}\n")
        print("---------------------------")

    # --- 修改: control_servo 使用成員變數 ---
    def control_servo(self):
        """定時器回調，根據 self.servo_angles 控制舵機."""
        if self.stop_control: # --- 修改: 使用 self.stop_control ---
            if self.control_timer and not self.control_timer.is_canceled():
                self.control_timer.cancel()
                self.get_logger().info("控制定時器已取消。")
            if hasattr(self, 'uart') and self.uart and self.uart.is_open:
                self.uart.close()
                self.get_logger().info(f"串口 {self.servo_port_name} 已關閉。")
            print("云台控制程序正在退出...")
            # Don't call shutdown here, let main handle it
            # rclpy.try_shutdown() # Removed from here
            return

        if self.servo_manager is not None: # --- 修改: 使用 self.servo_manager ---
            try:
                # --- 修改: 根據 self.servo_angles 列表設定每個舵機 ---
                for i, angle_offset in enumerate(self.servo_angles):
                    servo_actual_id = self.servo_id[i] # 獲取真實舵機 ID
                    self.set_angle(servo_actual_id, angle_offset) # 使用 ID 和 角度偏移量
            except Exception as e:
                self.get_logger().error(f"控制舵機時出錯: {e}")

    # --- 修改: 採用 AimNode 的 set_angle 邏輯 ---
    def set_angle(self, id, angle_offset):
        """
        设置舵机角度 (基於 AimNode 的邏輯修改).
        參數:
        id: 舵機 ID (例如 1 或 2)
        angle_offset: 目標角度偏移量 (度)
        """
        if self.servo_manager is None:
            return

        if id < 0 or id >= len(self.mid) or id >= len(self.angle_range):
            self.get_logger().warn(f"舵機 ID {id} 無效或參數列表不足!")
            return

        mid_pos = self.mid[id]
        min_offset_pos = self.angle_range[id][0]
        max_offset_pos = self.angle_range[id][1]

        # 假设你的角度偏移量需要转换为位置偏移量，你需要知道你的舵机每度的位置值变化
        # 这里需要根据你的舵机规格进行转换，例如假设 1 度对应 10 个位置值 (这只是一个假设)
        position_offset = int(angle_offset * 10) # 需要根据实际情况调整转换因子

        target_pos = mid_pos + position_offset
        min_servo_pos = mid_pos + min_offset_pos
        max_servo_pos = mid_pos + max_offset_pos

        clamped_pos = max(min_servo_pos, min(max_servo_pos, target_pos))
        clamped_pos = int(clamped_pos)

        self.get_logger().info(f"Setting ID {id}: angle_offset={angle_offset:.2f}, target_pos={clamped_pos}")

        try:
            self.servo_manager.set_position(id, clamped_pos)
        except Exception as e:
            self.get_logger().error(f"通过 servo_manager 设置舵机 ID {id} 位置时出错: {e}")


def main(args=None):
    rclpy.init(args=args)
    servo_control_node = None
    stop_signal_received = False # 添加標誌來防止重複清理
    try:
        servo_control_node = ServoControlNode()
        # --- 修改: 使用 spin 而不是 try_spin_once_until_future_complete ---
        # spin 會阻塞直到節點關閉，更適合這種持續運行的節點
        rclpy.spin(servo_control_node)

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止...")
        stop_signal_received = True
        if servo_control_node:
            servo_control_node.stop_control = True # 觸發節點內的停止邏輯
        # 給一點時間讓回調和線程處理停止信號
        time.sleep(0.5)
    except Exception as e:
        stop_signal_received = True # 發生錯誤也標記為停止
        if servo_control_node:
            servo_control_node.get_logger().fatal(f"節點運行時發生未處理的致命錯誤: {e}", exc_info=True) # 打印堆栈信息
        else:
            print(f"節點初始化時發生致命錯誤: {e}")
    finally:
        # --- 修改: 清理邏輯確保在 spin 結束或異常後執行 ---
        print("開始清理資源...")
        if servo_control_node:
            # 確保 stop_control 被設置，以防是 spin 正常結束
            if not stop_signal_received:
                 servo_control_node.stop_control = True
                 time.sleep(0.2) # 給點時間讓 control_servo 執行關閉邏輯

            # 再次檢查並關閉定時器和串口 (可能已在 control_servo 中關閉)
            if hasattr(servo_control_node, 'control_timer') and servo_control_node.control_timer and not servo_control_node.control_timer.is_canceled():
                servo_control_node.control_timer.cancel()
                print("控制定時器已確認取消。")
            if hasattr(servo_control_node, 'uart') and servo_control_node.uart and servo_control_node.uart.is_open:
                servo_control_node.uart.close()
                print(f"串口 {servo_control_node.servo_port_name} 已確認關閉。")

            servo_control_node.destroy_node()
            print("ServoControlNode 已銷毀。")

        # 確保 rclpy 只關閉一次
        if rclpy.ok():
            rclpy.shutdown()
            print("rclpy 已關閉。")
        print("程序退出。")


if __name__ == '__main__':
    main()