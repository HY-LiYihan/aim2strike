import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import serial


'''
常量与数据表定义 - JOHO串口总线舵机 Python SDK 
--------------------------------------------------
- 作者: 阿凯爱玩机器人@成都深感机器人
- Email: xingshunkai@qq.com
- 更新时间: 2021-12-19
--------------------------------------------------
'''
# 广播地址
SERVO_ID_BRODCAST = 0xFE
# 舵机状态掩码
STATUS_MASK_UNDER_VOLTAGE = 1			# 欠压保护
STATUS_MASK_OVER_VOLTAGE = 1 << 1		# 过压保护
STATUS_MASK_OVER_TEMPERATURE = 1 << 2	# 过温保护
STATUS_MASK_OVER_ELEC_CURRENT = 1 << 3	# 过流保护
STATUS_MASK_STALL_PROTECTION = 1 << 4	# 堵转保护
# 电机模式
MOTOR_MODE_SERVO = 0x01 # 舵机模式
MOTOR_MODE_DC = 0x00 	# 直流电机模式
# 电机旋转方向
DC_DIR_CCW = 0x01		# 电机顺时针旋转
DC_DIR_CW = 0x00		# 电机逆时针旋转
# 波特率
BDR_USER_DEFINE = 0 	# 用户自定义波特率
BDR_9600 = 1			
BDR_38400 = 2
BDR_57600 = 3
BDR_76800 = 4
BDR_115200 = 5
BDR_128000 = 6
BDR_250000 = 7
BDR_500000 = 8
BDR_1000000 = 9
# 扭力开关
TORQUE_ENABLE = 0x01
TORQUE_DISABLE = 0x00
# 示教点
TEACHING_POINT_1 = 1
TEACHING_POINT_2 = 2
TEACHING_POINT_3 = 3

# 串口总线舵机数据表
UART_SERVO_DATA_TABLE = {
	'DATA_VERSION_A': (0x03, 'B'), 		# 版本号 A
	'DATA_VERSION_B': (0x04, 'B'), 		# 版本号 B
	'SERVO_ID' : (0x05, 'B'),			# 舵机ID
	'STALL_PROTECTION_S' : (0x06, 'B'), # 舵机堵转保护
	'ANGLE_LOWERB' : (0x09, 'H'), 		# 舵机角度最小值 
	'ANGLE_UPPERB' : (0x0B, 'H'), 		# 舵机角度最大值
	'TEMPERATURE_PROTECTION_THRESHOLD': (0x0D, 'B'), # 温度保护阈值
	'VOLTAGE_UPPERB' : (0x0E, 'B'), 	# 电压上限, 单位V
	'VOLTAGE_LOWERB' : (0x0F, 'B'), 	# 电压下限, 单位V
	'TORQUE_UPPERB' : (0x10, 'H'), 		# 最大扭矩，取值范围 [0, 1000]
	'MIDDLE_POSI_ADJUST' : (0x14, 'h'), # 中位调整
	'TEACHING_POINT_1' : (0X16, 'H'), 	# 示教点1
 	'TEACHING_POINT_2' : (0X18, 'H'), 	# 示教点2
	'TEACHING_POINT_3' : (0X1A, 'H'), 	# 示教点3
	'MOTOR_MODE' : (0x1C, 'B'), 		# 电机模式
	'MOTOR_DIR' : (0x1D, 'B'),			# 电机旋转方向
	'BAUDRATE' : (0x1E, 'B'), 			# 波特率
	'CONTROL_P_KP' : (0x1F, 'B'), 		# 位置环 Kp
	'CONTROL_P_KI' : (0x20, 'B'), 		# 位置环 Ki
 	'CONTROL_P_KD' : (0x21, 'B'), 		# 位置环 Kd
	'CONTROL_V_KP' : (0x22, 'B'), 		# 速度环 Kp
	'CONTROL_V_KI' : (0x23, 'B'), 		# 速度环 Ki
	'CONTROL_V_KD' : (0x24, 'B'), 		# 速度环 Kd
	'TORQUE_ENABLE' : (0x28, 'B'), 		# 扭力开关
	'TARGET_POSITION' : (0x2A, 'H'), 	# 目标位置, 取值范围 [0, 4095]
	'RUNTIME_MS' : (0x2C, 'H'), 		# 运行时间, 单位ms
	'ELECTRIC_CURRENT_MA' : (0x2E, 'H'),# 电流, 单位mA
 	'CURRENT_POSITION' : (0x38, 'H'), 	# 当前位置, 取值范围 [0, 4095]
	'CURRENT_VELOCITY' : (0x3A, 'H'), 	# 当前速度, 单位 °/s
	'GO2TECHING_POINT' : (0x3C, 'B'), 	# 运行到示教点
	'CURRENT_VOLTAGE' : (0x3E, 'B'), 	# 当前电压, 单位V
	'CURRENT_TEMPERATURE' : (0x3F, 'B'),# 当前温度, 单位摄氏度
	'REG_WRITE_FLAG' : (0x40, 'B'), 	# 寄存器异步写入标志位
	'MOTOR_SPEED' : (0x41, 'h'), 		# 电机PWM, 取值范围[0, 100]
}

'''
数据帧 - JOHO串口总线舵机 Python SDK 
--------------------------------------------------
- 作者: 阿凯爱玩机器人@成都深感机器人
- Email: xingshunkai@qq.com
- 更新时间: 2021-12-19
--------------------------------------------------
'''
import logging
import struct

# 设置日志等级
class Packet:
	'''数据包'''
	# 使用pkt_type来区分请求数据还是响应数据
	PKT_TYPE_REQUEST = 0 # 请求包
	PKT_TYPE_RESPONSE = 1 # 响应包
	HEADER_LEN = 2 # 帧头校验数据的字节长度
	HEADERS = [b'\xff\xff', b'\xff\xf5']
	
	CODE_LEN = 1 # 功能编号长度
	ID_LEN = 1 # 舵机ID的长度
	SIZE_LEN = 1 # 字节长度
	STATUS_LEN = 1 # 舵机状态长度
	CHECKSUM_LEN = 1 # 校验和长度

	@classmethod
	def calc_checksum_request(cls, servo_id, data_size, cmd_type, param_bytes):
		'''计算请求包的校验和'''
		bytes_buffer = struct.pack('>BBB', servo_id, data_size, cmd_type) + param_bytes
		return 0xFF - (sum(bytes_buffer) & 0xFF)

	@classmethod
	def calc_checksum_response(cls, servo_id, data_size, servo_status, param_bytes):
		'''计算响应包的校验和'''
		bytes_buffer = struct.pack('>BBB', servo_id, data_size, servo_status) + param_bytes
		return 0xFF - (sum(bytes_buffer) & 0xFF)

	@classmethod
	def is_response_legal(cls, packet_bytes):
		'''检验响应包是否合法'''
		# 获取帧头
		header = cls.HEADERS[cls.PKT_TYPE_RESPONSE]
		# 帧头检验
		if packet_bytes[:cls.HEADER_LEN] != cls.HEADERS[cls.PKT_TYPE_RESPONSE]:
			return False, None
		# 提取ID, 数据长度
		idx_status = cls.HEADER_LEN + cls.ID_LEN + cls.SIZE_LEN + cls.STATUS_LEN
		servo_id, data_size, servo_status = struct.unpack('<BBB', packet_bytes[cls.HEADER_LEN : idx_status])
		# 长度校验
		param_bytes = packet_bytes[idx_status: -cls.CHECKSUM_LEN]
		if (len(param_bytes) + 2) != data_size:
			print("param size not match")
			return False, None

		# 校验和检验
		checksum1 = packet_bytes[-cls.CHECKSUM_LEN]
		checksum2 = cls.calc_checksum_response(servo_id, data_size, servo_status, param_bytes)
  
		# 校验和检查
		if checksum1 != checksum2:
			print(f"checksum1: {checksum1}  checksum2: {checksum2}")
			return False, None
		# 数据检验合格
		return True, [servo_id, data_size, servo_status, param_bytes]

	@classmethod
	def pack(cls, servo_id, cmd_type, param_bytes=b''):
		'''数据打包为二进制数据'''
		data_size = len(param_bytes) + 2
		checksum = cls.calc_checksum_request(servo_id, data_size, cmd_type, param_bytes)
		frame_bytes = cls.HEADERS[cls.PKT_TYPE_REQUEST] + struct.pack('<BBB', servo_id, data_size, cmd_type) + param_bytes + struct.pack('<B', checksum)
		return frame_bytes
	
	@classmethod
	def unpack(cls, packet_bytes):
		'''二进制数据解包为所需参数'''
		ret, result =  cls.is_response_legal(packet_bytes)
		if not ret:
			# 数据非法
			return None
		return result



'''
数据帧缓冲队列 - JOHO串口总线舵机 Python SDK 
--------------------------------------------------
- 作者: 阿凯爱玩机器人@成都深感机器人
- Email: xingshunkai@qq.com
- 更新时间: 2021-12-19
--------------------------------------------------
'''
import logging
import struct
from src.packet import Packet

class PacketBuffer:
	'''Packet中转站'''
	def __init__(self, is_debug=False):
		self.is_debug = is_debug
		self.packet_bytes_list = []
		# 清空缓存区域
		self.empty_buffer()
	
	def update(self, next_byte):
		'''将新的字节添加到Packet中转站'''
		
		# < int > 转换为 bytearray
		next_byte = struct.pack(">B", next_byte)
		# print(f"next_byte = {next_byte}")
		if not self.header_flag:
			
			# 接收帧头
			if len(self.header) < Packet.HEADER_LEN:
				# 向Header追加字节
				self.header += next_byte
				if len(self.header) == Packet.HEADER_LEN and self.header == Packet.HEADERS[Packet.PKT_TYPE_RESPONSE]:
					# print(f"recv header: {self.header}")
					self.header_flag = True
			elif len(self.header) == Packet.HEADER_LEN:
				# 首字节出队列
				self.header = self.header[1:] + next_byte
				# 查看Header是否匹配
				if self.header == Packet.HEADERS[Packet.PKT_TYPE_RESPONSE]:
					# print('header: {}'.format(self.header))
					self.header_flag = True
		elif not self.servo_id_flag:
			# 接收舵机ID
			self.servo_id += next_byte
			self.servo_id_flag = True
			# print(f"servo_id : {self.servo_id}")
		elif not self.data_size_flag:
			# 填充参数尺寸
			self.data_size += next_byte
			self.data_size_flag = True 
			# 参数长度
			self.param_len = struct.unpack('>B', self.data_size)[0] - 2
			# print(f"参数长度:  {self.param_len}")
			if self.param_len == 0:
				self.param_bytes_flag = True
		elif not self.servo_status_flag:
			# 舵机状态
			self.servo_status += next_byte
			self.servo_status_flag = True
			# print(f"servo_status: {self.servo_status}")
		elif not self.param_bytes_flag:
			# 填充参数
			if len(self.param_bytes) < self.param_len:
				self.param_bytes += next_byte
				if len(self.param_bytes) == self.param_len:
					self.param_bytes_flag = True
		else:
			# 计算校验和
			tmp_packet_bytes = self.header + self.servo_id + self.data_size + self.servo_status + self.param_bytes + next_byte
			# print(f"tmp_packet_bytes : {tmp_packet_bytes}")
			ret, result = Packet.is_response_legal(tmp_packet_bytes)
			if ret:
				self.checksum_flag = True
				# 将新的Packet数据添加到中转列表里
				self.packet_bytes_list.append(tmp_packet_bytes)
			# 重新清空缓冲区
			self.empty_buffer()
		
	def empty_buffer(self):
		# 数据帧是否准备好
		self.param_len = None
		# 帧头
		self.header = b''
		self.header_flag = False
		# 舵机ID
		self.servo_id = b''
		self.servo_id_flag = False
		# 数据长度
		self.data_size = b''
		self.data_size_flag = False
		# 舵机状态
		self.servo_status = b''
		self.servo_status_flag = False
		# 参数
		self.param_bytes = b''
		self.param_bytes_flag = False
	
	def has_valid_packet(self):
		'''是否有有效的包'''
		return len(self.packet_bytes_list) > 0
	
	def get_packet(self):
		'''获取队首的Bytes'''
		return self.packet_bytes_list.pop(0)

'''
JOHO串口总线舵机 Python SDK 
--------------------------------------------------
- 作者: 阿凯爱玩机器人@成都深感机器人
- Email: xingshunkai@qq.com
- 更新时间: 2021-12-19
--------------------------------------------------
'''
import time
import logging
import serial
import struct
from src.packet import Packet
from src.packet_buffer import PacketBuffer
from src.data_table import *

class UartServoInfo:
	'''串口舵机的信息'''
	SERVO_DEADBLOCK = 1 # 舵机死区
	
	def __init__(self, servo_id, lowerb=None, upperb=None):
		self.is_online = False 	 # 电机是否在线 
		self.servo_id = servo_id # 舵机的ID
		self.cur_position = None # 当前位置
		self.target_position = None # 目标位置
		# 位置与角度的变换关系
		self.position2angle_k = 360 / 4096.0
		self.position2angle_b = 0.0

		self.last_angle_error = None    # 上一次的角度误差
		self.last_sample_time = None    # 上一次的采样时间
		# 内存表数据
		self.data_table_raw_dict = {} # 原始数据 字典类型
		# 内存表写入标志位
		self.data_write_success = False
		# 舵机状态
		self.status = 0 # 舵机状态

	def is_stop(self):
		'''判断舵机是否已经停止'''
		# # 如果没有指定目标角度， 就将其设置为当前角度
		# if self.target_angle is None:
		# 	self.target_angle = self.cur_angle
		# 角度误差判断
		angle_error = self.target_angle - self.cur_angle
		if abs(angle_error) <= self.SERVO_DEADBLOCK:
			return True
		
		if self.last_angle_error is None:
			self.last_angle_error = angle_error
			self.last_sample_time = time.time()
		
		# 角度误差在死区范围以内则判断为已经到达目标点
		
		# 更新采样数据
		if abs(self.last_angle_error - angle_error) > 0.2:
			self.last_angle_error = angle_error
			self.last_sample_time = time.time() # 更新采样时间

		if (time.time() - self.last_sample_time) > 1:
			# 已经有1s没有更新误差了, 舵机卡住了
			self.last_angle_error = None
			self.last_sample_time = None
			return True
		
		return False

	def position2angle(self, position):
		'''位置转化为角度'''
		return self.position2angle_k * position + self.position2angle_b
	
	def angle2position(self, angle):
		'''角度转换为位置'''

		pass

	@property
	def cur_angle(self):
		return self.position2angle(self.cur_position)

	@property
	def target_angle(self):
		return self.position2angle(self.target_position)

	def move(self, target_position):
		'''设置舵机的目标角度'''
		# 设置目标角度
		self.target_position = target_position

	def update(self, cur_position):
		'''更新当前舵机的角度'''
		self.cur_position = cur_position
	
	def __str__(self):
		return "目标角度:{:.1f} 实际角度:{:.1f} 角度误差:{:.2f}".format(self.target_angle, self.cur_angle, self.target_angle-self.cur_angle)

class UartServoManager:
	'''串口总线舵机管理器'''
	# 数据帧接收Timeout
	RECEIVE_TIMEOUT = 0.02		# 接收超时延迟
	RETRY_NTIME = 10 			# 通信失败后，重试的次数
	DELAY_BETWEEN_CMD = 0.001	# 数据帧之间的延时时间
	# 命令类型定义
	CMD_TYPE_PING = 0x01		# 查询
	CMD_TYPE_READ_DATA = 0x02	# 读
	CMD_TYPE_WRITE_DATA = 0x03	# 写
	CMD_TYPE_REG_WRITE = 0x04	# 异步写
	CMD_TYPE_ACTION = 0x05		# 执行异步写
	CMD_TYPE_RESET = 0x06		# 回复出厂设置
	CMD_TYPE_SYNC_WRITE = 0x83	# 同步写
	# 电机控制
	POSITION_DEADAREA = 10		# 舵机位置控制 死区 	 
	def __init__(self, uart, servo_id_list=[1]):
		'''初始化舵机管理器'''
		self.uart = uart						# 串口
		self.pkt_buffer = PacketBuffer()		# 数据帧缓冲区
  		# 创建舵机信息字典
		self.servo_info_dict = {}				# 舵机信息字典
		# 舵机扫描
		self.servo_scan(servo_id_list)
	
	def receive_response(self):
		'''接收单个数据帧'''
  		# 清空缓冲区
		self.pkt_buffer.empty_buffer()
		# 开始计时
		t_start = time.time()
		while True:
			# 判断是否有新的数据读入
			buffer_bytes = self.uart.readall()
			if buffer_bytes is not None and len(buffer_bytes) > 0:
				for next_byte in buffer_bytes:
					self.pkt_buffer.update(next_byte)
			# 弹出接收的数据帧
			if self.pkt_buffer.has_valid_packet():
				# 获取数据帧
				packet_bytes = self.pkt_buffer.get_packet()
				# 提取数据帧参数
				result = Packet.unpack(packet_bytes)
				servo_id, data_size, servo_status, param_bytes = result
				# 舵机状态自动同步
				if servo_id in self.servo_info_dict.keys():
					self.servo_info_dict[servo_id].status = servo_status
				return packet_bytes
			# 超时判断
			if time.time() - t_start > self.RECEIVE_TIMEOUT:
				return None

	def send_request(self, servo_id, cmd_type, param_bytes, wait_response=False, retry_ntime=None):
		'''发送请求'''
		if wait_response:
			# 清空串口缓冲区
			self.uart.readall()

		packet_bytes = Packet.pack(servo_id, cmd_type, param_bytes)	

		if not wait_response:
			# 发送指令
			self.uart.write(packet_bytes)
			time.sleep(self.DELAY_BETWEEN_CMD)
			return True, None
		else:
			
			if retry_ntime is None:
				retry_ntime = self.RETRY_NTIME
			# 尝试多次
			for i in range(retry_ntime):
				self.uart.write(packet_bytes)
				time.sleep(self.DELAY_BETWEEN_CMD)
				response_packet =  self.receive_response()
				if response_packet is not None:
					return True, response_packet
			# 发送失败
			return False, None

	def ping(self, servo_id):
		'''舵机通讯检测'''
		ret, response_packet = self.send_request(servo_id, self.CMD_TYPE_PING, b'', wait_response=True, retry_ntime=3)
		if ret and servo_id not in self.servo_info_dict.keys():
			# 创建舵机对象
			self.servo_info_dict[servo_id] = UartServoInfo(servo_id)
			# 舵机角度查询
			# TODO?
		return ret
	
	def read_data(self, servo_id, data_address, read_nbyte=1):
		'''读取数据'''
		param_bytes = struct.pack('>BB', data_address, read_nbyte)
		ret, response_packet = self.send_request(servo_id, self.CMD_TYPE_READ_DATA, param_bytes, wait_response=True)
		if ret:
			# 提取读取到的数据位
			servo_id, data_size, servo_status, param_bytes = Packet.unpack(response_packet)
			return True, param_bytes
		else:
			return False, None
	
	def write_data(self, servo_id, data_address, param_bytes):
		'''写入数据'''
		param_bytes = struct.pack('>B', data_address) + param_bytes
		self.send_request(servo_id, self.CMD_TYPE_WRITE_DATA, param_bytes)
		return True

	def read_data_by_name(self, servo_id, data_name):
		'''根据名字读取数据'''
		# 获取地址位与数据类型
		if data_name not in UART_SERVO_DATA_TABLE:
			return None
		data_address, dtype = UART_SERVO_DATA_TABLE[data_name]
		read_nbyte = 1
		if dtype in ['h', 'H']:
			read_nbyte = 2
		ret, param_bytes = self.read_data(servo_id, data_address, read_nbyte=read_nbyte)
		if not ret:
			return None
		# 数据解析
		value = struct.unpack(f">{dtype}", param_bytes)[0]
		return value

	def write_data_by_name(self, servo_id, data_name, value):
		'''根据名称写入数据'''
		if data_name not in UART_SERVO_DATA_TABLE:
			return None
		data_address, dtype = UART_SERVO_DATA_TABLE[data_name]
		param_bytes = struct.pack(f">{dtype}", value)
		self.write_data(servo_id, data_address, param_bytes)
	
	def get_legal_position(self, position):
		'''获取合法的位置'''
		position = int(position)
		if position > 4095:
			position = 4095
		elif position < 0:
			position = 0
		return position

	def async_set_position(self, servo_id, position, runtime_ms):
		'''异步写入位置控制信息'''
		# 参数规范化
		if servo_id in self.servo_info_dict.keys():
			self.servo_info_dict[servo_id].is_stop = False
		position = self.get_legal_position(position)
		runtime_ms = int(runtime_ms)

		address, _ = UART_SERVO_DATA_TABLE['TARGET_POSITION']
		param_bytes = struct.pack('>BHH', address,  position, runtime_ms)
		self.send_request(servo_id, self.CMD_TYPE_REG_WRITE, param_bytes)
		return True

	def async_action(self):
		'''执行异步位置控制信息'''
		self.send_request(SERVO_ID_BRODCAST, self.CMD_TYPE_ACTION, b'')
		return True

	def sync_set_position(self, servo_id_list, position_list, runtime_ms_list):
		'''同步写指令'''
		param_bytes = b'\x2A\x04'
		servo_num = len(servo_id_list) # 舵机个数
		for sidx in range(servo_num):
			servo_id = servo_id_list[sidx]
			if servo_id in self.servo_info_dict.keys():
				self.servo_info_dict[servo_id].is_stop = False
			position = position_list[sidx]
			position = self.get_legal_position(position)
			runtime_ms = runtime_ms_list[sidx]
			runtime_ms = int(runtime_ms)
			param_bytes += struct.pack('>BHH', servo_id, position, runtime_ms)
		self.send_request(SERVO_ID_BRODCAST, self.CMD_TYPE_SYNC_WRITE, param_bytes)
	
	def reset(self, servo_id):
		'''舵机恢复出厂设置'''
		self.send_request(servo_id, self.CMD_TYPE_RESET, b'')
	
	def set_position(self, servo_id, position, is_wait=False):
		'''设置舵机位置'''
		position = self.get_legal_position(position)
		self.write_data_by_name(servo_id, "TARGET_POSITION", position)
		if servo_id in self.servo_info_dict.keys():
			self.servo_info_dict[servo_id].move(position)
			self.servo_info_dict[servo_id].is_stop = False
		if is_wait:
			self.wait(servo_id)
	def set_runtime_ms(self, servo_id, runtime_ms):
		'''设置运行时间ms'''
		self.write_data_by_name(servo_id, "RUNTIME_MS", runtime_ms)
	
	def get_target_position(self, servo_id):
		'''获取目标位置'''
		return self.read_data_by_name(servo_id, "TARGET_POSITION")

	def get_position(self, servo_id):
		'''查询舵机位置'''
		return self.read_data_by_name(servo_id, "CURRENT_POSITION")

	def get_velocity(self, servo_id):
		'''查询舵机速度'''
		return self.read_data_by_name(servo_id, "CURRENT_VELOCITY")

	def servo_scan(self, servo_id_list=[1]):
		'''舵机扫描'''
		for servo_id in servo_id_list:
			# 尝试ping一下舵机
			if self.ping(servo_id):
				print("发现舵机: {}".format(servo_id))
				# 设置为舵机模式
				self.set_motor_mode(servo_id, MOTOR_MODE_SERVO)
				# 创建舵机对象
				self.servo_info_dict[servo_id] = UartServoInfo(servo_id)
				self.servo_info_dict[servo_id].is_online = True
				# 查询角度并同步角度
				position = self.get_position(servo_id)
				self.servo_info_dict[servo_id].update(position)
				self.servo_info_dict[servo_id].move(position)
			else:
				if servo_id in self.servo_info_dict.keys():
					self.servo_info_dict[servo_id].is_online = False
		
	def wait(self, servo_id):
		'''等待单个舵机停止运动'''
		angle_error_dict = {}
		while True:
			target_position = self.servo_info_dict[servo_id].target_position
			cur_position = self.get_position(servo_id)
			angle_error = abs( cur_position - target_position)
			# print(f"target_position: {target_position}  cur_position:{cur_position}  angle_error:{angle_error}")
			# 小于死区
			if angle_error < self.POSITION_DEADAREA:
				self.servo_info_dict[servo_id].is_stop = True
				break
			# 判断是否卡住了
			if angle_error not in angle_error_dict:
				angle_error_dict[angle_error] = 1
			else:
				angle_error_dict[angle_error] += 1
			if angle_error_dict[angle_error] >= 100:
				self.servo_info_dict[servo_id].is_stop = True
				break
	
	def wait_all(self):
		'''等待所有舵机执行动作'''
		for servo_id in self.servo_info_dict.keys():
			if self.servo_info_dict[servo_id].is_online:
				self.wait(servo_id)

	def set_motor_mode(self, servo_id, mode):
		'''设置电机模式'''
		self.write_data_by_name(servo_id, "MOTOR_MODE", mode)
	
	def dc_rotate(self, servo_id, direction, pwm):
		'''直流电机旋转'''
		pwm = int(pwm)
		pwm = min(100, max(pwm, 0))
		# 设置方向 顺时针: DC_DIR_CW |  逆时针: DC_DIR_CCW
		self.write_data_by_name(servo_id, "MOTOR_DIR", direction)
		# 设置转速 [0, 100]
		self.write_data_by_name(servo_id, "MOTOR_SPEED", pwm)
	
	def dc_stop(self, servo_id):
		self.write_data_by_name(servo_id, "MOTOR_SPEED", 0)
  
	def torque_enable(self, servo_id, enable):
		'''扭力使能'''
		value = TORQUE_ENABLE if enable else TORQUE_DISABLE
		self.write_data_by_name(servo_id, "TORQUE_ENABLE", value)
	
	def torque_enable_all(self, enable):
		'''扭力使能(所有舵机)'''
		value = TORQUE_ENABLE if enable else TORQUE_DISABLE
		self.write_data_by_name(SERVO_ID_BRODCAST, "TORQUE_ENABLE", value)
	
	def set_torque_upperb(self, servo_id, torque_upperb):
		'''设置最大扭力
		@torque_upperb: 取值范围[0, 1000]
  		'''
		self.write_data_by_name(servo_id, "TORQUE_UPPERB", torque_upperb)
  
	def get_temperature(self, servo_id):
		'''获取当前温度'''
		return self.read_data_by_name(servo_id, "CURRENT_TEMPERATURE")

	def get_voltage(self, servo_id):
		'''获取当前电压'''
		return self.read_data_by_name(servo_id, "CURRENT_VOLTAGE")
















# 初始化串口
uart = serial.Serial(port='/dev/ttyUSB0', baudrate=115200,
                     parity=serial.PARITY_NONE, stopbits=1,
                     bytesize=8, timeout=0)

# 创建舵机对象
uservo = UartServoManager(uart, servo_id_list=[1, 2])

angle_range = [(0,0),(-1400,1400),(-650,1024)]
mid = [0,2047,2047]
def set_angle(id, angle):
    """
    设置舵机角度。
    参数:
    id: 舵机ID
    angle: 目标角度
    """
    if (angle + mid[id]) >= (angle_range[id][1] + mid[id]):
        uservo.set_position(id, angle_range[id][1] + mid[id])
    elif (angle + mid[id]) <= (angle_range[id][0] + mid[id]):
        uservo.set_position(id, angle_range[id][0] + mid[id])
    else:
        uservo.set_position(id, angle + mid[id])

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        """
        初始化 PID 控制器.
        
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

    def update(self, feedback_value, dt):
        """
        更新 PID 控制器并计算输出.
        
        参数:
        feedback_value: 当前反馈值
        dt: 时间间隔
        
        返回:
        控制器输出
        """
        # 计算误差
        error = self.setpoint - feedback_value
        
        # 计算积分部分
        self.integral += error * dt
        
        # 计算微分部分
        derivative = (error - self.previous_error) / dt
        
        # 计算 PID 输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 保存误差以备下次计算微分部分
        self.previous_error = error
        
        return output

class AimNode(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('aim_node')
        
        # 加载参数
        self.declare_parameter('servo_port_name', '/dev/ttyUSB0')
        self.declare_parameter('servo_baudrate', 115200)
        self.declare_parameter('servo_id', [1, 2])
        self.declare_parameter('mid', [0, 2047, 2047])
        self.declare_parameter('angle_range', [[0, 0], [-1400, 1400], [-650, 1024]])
        self.declare_parameter('pid_gains.Kp', 0.08)
        self.declare_parameter('pid_gains.Ki', 0.0)
        self.declare_parameter('pid_gains.Kd', 0.01)

        # 获取参数值
        self.servo_port_name = self.get_parameter('servo_port_name').get_parameter_value().string_value
        self.servo_baudrate = self.get_parameter('servo_baudrate').get_parameter_value().integer_value
        self.servo_id = self.get_parameter('servo_id').get_parameter_value().integer_array_value
        self.mid = self.get_parameter('mid').get_parameter_value().integer_array_value
        self.angle_range = self.get_parameter('angle_range').get_parameter_value().integer_array_value
        self.pid_gains = {
            'Kp': self.get_parameter('pid_gains.Kp').get_parameter_value().double_value,
            'Ki': self.get_parameter('pid_gains.Ki').get_parameter_value().double_value,
            'Kd': self.get_parameter('pid_gains.Kd').get_parameter_value().double_value
        }

        # 初始化串口
        self.uart = serial.Serial(port=self.servo_port_name, baudrate=self.servo_baudrate,
                                  parity=serial.PARITY_NONE, stopbits=1,
                                  bytesize=8, timeout=0)

        # 创建舵机对象
        self.uservo = UartServoManager(self.uart, servo_id_list=self.servo_id)

        # 创建订阅者
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'detect/center',
            self.center_callback,
            10)

        # 初始化 PID 控制器
        self.pid_x = PID(self.pid_gains['Kp'], self.pid_gains['Ki'], self.pid_gains['Kd'], setpoint=0.5)
        self.pid_y = PID(self.pid_gains['Kp'], self.pid_gains['Ki'], self.pid_gains['Kd'], setpoint=0.5)

        # 初始化时间戳
        self.last_time = time.time()

        # 初始化舵机角度
        self.angle1 = 0
        self.angle2 = 969
        set_angle(1, self.angle1)
        set_angle(2, self.angle2)
        time.sleep(0.5)

    def center_callback(self, msg):
        # 获取当前时间
        current_time = time.time()
        # 计算时间间隔
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 获取红色物体的归一化中心坐标
        normalized_cX, normalized_cY = msg.data
        
        # 更新 PID 控制器并计算输出
        output_x = self.pid_x.update(normalized_cX, dt)
        output_y = self.pid_y.update(normalized_cY, dt)
        
        # 更新舵机角度
        self.angle1 += output_x
        self.angle2 -= output_y  # Y轴方向可能需要反向
        
        # 设置舵机角度
        set_angle(1, self.angle1)
        set_angle(2, self.angle2)
        
        # 打印舵机角度（调试用）
        self.get_logger().info(f'Servo Angles - X: {self.angle1}, Y: {self.angle2}')

def main(args=None):
    # 初始化 ROS 2 节点
    rclpy.init(args=args)
    # 创建 AimNode 节点实例
    aim_node = AimNode()
    # 开始自旋，等待回调函数触发
    rclpy.spin(aim_node)
    # 销毁节点
    aim_node.destroy_node()
    # 关闭 ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
