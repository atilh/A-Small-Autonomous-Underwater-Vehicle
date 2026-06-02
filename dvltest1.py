import json
import socket
import time
import threading
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import sys
from ticop import sleep_ms
import cv2
import os
from pai import snap
from ph import a_jpg
from ph import b_jpg
from ph import c_jpg


class PIDController:
    """PID控制器类"""

    def __init__(self, Kp, Ki, Kd, output_min, output_max):
        self.Kp = Kp  # 比例系数
        self.Ki = Ki  # 积分系数
        self.Kd = Kd  # 微分系数
        self.output_min = output_min  # 输出最小值
        self.output_max = output_max  # 输出最大值
        self.inte_min = -20
        self.inte_max = 20
        self.integral = 0.0  # 积分项
        self.previous_error = 0.0  # 上一次误差
        self.last_time = time.time()  # 上次更新时间

    def reset(self):
        """重置 PID 内部状态，避免重新启用时的 dt 冲击"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

    def compute(self, setpoint, current_value):
        """计算PID输出"""
        # 计算误差
        error = setpoint - current_value

        # 计算时间变化量
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 比例项
        proportional = self.Kp * error

        # 积分项 (带抗饱和)
        self.integral += error * dt
        integral = self.Ki * self.integral
        if integral > self.inte_max:
            self.integral -= error * dt  # 抗饱和处理
        elif integral < self.inte_min:
            self.integral -= error * dt  # 抗饱和处理
        # 微分项
        derivative = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error

        # 计算总输出
        output = proportional + integral + derivative

        # 输出限幅
        if output > self.output_max:
            output = self.output_max
            # self.integral -= error * dt  # 抗饱和处理
        elif output < self.output_min:
            output = self.output_min
            # self.integral -= error * dt  # 抗饱和处理

        return output


class AdvancedAUVControl:
    def __init__(self):
        # 飞控连接
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14553')
        self.master.wait_heartbeat()
        print("Pixhawk connected!")
        # print('Try:', list(self.master.mode_mapping().keys()))

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        # wait until arming confirmed (can manually check with master.motors_armed())

        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')

        # DVL连接
        self.dvl_ip = "192.168.137.3"
        self.dvl_port = 16171
        self.dvl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dvl_socket.connect((self.dvl_ip, self.dvl_port))
        print(f"Connected to DVL at {self.dvl_ip}:{self.dvl_port}")

        # 数据存储
        self.current_position = [0.0, 0.0]  # [x, y]
        self.current_depth = 0.0
        self.current_heading = 0.0

        self.position_lock = threading.Lock()
        self.current_altitude = 0.0  # 存储当前高度值
        self.altitude_lock = threading.Lock()  # 高度数据的线程锁
        # 控制参数
        self.channel_values = [65535] * 8
        for i in [2, 3, 4]:  # throttle, yaw, forward
            self.channel_values[i] = 1500

        self.target_depth = 0.1  # 默认目标深度
        self.target_altitude = 0.1
        self.move_start_time = None
        self.adjust_start_time = None
        self.last_remaining = 0
        self.boost_level = 1560
        self.boosted_thrust = 1560

        self.altitude_pid = PIDController(
            Kp=500.0,
            Ki=5.0,
            Kd=0,
            output_min=-150,
            output_max=150
        )

        self.depth_pid = PIDController(
            Kp=500.0,
            Ki=10.0,
            Kd=0,
            output_min=-200,
            output_max=200
        )

        self.waypoints = []  # 路径点列表 [(heading, distance, depth), ...]
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.10  # 到达路径点的距离阈值(米)

        self.current_task_index = 0  # 当前任务序号
        self.task_done = False

        # 航段控制状态
        self.segment_state = "adjust_heading"  # 航段状态: adjust_heading, adjust_depth, reset_dvl, move_forward
        self.segment_progress = 0.0  # 当前航段前进距离
        self.segment_target_heading = 0.0  # 当前航段目标航向
        self.segment_target_distance = 0.0  # 当前航段目标距离
        self.last_reset_time = 0  # 上次重置时间

        self.H = "hold"
        self.mo = 0
        # PID参数
        self.Kp_depth = 80.0  # 深度比例系数
        self.Kp_yaw = 2.0  # 偏航比例系数
        self.Kp_forward = 230  # 前进比例系数
        self.heading_threshold = 3.0  # 航向调整阈值(度)
        self.depth_threshold = 0.1  # 深度调整阈值(米)
        self.stuck_timer_start = 0
        self.is_stuck = False
        self.depth_control_enabled = False

        # 手动控制参数
        self.manual_throttle = 1500
        self.manual_yaw = 1500
        self.manual_forward = 1500
        self.begin = self.get_initial_heading()

        self.target_attitude = {'roll': 0, 'pitch': 0, 'yaw': self.begin}

        self.waypoints = [

            {"h": self.begin, "d": 0.5, "z": 0.2, "tasks": []},

        ]


        self.waypoints_use = self.waypoints

        self.boost_fwd = 1500  # 前进累加
        self.boost_rev = 1500  # 后退累加

        self.ok = 0

        # 线程控制
        self.running = True
        self.boot_time = time.time()
        self.control_mode = "manual"  # "manual" 或 "auto"

        # 启动线程

        self.dvl_thread = threading.Thread(target=self.read_dvl_data)
        self.dvl_thread.daemon = True
        self.dvl_thread.start()

        self.fc_thread = threading.Thread(target=self.read_fc_data)
        self.fc_thread.daemon = True
        self.fc_thread.start()

        self.altitude_control_thread = threading.Thread(target=self.altitude_control_loop)
        self.altitude_control_thread.daemon = True
        self.altitude_control_thread.start()

        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.send_thread = threading.Thread(target=self.send_loop)
        self.send_thread.daemon = True
        self.send_thread.start()

    def get_initial_heading(self, timeout=5.0):
        """启动时读取一次飞控当前航向，作为初始目标航向。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
                if msg and getattr(msg, 'heading', None) is not None:
                    self.current_heading = float(msg.heading)
                    print(f"Initial heading locked: {self.current_heading:.1f}°")
                    return self.current_heading
            except Exception as e:
                print(f"Read initial heading error: {e}")
                break

        print("Initial heading unavailable, fallback to 0.0°")
        return 0.0

    def reset_dvl_dead_reckoning(self):
        """重置DVL航位推算"""
        try:
            # 发送重置命令
            reset_cmd = '{"command": "reset_dead_reckoning"}\r\n'
            self.dvl_socket.sendall(reset_cmd.encode('utf-8'))
            # print("Sent DVL reset command")

            # 等待响应
            response = self.dvl_socket.recv(1024).decode()
            # print(f"DVL reset response: {response}")

            # 等待50ms让DVL处理
            time.sleep(0.05)
            self.last_reset_time = time.time()
            return True
        except Exception as e:
            print(f"Error resetting DVL: {e}")
            return False

    def set_mode(self, mode):
        """设置飞行模式"""
        mode_id = self.master.mode_mapping().get(mode)
        if mode_id is None:
            print(f"Unknown mode: {mode}")
            return False
        self.master.set_mode(mode_id)
        print(f"Mode set to {mode}")
        return True

    def set_stabilize(self):
        """设置飞行模式"""

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0)

        return True

    def set_depth_hold(self):
        """设置飞行模式"""

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            2)

        return True

    def set_channel(self, channel_index, value):
        """设置通道值"""
        self.channel_values[channel_index] = max(1100, min(int(value), 1900))

    def enable_depth_control(self):
        """启用深度控制"""
        self.altitude_pid.reset()
        self.depth_control_enabled = True

        print("Depth control enabled")

    def disable_depth_control(self):
        """禁用深度控制"""
        self.depth_control_enabled = False
        print("Depth control disabled")

        # 重置PID控制器以避免积分饱和
        self.altitude_pid = PIDController(
            Kp=300.0,
            Ki=5.0,
            Kd=10,
            output_min=-150,
            output_max=150
        )
        self.set_channel(2, 1500)  # 设置油门为中性值

    def set_manual_channel(self, channel_name, value):
        """设置手动控制通道值"""
        if channel_name == 'throttle':
            self.manual_throttle = value
            self.set_channel(2, value)
        elif channel_name == 'yaw':
            self.manual_yaw = value
            self.set_channel(3, value)
        elif channel_name == 'forward':
            self.manual_forward = value
            self.set_channel(4, value)
        print(f"Set {channel_name} to {value}")

    def set_target_attitude(self, roll=None, pitch=None, yaw=None):

        if roll is not None:
            self.target_attitude['roll'] = roll
        if pitch is not None:
            self.target_attitude['pitch'] = pitch
        if yaw is not None:
            self.target_attitude['yaw'] = yaw
        '''
        print(f"Set target attitude: Roll={self.target_attitude['roll']}°, "
              f"Pitch={self.target_attitude['pitch']}°, "
              f"Yaw={self.target_attitude['yaw']}°")
        '''

    def altitude_control_loop(self):
        """独立的高度控制线程"""
        while self.running:
            # 检查深度控制是否启用
            if not self.depth_control_enabled:
                time.sleep(0.05)

                continue

            # 检查是否有高度数据
            if not hasattr(self, 'current_depth'):
                time.sleep(0.1)
                continue

            try:

                current_d = -self.current_depth
                # 使用PID控制器计算油门调整量

                pid_output = self.altitude_pid.compute(self.target_depth, current_d)

                # 计算油门值 (1500为中性点)
                throttle_value = 1500 + pid_output

                # 设置油门通道
                self.set_channel(2, throttle_value)

                # 打印调试信息
                # print(f"Altitude Control: target={self.target_depth:.2f}m, current={current_d:.2f}m, PID={pid_output:.1f}, throttle={throttle_value}")
            except Exception as e:
                print(f"Altitude control error: {e}")

            # 以固定频率运行 (20Hz)
            # time.sleep(0.05)
            sleep_ms(50)

    def set_target_depth(self, depth):
        """设置目标深度（米，正数表示水下深度）"""
        # self.target_altitude = depth
        self.target_depth = depth
        # print(f"Set target depth: {depth} m")

    def read_dvl_data(self):
        """从DVL读取位置数据"""
        buffer = ""
        while self.running:
            try:
                data = self.dvl_socket.recv(4096).decode()
                if not data:
                    time.sleep(0.1)
                    continue

                buffer += data
                if "\r\n" not in buffer:
                    continue

                messages = buffer.split("\r\n")
                buffer = messages.pop()  # 保存不完整的剩余部分

                for msg in messages:
                    if not msg:
                        continue

                    try:
                        report = json.loads(msg)
                        msg_type = report.get("type")
                        if msg_type == "position_local":
                            with self.position_lock:
                                self.current_position[0] = report['x']
                                self.current_position[1] = report['y']
                        '''        
                        elif msg_type == "velocity":
                            with self.altitude_lock:  # 确保线程安全
                                self.current_altitude = report['altitude']
                        '''
                    except json.JSONDecodeError:
                        pass
            except Exception as e:
                print(f"DVL error: {e}")
                time.sleep(1)

    def read_fc_data(self):
        """从飞控读取深度和航向数据"""
        while self.running:
            try:
                # 获取VFR_HUD消息，包含深度和航向
                msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
                if msg:
                    self.current_heading = msg.heading  # 航向(度, 0-360)
                msg = self.master.recv_match(type='AHRS2', blocking=True, timeout=1.0)
                if msg:
                    self.current_depth = msg.altitude  # 深度(米)
                    # print(f"current={self.current_depth:.2f}m")
                    # print()
                # 确保我们获取姿态消息以保持连接
                self.master.recv_match(type='ATTITUDE', blocking=False)
                time.sleep(0.05)
            except Exception as e:
                print(f"FC error: {e}")
                time.sleep(1)

    def normalize_angle(self, angle):
        """将角度归一化到-180到180度范围内"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def control_loop(self):
        """主控制循环"""

        while self.running:

            # self.depth_control()
            if self.control_mode == "auto":
                # 自动巡线模式
                self.auto_control()
            else:
                # 手动模式 - 使用用户设置的通道值
                # self.set_channel(2, self.manual_throttle)
                self.set_channel(3, self.manual_yaw)
                self.set_channel(4, self.manual_forward)

            time.sleep(0.05)  # 控制循环频率

    def auto_control(self):
        """自动巡线控制逻辑 - 简化版: 调整航向 -> 调整深度 -> 重置DVL -> 前进指定距离"""
        # 检查是否有路径点
        if not self.waypoints_use or self.current_waypoint_index >= len(self.waypoints_use):
            return

        # 获取当前航段参数
        # target_heading, target_distance, target_depth = self.waypoints[self.current_waypoint_index]

        wp_raw = self.waypoints_use[self.current_waypoint_index]
        if isinstance(wp_raw, tuple):
            wp = {"h": wp_raw[0], "d": wp_raw[1], "z": wp_raw[2], "tasks": []}
        else:
            wp = wp_raw

        target_heading = wp["h"]
        target_distance = wp["d"]
        target_depth = wp["z"]
        tasks = wp.get("tasks", [])

        self.set_target_depth(target_depth)
        # 根据航段状态执行不同操作
        if self.segment_state == "adjust_depth":
            # 调整深度阶段
            depth_error = abs(target_depth - (-self.current_depth))

            if depth_error < self.depth_threshold:
                print(f"Depth adjusted: {self.current_depth:.2f}m -> {target_depth:.2f}m")
                time.sleep(1)
                self.segment_state = "adjust_heading"

        elif self.segment_state == "adjust_heading":

            now = time.time()
            # 调整航向阶段

            if self.adjust_start_time is None:
                self.adjust_start_time = now

            self.set_target_attitude(yaw=target_heading)
            # 检查航向是否调整到位
            heading_error = abs(self.normalize_angle(target_heading - self.current_heading))

            reached = heading_error < self.heading_threshold
            timed_out = (now - self.adjust_start_time) > 8.0
            if reached or timed_out:
                self.adjust_start_time = None
                time.sleep(1)
                self.segment_state = "reset_dvl"

        elif self.segment_state == "reset_dvl":
            # 重置DVL阶段
            if self.reset_dvl_dead_reckoning():
                self.segment_target_heading = target_heading
                self.segment_target_distance = target_distance
                self.segment_progress = 0.0
                self.segment_state = "move_forward"
                print(f"Starting forward movement: {target_distance:.2f}m at {target_heading:.1f}°")
                time.sleep(1)
            else:
                print("DVL reset failed, retrying...")

        elif self.segment_state == "move_forward":
            # 前进阶段
            now = time.time()

            if self.move_start_time is None:
                self.move_start_time = now
                self.boost_fwd = 1500  # 前进累加
                self.boost_rev = 1500  # 后退累加
            with self.position_lock:
                self.segment_progress = self.current_position[0]

            # 计算剩余距离
            remaining = self.segment_target_distance - self.segment_progress

            base = 1500 + self.Kp_forward * remaining
            # forward_value = max(1420, min(int(base), 1560))   # 不卡时的上限 1560

            # ------------ 2. 卡死累加逻辑 ------------

            if abs(remaining - self.last_remaining) < 0.05:  # 距离几乎没变
                if not self.is_stuck:
                    self.is_stuck = True
                    self.stuck_timer_start = now

                if now - self.stuck_timer_start >= 3.0:

                    if remaining > 0:  # 还需前进
                        self.boost_fwd = min(self.boost_fwd + 300, 1800)
                        forward_value = max(1420, int(self.boost_fwd))
                    else:  # 还需后退
                        self.boost_rev = max(self.boost_rev - 200, 1000)
                        forward_value = min(1580, int(self.boost_rev))  # 后退 PWM 范围自己设

                    self.stuck_timer_start = now  # 重新计时
                else:
                    forward_value = max(1420, min((base), 1560))


            else:
                # 真正在前进：退出卡死状态，复位推力
                self.is_stuck = False
                self.boosted_thrust = 1560
                forward_value = max(1430, min((base), 1560))

            self.last_remaining = remaining
            self.set_channel(4, forward_value)

            # 打印状态
            print(f"Moving forward: {self.segment_progress:.2f}/{self.segment_target_distance:.2f}m "
                  f"(remaining: {remaining:.2f}m)" f"(pwm: {forward_value}) ")
            timed_out = (now - self.move_start_time) > 20.0
            reached = abs(remaining) <= self.waypoint_threshold
            # 检查是否到达目标距离
            if reached or timed_out:

                if timed_out:
                    print("Forward timeout, skip to next waypoint")
                else:
                    print(f"Reached waypoint {self.current_waypoint_index + 1}")
                self.move_start_time = None
                self.set_channel(4, 1500)  # 停止前进

                if tasks:
                    self.segment_state = "do_tasks"
                    self.current_task_index = 0
                    self.task_done = False
                else:
                    self._next_waypoint()


        elif self.segment_state == "do_tasks":
            task = tasks[self.current_task_index]
            if not self.task_done:
                print(f"Start task: {task}")
                # 根据任务名调用已实现的函数
                if task == "roll":

                    self.do_roll_task()

                    # self.up_task()
                elif task == "turn":
                    self.turn_task()
                elif task == "photo":
                    time.sleep(0.5)
                    snap()
                elif task == "mod":
                    self.up_task()
                elif task == "a":
                    self.a()
                    self.ppp_task()
                elif task == "b":
                    self.b()
                    self.ppp_task()
                elif task == "c":
                    self.c()
                    self.ppp_task()

                else:
                    print(f"Unknown task: {task}")
                self.task_done = True

            self.task_done = False
            self.current_task_index += 1

            if self.current_task_index >= len(tasks):
                # 该航点所有任务完成
                self._next_waypoint()

    def _next_waypoint(self):
        self.current_waypoint_index += 1
        self.segment_state = "adjust_depth"
        if self.current_waypoint_index >= len(self.waypoints_use):
            self.ok = 1
            print("Mission completed!")
            self.waypoints_use = []
            self.current_waypoint_index = 0

    def do_roll_task(self):

        self.disable_depth_control()
        time.sleep(1)
        pitch_angle = 0
        yaw_angle = 10
        for roll_angle in range(0, 730, 10):  # 0→720°
            self.set_target_attitude(roll=roll_angle)
            time.sleep(0.1)

        time.sleep(1)
        self.enable_depth_control()

    def turn_task(self):
        for yaw_angle in range(0, 100, 10):
            self.set_target_attitude(yaw=yaw_angle)
            time.sleep(0.5)

    def ppp_task(self):
        self.set_target_attitude(pitch=-20)
        time.sleep(2)
        self.set_target_attitude(pitch=0)
        time.sleep(1)

    def a(self):
        a_jpg()
        time.sleep(0.5)

    def b(self):
        b_jpg()
        time.sleep(0.5)

    def c(self):
        c_jpg()
        time.sleep(0.5)

    def up_task(self):
        self.set_target_depth(0.5)

        time.sleep(15)

    def send_loop(self):
        """发送控制指令到飞控"""
        while self.running:
            try:

                # 发送姿态目标
                if self.control_mode == "auto":
                    q = QuaternionBase([
                        math.radians(self.target_attitude['roll']),
                        math.radians(self.target_attitude['pitch']),
                        math.radians(self.target_attitude['yaw'])
                    ])
                    self.master.mav.set_attitude_target_send(
                        int(1e3 * (time.time() - self.boot_time)),
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                        q, 0, 0, 0, 0
                    )

                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *self.channel_values
                )

                if self.ok == 1:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        0, 0, 0, 0, 0, 0, 0)

                time.sleep(0.05)  # 20Hz
            except Exception as e:
                print(f"Send error: {e}")
                time.sleep(1)

    def stop_movement(self):
        """停止所有运动"""
        self.set_channel(2, 1500)  # 油门
        self.set_channel(3, 1500)  # 偏航
        self.set_channel(4, 1500)  # 前进
        self.manual_throttle = 1500
        self.manual_yaw = 1500
        self.manual_forward = 1500
        print("All movement stopped")

    def set_waypoints(self, waypoints):
        """设置路径点列表"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.segment_state = "adjust_heading"  # 重置状态
        print(f"Set {len(waypoints)} waypoints")

    def interactive_control(self):
        """交互式控制界面"""
        while self.running:
            self.print_menu()
            cmd = input("输入指令: ").strip().lower()

            if cmd == 'q':
                self.running = False
                self.stop_movement()
                print("退出系统")
                break

            elif cmd == 'g':  # 停止
                self.stop_movement()

            elif cmd == 'm':  # 切换控制模式
                self.control_mode = "auto" if self.control_mode == "manual" else "manual"
                if self.control_mode == "auto":
                    self.enable_depth_control()
                else:
                    self.disable_depth_control()
                print(f"切换至{'自动' if self.control_mode == 'auto' else '手动'}模式")
                self.stop_movement()

            elif self.control_mode == "manual":
                self.handle_manual_command(cmd)
            else:
                self.handle_auto_command(cmd)

    def print_menu(self):
        """打印当前模式的控制菜单"""
        print("\n===== AUV 高级控制系统 =====")
        print(f"当前模式: {'自动巡线' if self.control_mode == 'auto' else '手动控制'}")

        if self.control_mode == "manual":
            print("推进器控制:")
            print("  w - 上升       s - 下降")
            print("  a - 左转       d - 右转")
            print("  e - 前进       c - 后退")
            print("姿态控制:")
            print("  i - 俯仰+10°   k - 俯仰-10°")
            print("  j - 滚转-10°   l - 滚转+10°")
            print("  u - 偏航-10°   o - 偏航+10°")
            print("  r - 重置姿态")
            print("深度控制:")
            print("  z - 设置深度为1米")
            print("  x - 设置深度为0.5米")
            print("  v - 设置深度为2米")
            print("  b - 自定义深度")
        else:
            print("巡线控制:")
            print("  s - 设置路径点")
            print("  c - 清除路径点")
            print("  d - 设置目标深度")
            print("  e - 开始巡线任务")

        print("通用控制:")
        print("  g - 停止所有运动")
        print("  m - 切换手动/自动模式")
        print("  p - 自定义PWM值")
        print("  q - 退出")

    def handle_manual_command(self, cmd):
        """处理手动模式命令"""
        # 推进器控制
        # 深度控制 (始终进行)

        if cmd == 'w':  # 上升
            self.set_manual_channel('throttle', 1700)
            print("上升中... (按g停止)")

        elif cmd == 's':  # 下降
            self.set_manual_channel('throttle', 1450)
            print("下降中... (按g停止)")

        elif cmd == 'a':  # 左转
            self.set_manual_channel('yaw', 1300)
            print("左转中... (按g停止)")

        elif cmd == 'd':  # 右转
            self.set_manual_channel('yaw', 1700)
            print("右转中... (按g停止)")

        elif cmd == 'e':  # 前进
            self.set_manual_channel('forward', 1700)
            print("前进中... (按g停止)")

        elif cmd == 'c':  # 后退
            self.set_manual_channel('forward', 1300)
            print("后退中... (按g停止)")

        # 姿态控制
        elif cmd == 'i':  # 俯仰+
            self.set_target_attitude(pitch=self.target_attitude['pitch'] + 10)

        elif cmd == 'k':  # 俯仰-
            self.set_target_attitude(pitch=self.target_attitude['pitch'] - 10)

        elif cmd == 'j':  # 滚转-
            self.set_target_attitude(roll=self.target_attitude['roll'] - 10)

        elif cmd == 'l':  # 滚转+
            self.set_target_attitude(roll=self.target_attitude['roll'] + 10)

        elif cmd == 'u':  # 偏航-
            self.set_target_attitude(yaw=self.target_attitude['yaw'] - 10)

        elif cmd == 'o':  # 偏航+
            self.set_target_attitude(yaw=self.target_attitude['yaw'] + 10)

        elif cmd == 'r':  # 重置姿态
            self.set_target_attitude(0, 0, 0)
        elif cmd == '=':  # 重置姿态
            img_path = snap()  # 拍一张，返回绝对路径
            print("已保存：", img_path)
        # 深度控制
        elif cmd == 'z':  # 1米深度
            self.mo = 0
        elif cmd == 'x':  # 0.1米深度
            self.mo = 2




        elif cmd == 'b':  # 自定义深度
            try:
                depth = float(input("输入目标深度（米）: "))
                self.target_depth = depth
                print(f"目标深度设置为{depth}米")
            except ValueError:
                print("无效输入! 请输入数字")

        # 自定义控制
        elif cmd == 'p':  # 自定义PWM值
            try:
                throttle = int(input("油门(1100-1900, 1500=停止): "))
                yaw = int(input("偏航(1100-1900, 1500=停止): "))
                forward = int(input("前进(1100-1900, 1500=停止): "))

                self.set_manual_channel('throttle', throttle)
                self.set_manual_channel('yaw', yaw)
                self.set_manual_channel('forward', forward)

                print(f"已设置: T={throttle}, Y={yaw}, F={forward}")
            except ValueError:
                print("无效输入! 请输入整数")

    def handle_auto_command(self, cmd):
        """处理自动模式命令"""
        if cmd == 's':  # 设置路径点
            try:
                count = int(input("输入路径点数量: "))
                waypoints = []
                for i in range(count):
                    print(f"路径点 {i + 1}:")
                    heading = float(input("  目标航向(度): "))
                    distance = float(input("  前进距离(米): "))
                    depth = float(input("  深度(米): "))
                    waypoints.append((heading, distance, depth))
                self.set_waypoints(waypoints)
            except ValueError:
                print("无效输入!")

        elif cmd == 'c':  # 清除路径点
            self.waypoints = []
            self.current_waypoint_index = 0
            self.segment_state = "adjust_heading"
            print("路径点已清除")

        elif cmd == 'd':  # 设置目标深度
            try:
                depth = float(input("输入目标深度(米): "))
                self.target_depth = depth
                print(f"目标深度设置为 {depth} 米")
            except ValueError:
                print("无效输入!")

        elif cmd == 'e':  # 开始巡线
            if self.waypoints:
                print("开始巡线任务...")
                # 重置状态以启动任务
                self.segment_state = "adjust_heading"
            else:
                print("请先设置路径点!")


if __name__ == "__main__":
    for i in range(5, 0, -1):
        print(f'\r倒计时 {i} 秒', end='', flush=True)
        time.sleep(1)
    print('\r开始了！     ')
    auv = AdvancedAUVControl()
    try:
        auv.interactive_control()
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        auv.running = False
        time.sleep(1)  # 给线程时间退出
        print("系统关闭")

