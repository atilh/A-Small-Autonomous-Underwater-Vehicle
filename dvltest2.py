import copy
import logging
import queue
from collections import deque
import json
import math
import socket
import threading
import time

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from pai import snap
from ph import a_jpg, b_jpg, c_jpg
from ticop import sleep_ms

PWM_NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900
PWM_IGNORE = 65535

THROTTLE_CHANNEL = 2
YAW_CHANNEL = 3
FORWARD_CHANNEL = 4

# 通道名 -> (RC 数组下标, 手动控制状态属性)。
MANUAL_CHANNELS = {
    "throttle": (THROTTLE_CHANNEL, "manual_throttle"),
    "yaw": (YAW_CHANNEL, "manual_yaw"),
    "forward": (FORWARD_CHANNEL, "manual_forward"),
}

# 按键 -> (通道名, PWM,)。
MOVEMENT_COMMANDS = {
    "w": ("throttle", 1700, "上升中... (按g停止)"),
    "s": ("throttle", 1450, "下降中... (按g停止)"),
    "a": ("yaw", 1300, "左转中... (按g停止)"),
    "d": ("yaw", 1700, "右转中... (按g停止)"),
    "e": ("forward", 1700, "前进中... (按g停止)"),
    "c": ("forward", 1300, "后退中... (按g停止)"),
}

# 按键 -> (姿态轴, 角度增量)。
ATTITUDE_COMMANDS = {
    "i": ("pitch", 10),
    "k": ("pitch", -10),
    "j": ("roll", -10),
    "l": ("roll", 10),
    "u": ("yaw", -10),
    "o": ("yaw", 10),
}


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
        # 回退累计积分，但本次输出仍使用回退前的 integral。
        if integral > self.inte_max or integral < self.inte_min:
            self.integral -= error * dt
        # 微分项
        derivative = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error

        # 计算总输出
        output = proportional + integral + derivative

        # 输出限幅
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        return output


class _DvlResetRequest:
    def __init__(self, deadline, cancel_event):
        self.deadline = deadline
        self.cancel_event = cancel_event
        self.cancelled = threading.Event()
        self.done = threading.Event()
        self.result = False
        self.sent = False
        self.ack_time = None


class DvlClient:
    """负责重连 TCP 流及序列化重置请求的读取器。

    回调函数的执行不依赖内部锁。回调必须迅速返回，且不得同步等待来自读取器线程的 request_reset() 调用。
    on_disconnect 还会使格式错误或异常的位置报告失效；
    connected 字段描述传输连接状态，而 last_position 描述数据状态。
    时间戳值为本地单调递增的接收时间，而非设备采样时间。
    """

    MAX_FRAME_BYTES = 65536
    RESET_SETTLE_SECONDS = 0.05  # Official API documents approximately 50 ms.

    def __init__(self, host, port, on_position, on_disconnect,
                 clock=time.monotonic, socket_factory=socket.socket,
                 io_timeout=0.2, reconnect_delay=1.0, reset_timeout=3.0):
        for name, value in (("io_timeout", io_timeout),
                            ("reconnect_delay", reconnect_delay),
                            ("reset_timeout", reset_timeout)):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(name + " must be finite")
            if value < 0 or (name != "reconnect_delay" and value == 0):
                raise ValueError(name + " has an invalid interval")
        self.host, self.port = host, port
        self.on_position, self.on_disconnect = on_position, on_disconnect
        self.clock, self.socket_factory = clock, socket_factory
        self.io_timeout = io_timeout
        self.reconnect_delay = reconnect_delay
        self.reset_timeout = reset_timeout
        self.connected = threading.Event()
        self._closed = threading.Event()
        self._state_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._run_lock = threading.Lock()
        self._requests = queue.Queue()
        self._socket = None
        self._last_position = None
        self._last_reset_position = None
        self._stop_event = None
        self._reader_id = None
        self._active_reset = None  # Only the reader changes active request state.
        self._buffer = b""
        self._retry_after = 0.0

    @property
    def socket(self):
        """仅限传输检查；不支持外部接收/发送"""
        with self._state_lock:
            return self._socket

    @property
    def last_position(self):
        with self._state_lock:
            return self._last_position

    @property
    def last_reset_position(self):
        """最近一次成功重置后，首个有效的 ACK 后位置"""
        with self._state_lock:
            return self._last_reset_position

    def _stopping(self):
        return self._closed.is_set() or (
            self._stop_event is not None and self._stop_event.is_set())

    def _notify_invalid(self, reason):
        try:
            self.on_disconnect(reason)
        except Exception:
            logging.exception("DVL invalidation callback failed")

    def _invalidate_position(self, reason):
        with self._state_lock:
            self._last_position = None
        self._notify_invalid(reason)

    def _disconnect(self, reason):
        with self._state_lock:
            sock = self._socket
            self._socket = None
            self._last_position = None
            was_connected = self.connected.is_set()
            self.connected.clear()
            self._retry_after = self.clock() + self.reconnect_delay
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        if sock is not None or was_connected:
            self._notify_invalid(reason)

    def close(self):
        """永久停止此客户端；可安全地重复调用或从回调中调用"""
        self._closed.set()
        self._disconnect("DVL client closed")

    def _wait(self, delay):
        deadline = self.clock() + delay
        while not self._stopping():
            remaining = deadline - self.clock()
            if remaining <= 0:
                return
            self._closed.wait(min(remaining, 0.05))

    def _connect(self):
        self._wait(max(0.0, self._retry_after - self.clock()))
        if self._stopping():
            return
        sock = self.socket_factory(socket.AF_INET, socket.SOCK_STREAM)
        with self._state_lock:
            if self._stopping():
                sock.close()
                return
            self._socket = sock
        sock.settimeout(self.io_timeout)
        sock.connect((self.host, self.port))
        with self._state_lock:
            if self._socket is sock and not self._stopping():
                self._buffer = b""
                self.connected.set()

    @staticmethod
    def _finish(request, result):
        request.result = result
        request.done.set()

    def _request_expired(self, request):
        return (request.cancelled.is_set() or self._stopping()
                or (request.cancel_event is not None
                    and request.cancel_event.is_set())
                or self.clock() >= request.deadline)

    def request_reset(self, timeout=None, cancel_event=None):
        """等待匹配成功的 ACK 响应以及随后的有效位置信息。

        超时时间包含排队等待其他请求处理的时间。对于发送后失败或被取消的指令，其 TCP 连接会被丢弃，
        因为 API 不包含请求 ID；必须确保不会将迟到的响应误认为是针对后续重置请求的响应。
        """

        timeout = self.reset_timeout if timeout is None else timeout
        if (not isinstance(timeout, (int, float)) or not math.isfinite(timeout)
                or timeout <= 0 or threading.get_ident() == self._reader_id):
            return False
        request = _DvlResetRequest(self.clock() + timeout, cancel_event)
        acquired = False
        try:
            while not self._request_expired(request):
                acquired = self._request_lock.acquire(timeout=0.02)
                if acquired:
                    break
            if not acquired or self._request_expired(request):
                return False
            self._requests.put(request)
            while not request.done.wait(0.02):
                if self._request_expired(request):
                    request.cancelled.set()
                    return False
            return request.result and not self._request_expired(request)
        finally:
            if acquired:
                self._request_lock.release()

    def _service_reset(self):
        request = self._active_reset
        if request is not None and self._request_expired(request):
            if request.sent:
                self._disconnect("DVL reset cancelled or timed out")
                self._buffer = b""
            self._finish(request, False)
            self._active_reset = None
        if self._active_reset is not None or not self.connected.is_set():
            return
        try:
            request = self._requests.get_nowait()
        except queue.Empty:
            return
        if self._request_expired(request):
            self._finish(request, False)
            return
        self._active_reset = request
        with self._state_lock:
            self._last_position = None
            sock = self._socket
        request.sent = True  # A partial send is also an ambiguous transaction.
        if sock is None:
            raise ConnectionError("DVL disconnected before reset")
        sock.sendall(b'{"command":"reset_dead_reckoning"}\r\n')

    @staticmethod
    def _finite_number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        try:
            return math.isfinite(value)
        except (OverflowError, TypeError, ValueError):
            return False

    def _handle_report(self, report):
        if not isinstance(report, dict):
            return
        request = self._active_reset
        if (report.get("type") == "response"
                and report.get("response_to") == "reset_dead_reckoning"
                and request is not None and request.sent):
            if self._request_expired(request):
                self._service_reset()
            elif (report.get("success") is True
                  and report.get("error_message") == ""):
                if request.ack_time is None:
                    request.ack_time = self.clock()
            else:
                self._disconnect("DVL rejected reset: "
                                 + str(report.get("error_message", "invalid ACK")))
                self._buffer = b""
                self._finish(request, False)
                self._active_reset = None
            return
        if report.get("type") != "position_local":
            return
        x, y = report.get("x"), report.get("y")
        if (type(report.get("status")) is not int or report["status"] != 0
                or not self._finite_number(x) or not self._finite_number(y)):
            self._invalidate_position("DVL position is invalid or status is nonzero")
            return
        now = self.clock()
        if request is not None:
            if self._request_expired(request):
                self._service_reset()
                return
            if (request.ack_time is None
                    or now - request.ack_time < self.RESET_SETTLE_SECONDS):
                return
        position = (float(x), float(y), now)
        with self._state_lock:
            if self._stopping() or not self.connected.is_set():
                return
            self._last_position = position
        self.on_position(*position)
        if request is not None:
            if self._request_expired(request):
                self._service_reset()
                return
            with self._state_lock:
                self._last_reset_position = position
            self._finish(request, True)
            self._active_reset = None

    def _consume(self, chunk):
        self._buffer += chunk
        while b"\n" in self._buffer:
            frame, self._buffer = self._buffer.split(b"\n", 1)
            if len(frame) > self.MAX_FRAME_BYTES:
                raise ValueError("DVL frame exceeds size limit")
            if not frame.strip():
                continue
            try:
                report = json.loads(frame.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                self._invalidate_position("DVL malformed JSON report")
                continue
            self._handle_report(report)
            if not self.connected.is_set() or self._stopping():
                self._buffer = b""
                return
        if len(self._buffer) > self.MAX_FRAME_BYTES:
            raise ValueError("DVL unfinished frame exceeds size limit")

    def run(self, stop_event):
        """在应用程序的 DVL 读取器线程中运行。切勿同时调用两次"""
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("DVL reader is already running")
        self._stop_event = stop_event
        self._reader_id = threading.get_ident()
        try:
            while not self._stopping():
                try:
                    self._service_reset()
                    if not self.connected.is_set():
                        self._connect()
                    if self._stopping():
                        break
                    self._service_reset()
                    with self._state_lock:
                        sock = self._socket
                    if sock is None:
                        continue
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("DVL closed the TCP connection")
                    self._consume(chunk)
                except Exception as exc:
                    self._disconnect("DVL connection/read failure: " + str(exc))
                    self._buffer = b""
                    if self._active_reset is not None:
                        self._finish(self._active_reset, False)
                        self._active_reset = None
        finally:
            self.close()
            if self._active_reset is not None:
                self._finish(self._active_reset, False)
                self._active_reset = None
            while True:
                try:
                    self._finish(self._requests.get_nowait(), False)
                except queue.Empty:
                    break
            self._reader_id = None
            self._run_lock.release()


class MavlinkIO:
    """一个遥测读取者；所有 MAVLink 读写通过同一把锁串行访问。"""

    def __init__(self, master, factory=None):
        self.master = master
        self.factory = factory
        self._lock = threading.RLock()

    def poll(self, limit=64):
        messages = []
        with self._lock:
            for _ in range(limit):
                message = self.master.recv_match(blocking=False)
                if message is None:
                    break
                messages.append(message)
        return messages

    def reconnect(self):
        if self.factory is None:
            return False
        with self._lock:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = self.factory()
        return True

    def set_custom_mode(self, mode_id):
        with self._lock:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
        return True

    def set_named_mode(self, mode):
        with self._lock:
            mode_id = self.master.mode_mapping().get(mode)
            if mode_id is None:
                return False
            self.master.set_mode(mode_id)
        return True

    def send_arm_command(self, armed):
        with self._lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, armed, 0, 0, 0, 0, 0, 0,
            )

    def send_attitude(self, attitude, elapsed):
        quaternion = QuaternionBase([
            math.radians(attitude["roll"]),
            math.radians(attitude["pitch"]),
            math.radians(attitude["yaw"]),
        ])
        with self._lock:
            self.master.mav.set_attitude_target_send(
                int(1e3 * elapsed),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                quaternion, 0, 0, 0, 0,
            )

    def send_channels(self, channel_values):
        with self._lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *channel_values,
            )

    def close(self):
        with self._lock:
            self.master.close()


class TaskCancelled(Exception):
    """旧任务被停止、替换或关闭，不作为工作线程故障。"""


class AdvancedAUVControl:
    """集中管理连接、状态、停车和任务生命周期。

    嵌套锁顺序固定为 _send_lock -> _state_lock。状态锁内不执行网络、
    等待、拍照或线程 join。start=False 仅用于注入模拟设备的离线测试。
    """

    SENSOR_TIMEOUT = 1.0
    HEARTBEAT_TIMEOUT = 3.0
    STARTUP_TIMEOUT = 10.0
    ARM_TIMEOUT = 5.0
    CONTROL_PERIOD = 0.05
    STUCK_WINDOW = 3.0
    STUCK_DISTANCE = 0.05

    def __init__(self, master=None, dvl_client=None, start=True, clock=None):
        self._clock = clock or time.monotonic
        self._state_lock = threading.RLock()
        self._send_lock = threading.RLock()
        self._reset_lock = threading.Lock()
        self._shutdown_event = threading.Event()
        self._fc_reconnect_event = threading.Event()
        self._cancel_event = threading.Event()
        self._task_context = threading.local()
        self._workers = {}
        self._workers_started = False
        self._closing = False
        self._initialized = not start
        self._stopped = True
        self._fault_reason = None
        self._fault_recoverable = True
        self._mission_running = False
        self._mission_epoch = 0
        self._armed = False
        self._arm_requested = False
        self._mavlink_io = MavlinkIO(master) if master is not None else None
        self._dvl_client = dvl_client

        self.dvl_ip = "192.168.137.3"
        self.dvl_port = 16171
        self.current_position = [0.0, 0.0]
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_altitude = 0.0
        self.position_lock = self._state_lock
        self.altitude_lock = self._state_lock
        self.dvl_timestamp = None
        self.depth_timestamp = None
        self.heading_timestamp = None
        self.heartbeat_timestamp = None
        self.channel_values = [PWM_IGNORE] * 8
        self._neutral_locked()
        self.target_depth = 0.1
        self.target_altitude = 0.1
        self.move_start_time = None
        self.adjust_start_time = None
        self.last_remaining = 0
        self.boost_level = 1560
        self.boosted_thrust = 1560
        self.altitude_pid = PIDController(500.0, 5.0, 0, -150, 150)
        self.depth_pid = PIDController(500.0, 10.0, 0, -200, 200)
        self.waypoints = []
        self.waypoints_use = []
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.10
        self.current_task_index = 0
        self.task_done = False
        self.segment_state = "adjust_heading"
        self.segment_progress = 0.0
        self.segment_target_heading = 0.0
        self.segment_target_distance = 0.0
        self.segment_origin_x = 0.0
        self.last_reset_time = 0
        self.H = "hold"
        self.mo = 0
        self.Kp_depth = 80.0
        self.Kp_yaw = 2.0
        self.Kp_forward = 230
        self.heading_threshold = 3.0
        self.depth_threshold = 0.1
        self.stuck_timer_start = 0
        self.is_stuck = False
        self._stuck_samples = deque()
        self.depth_control_enabled = False
        self.manual_throttle = PWM_NEUTRAL
        self.manual_yaw = PWM_NEUTRAL
        self.manual_forward = PWM_NEUTRAL
        self.begin = 0.0
        self.target_attitude = {"roll": 0, "pitch": 0, "yaw": self.begin}
        self.boost_fwd = PWM_NEUTRAL
        self.boost_rev = PWM_NEUTRAL
        self.ok = 0
        self.running = True
        self.boot_time = self._clock()
        self.control_mode = "manual"

        if start:
            try:
                self._startup()
            except BaseException:
                self.close()
                raise

    @property
    def master(self):
        return self._mavlink_io.master if self._mavlink_io else None

    @master.setter
    def master(self, connection):
        with self._send_lock:
            self._mavlink_io = MavlinkIO(connection)

    @property
    def dvl_socket(self):
        return self._dvl_client.socket if self._dvl_client else None

    def _startup(self):
        if self._mavlink_io is None:
            factory = lambda: mavutil.mavlink_connection("udpin:0.0.0.0:14553")
            self._mavlink_io = MavlinkIO(factory(), factory=factory)
        if self._dvl_client is None:
            self._dvl_client = DvlClient(
                self.dvl_ip, self.dvl_port, self._on_position, self._on_dvl_disconnect,
                clock=self._clock,
            )
        self._start_worker("dvl_thread", self.read_dvl_data)
        self._start_worker("fc_thread", self.read_fc_data)
        self._wait_ready(self.STARTUP_TIMEOUT)
        with self._state_lock:
            self.begin = self.current_heading
            self.target_attitude["yaw"] = self.begin
            self.waypoints = [{"h": self.begin, "d": 0.5, "z": 0.2, "tasks": []}]
        print(f"Initial heading locked: {self.begin:.1f}°")
        self._send_neutral()
        self.set_stabilize()
        self._arm_vehicle()
        with self._state_lock:
            self._initialized = True
            self._stopped = False
        self._start_worker("altitude_control_thread", self.altitude_control_loop)
        self._start_worker("control_thread", self.control_loop)
        self._start_worker("send_thread", self.send_loop)
        self._workers_started = True
        print("AUV ready: manual mode")

    def _start_worker(self, name, target):
        worker = threading.Thread(target=self._worker_entry, args=(name, target), daemon=True)
        self._workers[name] = worker
        setattr(self, name, worker)
        worker.start()

    def _worker_entry(self, name, target):
        try:
            target()
        except Exception as error:
            self._enter_fault(f"{name} failed: {error}", recoverable=False)
        else:
            if not self._shutdown_event.is_set() and not self._closing:
                self._enter_fault(f"{name} exited unexpectedly", recoverable=False)

    def _missing_data_locked(self, now, need_dvl=True):
        sources = [("heading", self.heading_timestamp, self.SENSOR_TIMEOUT),
                   ("depth", self.depth_timestamp, self.SENSOR_TIMEOUT),
                   ("heartbeat", self.heartbeat_timestamp, self.HEARTBEAT_TIMEOUT)]
        if need_dvl:
            sources.append(("DVL", self.dvl_timestamp, self.SENSOR_TIMEOUT))
        return [name for name, stamp, timeout in sources
                if stamp is None or not 0 <= now - stamp <= timeout]

    def _wait_ready(self, timeout):
        deadline = self._clock() + timeout
        while not self._shutdown_event.is_set():
            with self._state_lock:
                missing = self._missing_data_locked(self._clock())
                fault = self._fault_reason
            if fault:
                raise RuntimeError(fault)
            if not missing:
                return
            if self._clock() >= deadline:
                raise TimeoutError("Startup data unavailable: " + ", ".join(missing))
            self._shutdown_event.wait(0.05)
        raise RuntimeError("Startup cancelled")

    def get_initial_heading(self, timeout=5.0):
        deadline = self._clock() + timeout
        while not self._shutdown_event.is_set():
            with self._state_lock:
                if self.heading_timestamp is not None:
                    return self.current_heading
            if self._clock() >= deadline:
                raise TimeoutError("Initial heading unavailable")
            self._shutdown_event.wait(0.05)
        raise RuntimeError("Heading read cancelled")

    def _arm_vehicle(self):
        with self._send_lock:
            with self._state_lock:
                if self._closing:
                    raise RuntimeError("Controller is closing")
                if self._armed:
                    return
                self._arm_requested = True
            if self._mavlink_io is None:
                raise RuntimeError("Flight controller not connected")
            for _ in range(3):
                self._mavlink_io.send_arm_command(1)
        deadline = self._clock() + self.ARM_TIMEOUT
        while not self._shutdown_event.is_set():
            with self._state_lock:
                if self._armed:
                    print("Armed!")
                    return
                if self._fault_reason:
                    raise RuntimeError(self._fault_reason)
            if self._clock() >= deadline:
                raise TimeoutError("Arming not confirmed by heartbeat")
            self._shutdown_event.wait(0.05)
        raise RuntimeError("Arming cancelled")

    def _neutral_locked(self):
        for channel in (THROTTLE_CHANNEL, YAW_CHANNEL, FORWARD_CHANNEL):
            self.channel_values[channel] = PWM_NEUTRAL

    def _send_neutral(self):
        with self._send_lock:
            with self._state_lock:
                self._neutral_locked()
                channels = tuple(self.channel_values)
            if self._mavlink_io is not None:
                self._mavlink_io.send_channels(channels)

    def _cancel_mission_locked(self):
        self._cancel_event.set()
        self._mission_epoch += 1
        self._mission_running = False
        self.move_start_time = None
        self.adjust_start_time = None
        self._stuck_samples.clear()
        self.is_stuck = False
        self.stuck_timer_start = 0

    def _latch_stop_locked(self):
        self._stopped = True
        self._cancel_mission_locked()
        self.depth_control_enabled = False
        self.manual_throttle = PWM_NEUTRAL
        self.manual_yaw = PWM_NEUTRAL
        self.manual_forward = PWM_NEUTRAL
        self._neutral_locked()

    def stop_movement(self):
        """持续禁止推进；新的显式操作才可以解除停止。"""
        with self._send_lock:
            with self._state_lock:
                self._latch_stop_locked()
                channels = tuple(self.channel_values)
            if self._mavlink_io is not None:
                self._mavlink_io.send_channels(channels)
        print("All movement stopped")

    def _enter_fault(self, reason, recoverable=True):
        with self._send_lock:
            with self._state_lock:
                changed = self._fault_reason != reason
                self._fault_reason = reason
                self._fault_recoverable = self._fault_recoverable and recoverable
                self._latch_stop_locked()
                channels = tuple(self.channel_values)
            try:
                if self._mavlink_io is not None:
                    self._mavlink_io.send_channels(channels)
            except Exception as error:
                if isinstance(error, (OSError, EOFError)):
                    self._fc_reconnect_event.set()
                print(f"Neutral command could not be delivered: {error}")
        if changed:
            print(f"Control stopped: {reason}")

    def close(self):
        """先停车和请求上锁，再结束线程；发送失败也继续清理资源。"""
        with self._state_lock:
            if self._closing:
                return
            self._closing = True
        try:
            try:
                self.stop_movement()
            except Exception as error:
                print(f"Stop send failed during shutdown: {error}")
            with self._send_lock:
                with self._state_lock:
                    should_disarm = self._arm_requested or self._armed
                if should_disarm and self._mavlink_io is not None:
                    try:
                        self._mavlink_io.send_arm_command(0)
                    except Exception as error:
                        print(f"Disarm send failed during shutdown: {error}")
        finally:
            with self._state_lock:
                self.running = False
            self._shutdown_event.set()
            self._cancel_event.set()
            if self._dvl_client is not None:
                try:
                    self._dvl_client.close()
                except Exception as error:
                    print(f"DVL close failed: {error}")
            for worker in self._workers.values():
                if worker is not threading.current_thread() and worker.ident is not None:
                    worker.join(timeout=2.0)
                    if worker.is_alive():
                        print(f"Worker still finishing external call: {worker.name}")
            if self._mavlink_io is not None:
                try:
                    self._mavlink_io.close()
                except Exception as error:
                    print(f"Flight controller close failed: {error}")

    def _on_position(self, x, y, timestamp):
        with self._state_lock:
            self.current_position[:] = [x, y]
            self.dvl_timestamp = timestamp

    def _on_dvl_disconnect(self, reason):
        with self._state_lock:
            self.dvl_timestamp = None
            must_stop = self._mission_running or (self._arm_requested and not self._initialized)
        if must_stop:
            self._enter_fault(f"DVL disconnected: {reason}")

    def read_dvl_data(self):
        self._dvl_client.run(self._shutdown_event)

    @staticmethod
    def _valid_number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    def read_fc_data(self):
        while not self._shutdown_event.is_set():
            if self._fc_reconnect_event.is_set():
                try:
                    self._mavlink_io.reconnect()
                    self._fc_reconnect_event.clear()
                except (OSError, EOFError):
                    self._shutdown_event.wait(1.0)
                    continue
            try:
                messages = self._mavlink_io.poll()
            except (OSError, EOFError) as error:
                with self._state_lock:
                    self.heading_timestamp = self.depth_timestamp = self.heartbeat_timestamp = None
                self._enter_fault(f"FC connection lost: {error}")
                self._fc_reconnect_event.set()
                if self._shutdown_event.wait(1.0):
                    break
                continue
            for message in messages:
                kind = message.get_type()
                now = self._clock()
                with self._state_lock:
                    if kind == "VFR_HUD" and self._valid_number(message.heading):
                        self.current_heading = message.heading
                        self.heading_timestamp = now
                    elif kind == "AHRS2" and self._valid_number(message.altitude):
                        self.current_depth = message.altitude
                        self.depth_timestamp = now
                    elif kind == "HEARTBEAT":
                        self.heartbeat_timestamp = now
                        self._armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self._shutdown_event.wait(self.CONTROL_PERIOD)

    def _worker_failure_locked(self):
        if not self._workers_started:
            return None
        for name, worker in self._workers.items():
            if not worker.is_alive():
                return f"{name} is not running"
        return None

    def _check_ready_to_resume(self, need_dvl):
        with self._state_lock:
            if self._closing:
                raise RuntimeError("Controller is closing")
            if self._fault_reason and not self._fault_recoverable:
                raise RuntimeError("Restart the program after worker failure: " + self._fault_reason)
            failure = self._worker_failure_locked()
            if failure:
                raise RuntimeError(failure)
            missing = self._missing_data_locked(self._clock(), need_dvl=need_dvl)
            if missing:
                raise RuntimeError("Fresh data required: " + ", ".join(missing))
            self._fault_reason = None
            self._fault_recoverable = True
        self._arm_vehicle()

    def _check_task_locked(self, epoch=None):
        if epoch is None:
            epoch = getattr(self._task_context, "epoch", None)
        if epoch is not None and (epoch != self._mission_epoch or not self._mission_running
                                  or self._stopped or self._closing):
            raise TaskCancelled()

    def _task_wait(self, seconds):
        with self._state_lock:
            self._check_task_locked()
            cancelled = getattr(self._task_context, "cancel_event", None) or self._cancel_event
        if cancelled.wait(seconds):
            raise TaskCancelled()
        with self._state_lock:
            self._check_task_locked()

    def set_channel(self, channel_index, value):
        value = max(PWM_MIN, min(int(value), PWM_MAX))
        with self._state_lock:
            self._check_task_locked()
            if self._stopped and value != PWM_NEUTRAL:
                return
            self.channel_values[channel_index] = value

    def set_manual_channel(self, channel_name, value):
        if channel_name in MANUAL_CHANNELS:
            self._set_manual_channels({channel_name: value})

    def _set_manual_channels(self, values):
        prepared = {name: max(PWM_MIN, min(int(value), PWM_MAX))
                    for name, value in values.items()}
        with self._state_lock:
            epoch = self._mission_epoch
        self._check_ready_to_resume(need_dvl=False)
        with self._state_lock:
            if self.control_mode != "manual" or self._closing or epoch != self._mission_epoch:
                return
            missing = self._missing_data_locked(self._clock(), need_dvl=False)
            if missing:
                raise RuntimeError("Fresh data required: " + ", ".join(missing))
            self._stopped = False
            for name, value in values.items():
                channel_index, attribute = MANUAL_CHANNELS[name]
                setattr(self, attribute, value)
                self.channel_values[channel_index] = prepared[name]
        for name, value in values.items():
            print(f"Set {name} to {value}")

    def set_target_attitude(self, roll=None, pitch=None, yaw=None):
        with self._state_lock:
            self._check_task_locked()
            for axis, value in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
                if value is not None:
                    self.target_attitude[axis] = value

    def set_target_depth(self, depth):
        with self._state_lock:
            self._check_task_locked()
            self.target_depth = depth

    def enable_depth_control(self):
        with self._state_lock:
            self._check_task_locked()
            if self._stopped:
                return
            self.altitude_pid.reset()
            self.depth_control_enabled = True
        print("Depth control enabled")

    def disable_depth_control(self):
        with self._state_lock:
            self._check_task_locked()
            self.depth_control_enabled = False
            self.altitude_pid = PIDController(300.0, 5.0, 10, -150, 150)
            self.channel_values[THROTTLE_CHANNEL] = PWM_NEUTRAL
        print("Depth control disabled")

    def altitude_control_loop(self):
        while not self._shutdown_event.is_set():
            with self._state_lock:
                enabled = self.depth_control_enabled and not self._stopped
                stale = enabled and (self.depth_timestamp is None or
                        self._clock() - self.depth_timestamp > self.SENSOR_TIMEOUT)
                if enabled and not stale:
                    output = self.altitude_pid.compute(self.target_depth, -self.current_depth)
                    self.channel_values[THROTTLE_CHANNEL] = max(PWM_MIN, min(int(PWM_NEUTRAL + output), PWM_MAX))
            if stale:
                self._enter_fault("Depth data expired")
            self._shutdown_event.wait(self.CONTROL_PERIOD)

    def _send_once(self):
        with self._send_lock:
            with self._state_lock:
                failure = self._worker_failure_locked()
                missing = [] if self._stopped else self._missing_data_locked(
                    self._clock(), need_dvl=self.control_mode == "auto")
            if failure:
                self._enter_fault(failure, recoverable=False)
            elif missing:
                self._enter_fault("Data expired: " + ", ".join(missing))
            with self._state_lock:
                allowed = not self._stopped and self._armed and not self._closing
                if not allowed:
                    self._neutral_locked()
                channels = tuple(self.channel_values)
                attitude = dict(self.target_attitude) if allowed and self.control_mode == "auto" else None
                elapsed = self._clock() - self.boot_time
            if self._mavlink_io is not None:
                if attitude is not None:
                    self._mavlink_io.send_attitude(attitude, elapsed)
                self._mavlink_io.send_channels(channels)

    def send_loop(self):
        while not self._shutdown_event.is_set():
            try:
                self._send_once()
            except (OSError, EOFError) as error:
                self._fc_reconnect_event.set()
                self._enter_fault(f"FC send failed: {error}")
                self._shutdown_event.wait(1.0)
            self._shutdown_event.wait(self.CONTROL_PERIOD)

    def set_mode(self, mode):
        with self._send_lock:
            result = self._mavlink_io.set_named_mode(mode)
        print(f"Mode set to {mode}" if result else f"Unknown mode: {mode}")
        return result

    def set_stabilize(self):
        with self._send_lock:
            return self._mavlink_io.set_custom_mode(0)

    def set_depth_hold(self):
        with self._send_lock:
            return self._mavlink_io.set_custom_mode(2)

    @staticmethod
    def _copy_waypoints(waypoints):
        copied = copy.deepcopy(list(waypoints))
        for waypoint in copied:
            if isinstance(waypoint, tuple):
                if len(waypoint) != 3:
                    raise ValueError("Waypoint tuple must contain heading, distance, depth")
                values = waypoint
            elif isinstance(waypoint, dict):
                values = [waypoint[key] for key in ("h", "d", "z")]
                if not isinstance(waypoint.get("tasks", []), list):
                    raise ValueError("Waypoint tasks must be a list")
            else:
                raise ValueError("Waypoint must be a tuple or dictionary")
            if not all(AdvancedAUVControl._valid_number(value) for value in values):
                raise ValueError("Waypoint values must be finite numbers")
        return copied

    def _reset_mission_state_locked(self):
        self.current_waypoint_index = 0
        self.current_task_index = 0
        self.task_done = False
        self.ok = 0
        self.segment_state = "adjust_heading"
        self.segment_progress = 0.0
        self.segment_target_heading = 0.0
        self.segment_target_distance = 0.0
        self.segment_origin_x = 0.0
        self.move_start_time = None
        self.adjust_start_time = None
        self.last_remaining = 0
        self.boost_fwd = self.boost_rev = PWM_NEUTRAL
        self.boosted_thrust = 1560
        self.is_stuck = False
        self.stuck_timer_start = 0
        self._stuck_samples.clear()

    def set_waypoints(self, waypoints):
        copied = self._copy_waypoints(waypoints)
        with self._send_lock:
            self.stop_movement()
            with self._state_lock:
                self.waypoints = copied
                self.waypoints_use = copy.deepcopy(copied)
                self._reset_mission_state_locked()
        print(f"Set {len(copied)} waypoints")

    def start_mission(self):
        with self._send_lock:
            self.stop_movement()
            with self._state_lock:
                configured = self._copy_waypoints(self.waypoints)
                epoch = self._mission_epoch
        if not configured:
            print("请先设置路径点!")
            return False
        self._check_ready_to_resume(need_dvl=True)
        with self._state_lock:
            if self._closing or epoch != self._mission_epoch:
                return False
            missing = self._missing_data_locked(self._clock())
            if missing:
                raise RuntimeError("Fresh data required: " + ", ".join(missing))
            self.waypoints_use = configured
            self._reset_mission_state_locked()
            self._mission_epoch += 1
            self._cancel_event = threading.Event()
            self._mission_running = True
            self._stopped = False
            self.control_mode = "auto"
            self.altitude_pid.reset()
            self.depth_control_enabled = True
        print("开始巡线任务...")
        return True

    def control_loop(self):
        while not self._shutdown_event.is_set():
            try:
                with self._state_lock:
                    automatic = self.control_mode == "auto" and self._mission_running and not self._stopped
                    if not automatic and self.control_mode == "manual" and not self._stopped:
                        self.channel_values[YAW_CHANNEL] = max(PWM_MIN, min(int(self.manual_yaw), PWM_MAX))
                        self.channel_values[FORWARD_CHANNEL] = max(PWM_MIN, min(int(self.manual_forward), PWM_MAX))
                if automatic:
                    self.auto_control()
            except TaskCancelled:
                pass
            self._shutdown_event.wait(self.CONTROL_PERIOD)

    def auto_control(self):
        with self._state_lock:
            if self._stopped or not self._mission_running or not self.waypoints_use:
                return
            missing = self._missing_data_locked(self._clock())
            epoch = self._mission_epoch
            cancelled = self._cancel_event
            waypoint = self.waypoints_use[self.current_waypoint_index]
            if isinstance(waypoint, tuple):
                waypoint = dict(zip(("h", "d", "z"), waypoint))
            waypoint = copy.deepcopy(waypoint)
            state = self.segment_state
        if missing:
            self._enter_fault("Data expired: " + ", ".join(missing))
            return
        self._task_context.epoch = epoch
        self._task_context.cancel_event = cancelled
        try:
            self.set_target_depth(waypoint["z"])
            if state == "adjust_depth":
                self._adjust_depth(waypoint["z"])
            elif state == "adjust_heading":
                self._adjust_heading(waypoint["h"])
            elif state == "reset_dvl":
                self._reset_segment(waypoint["h"], waypoint["d"])
            elif state == "move_forward":
                self._move_forward(waypoint.get("tasks", []))
            elif state == "do_tasks":
                self._run_waypoint_tasks(waypoint.get("tasks", []))
        finally:
            self._task_context.epoch = None
            self._task_context.cancel_event = None

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _adjust_depth(self, target_depth):
        with self._state_lock:
            self._check_task_locked()
            current_depth = self.current_depth
            reached = abs(target_depth - (-current_depth)) < self.depth_threshold
        if reached:
            print(f"Depth adjusted: {current_depth:.2f}m -> {target_depth:.2f}m")
            self._task_wait(1)
            with self._state_lock:
                self._check_task_locked()
                self.segment_state = "adjust_heading"

    def _adjust_heading(self, target_heading):
        with self._state_lock:
            self._check_task_locked()
            now = self._clock()
            if self.adjust_start_time is None:
                self.adjust_start_time = now
            self.target_attitude["yaw"] = target_heading
            error = abs(self.normalize_angle(target_heading - self.current_heading))
            reached = error < self.heading_threshold
            timed_out = now - self.adjust_start_time > 8.0
        if reached or timed_out:
            self._task_wait(1)
            with self._state_lock:
                self._check_task_locked()
                self.adjust_start_time = None
                self.segment_state = "reset_dvl"

    def reset_dvl_dead_reckoning(self):
        with self._state_lock:
            self._check_task_locked()
            cancelled = getattr(self._task_context, "cancel_event", None) or self._cancel_event
        with self._reset_lock:
            successful = self._dvl_client.request_reset(cancel_event=cancelled)
            origin = self._dvl_client.last_reset_position if successful else None
            with self._state_lock:
                self._check_task_locked()
                if not successful or origin is None:
                    return False
                x, y, timestamp = origin
                if not 0 <= self._clock() - timestamp <= self.SENSOR_TIMEOUT:
                    return False
                self._last_reset_position = (x, y, timestamp)
                self.last_reset_time = timestamp
                return True

    def _reset_segment(self, target_heading, target_distance):
        if not self.reset_dvl_dead_reckoning():
            with self._state_lock:
                self._check_task_locked()
            self._enter_fault("DVL reset was not confirmed with a new valid position")
            return
        with self._state_lock:
            self._check_task_locked()
            self.segment_origin_x = self._last_reset_position[0]
            self.segment_target_heading = target_heading
            self.segment_target_distance = target_distance
            self.segment_progress = 0.0
            self.move_start_time = None
            self._stuck_samples.clear()
            self.segment_state = "move_forward"
        print(f"Starting forward movement: {target_distance:.2f}m at {target_heading:.1f}°")
        self._task_wait(1)

    def _move_forward(self, tasks):
        with self._state_lock:
            self._check_task_locked()
            now = self._clock()
            if self.move_start_time is None:
                self.move_start_time = now
                self.boost_fwd = self.boost_rev = PWM_NEUTRAL
                self._stuck_samples.clear()
            self.segment_progress = self.current_position[0] - self.segment_origin_x
            remaining = self.segment_target_distance - self.segment_progress
            forward_value = self._calculate_forward_pwm(remaining, now)
            self.last_remaining = remaining
            self.channel_values[FORWARD_CHANNEL] = max(PWM_MIN, min(int(forward_value), PWM_MAX))
            progress, distance = self.segment_progress, self.segment_target_distance
            timed_out = now - self.move_start_time > 20.0
            reached = abs(remaining) <= self.waypoint_threshold
            index = self.current_waypoint_index
            if reached or timed_out:
                self.move_start_time = None
                self.channel_values[FORWARD_CHANNEL] = PWM_NEUTRAL
                if tasks:
                    self.segment_state = "do_tasks"
                    self.current_task_index = 0
                    self.task_done = False
        print(f"Moving forward: {progress:.2f}/{distance:.2f}m "
              f"(remaining: {remaining:.2f}m)(pwm: {forward_value}) ")
        if reached or timed_out:
            print("Forward timeout, skip to next waypoint" if timed_out else f"Reached waypoint {index + 1}")
            if not tasks:
                self._next_waypoint()

    def _calculate_forward_pwm(self, remaining, now):
        """按 3 秒时间窗内累计位移判断卡滞。"""
        self._stuck_samples.append((now, remaining))
        cutoff = now - self.STUCK_WINDOW
        while len(self._stuck_samples) > 1 and self._stuck_samples[1][0] <= cutoff:
            self._stuck_samples.popleft()
        first_time, first_remaining = self._stuck_samples[0]
        stationary = (now - first_time >= self.STUCK_WINDOW and
                      abs(remaining - first_remaining) < self.STUCK_DISTANCE)
        base = PWM_NEUTRAL + self.Kp_forward * remaining
        if stationary:
            if not self.is_stuck:
                self.is_stuck = True
                self.stuck_timer_start = now - self.STUCK_WINDOW
            if now - self.stuck_timer_start >= self.STUCK_WINDOW:
                if remaining > 0:
                    self.boost_fwd = min(self.boost_fwd + 300, 1800)
                    result = max(1420, int(self.boost_fwd))
                else:
                    self.boost_rev = max(self.boost_rev - 200, 1000)
                    result = min(1580, int(self.boost_rev))
                self.stuck_timer_start = now
                return result
            return max(1420, min(base, 1560))
        self.is_stuck = False
        self.boosted_thrust = 1560
        return max(1430, min(base, 1560))

    def _run_waypoint_tasks(self, tasks):
        with self._state_lock:
            self._check_task_locked()
            task = tasks[self.current_task_index]
            already_done = self.task_done
        if not already_done:
            print(f"Start task: {task}")
            if task == "roll":
                self.do_roll_task()
            elif task == "turn":
                self.turn_task()
            elif task == "photo":
                self._task_wait(0.5)
                snap()
            elif task == "mod":
                self.up_task()
            elif task in ("a", "b", "c"):
                getattr(self, task)()
                self.ppp_task()
            else:
                print(f"Unknown task: {task}")
        with self._state_lock:
            self._check_task_locked()
            self.task_done = False
            self.current_task_index += 1
            completed = self.current_task_index >= len(tasks)
        if completed:
            self._next_waypoint()

    def _next_waypoint(self):
        with self._send_lock:
            with self._state_lock:
                self._check_task_locked()
                self.current_waypoint_index += 1
                self.segment_state = "adjust_depth"
                self.move_start_time = self.adjust_start_time = None
                self._stuck_samples.clear()
                self.is_stuck = False
                finished = self.current_waypoint_index >= len(self.waypoints_use)
                if finished:
                    self._latch_stop_locked()
                    self.ok = 1
                    self.waypoints_use = []
                    self.current_waypoint_index = 0
                    channels = tuple(self.channel_values)
            if finished:
                if self._mavlink_io is not None:
                    self._mavlink_io.send_channels(channels)
                    self._mavlink_io.send_arm_command(0)
                with self._state_lock:
                    self._armed = False
                print("Mission completed!")

    def do_roll_task(self):
        self.disable_depth_control()
        self._task_wait(1)
        for roll_angle in range(0, 730, 10):
            self.set_target_attitude(roll=roll_angle)
            self._task_wait(0.1)
        self._task_wait(1)
        self.enable_depth_control()

    def turn_task(self):
        for yaw_angle in range(0, 100, 10):
            self.set_target_attitude(yaw=yaw_angle)
            self._task_wait(0.5)

    def ppp_task(self):
        self.set_target_attitude(pitch=-20)
        self._task_wait(2)
        self.set_target_attitude(pitch=0)
        self._task_wait(1)

    def a(self):
        a_jpg()
        self._task_wait(0.5)

    def b(self):
        b_jpg()
        self._task_wait(0.5)

    def c(self):
        c_jpg()
        self._task_wait(0.5)

    def up_task(self):
        self.set_target_depth(0.5)
        self._task_wait(15)

    def _switch_control_mode(self):
        self.stop_movement()
        with self._state_lock:
            self.control_mode = "auto" if self.control_mode == "manual" else "manual"
            automatic = self.control_mode == "auto"
        if automatic:
            self.start_mission()
        else:
            self.disable_depth_control()
        print(f"切换至{'自动' if automatic else '手动'}模式")

    def interactive_control(self):
        while not self._shutdown_event.is_set():
            self.print_menu()
            try:
                cmd = input("输入指令: ").strip().lower()
                if cmd == "q":
                    self.close()
                    print("退出系统")
                    break
                if cmd == "g":
                    self.stop_movement()
                elif cmd == "m":
                    self._switch_control_mode()
                else:
                    with self._state_lock:
                        manual = self.control_mode == "manual"
                    if manual:
                        self.handle_manual_command(cmd)
                    else:
                        self.handle_auto_command(cmd)
            except EOFError:
                self.close()
                break
            except Exception as error:
                self._enter_fault(f"Command failed: {error}")

    def handle_manual_command(self, cmd):
        movement = MOVEMENT_COMMANDS.get(cmd)
        attitude = ATTITUDE_COMMANDS.get(cmd)
        if movement is not None:
            channel, value, message = movement
            self.set_manual_channel(channel, value)
            print(message)
        elif attitude is not None:
            axis, step = attitude
            with self._state_lock:
                self.target_attitude[axis] += step
        elif cmd == "r":
            self.set_target_attitude(0, 0, 0)
        elif cmd == "=":
            print("已保存：", snap())
        elif cmd in ("z", "x"):
            # 保留原有按键语义，本轮不改菜单与 mo 的历史行为。
            with self._state_lock:
                self.mo = 0 if cmd == "z" else 2
        elif cmd == "b":
            try:
                depth = float(input("输入目标深度（米）: "))
                self.set_target_depth(depth)
                print(f"目标深度设置为{depth}米")
            except ValueError:
                print("无效输入! 请输入数字")
        elif cmd == "p":
            try:
                throttle = int(input("油门(1100-1900, 1500=停止): "))
                yaw = int(input("偏航(1100-1900, 1500=停止): "))
                forward = int(input("前进(1100-1900, 1500=停止): "))
                self._set_manual_channels({"throttle": throttle, "yaw": yaw, "forward": forward})
                print(f"已设置: T={throttle}, Y={yaw}, F={forward}")
            except ValueError:
                print("无效输入! 请输入整数")

    def handle_auto_command(self, cmd):
        if cmd == "s":
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
            except (ValueError, KeyError):
                print("无效输入!")
        elif cmd == "c":
            self.set_waypoints([])
            print("路径点已清除")
        elif cmd == "d":
            try:
                depth = float(input("输入目标深度(米): "))
                self.set_target_depth(depth)
                print(f"目标深度设置为 {depth} 米")
            except ValueError:
                print("无效输入!")
        elif cmd == "e":
            self.start_mission()

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


def main():
    auv = None
    try:
        for i in range(5, 0, -1):
            print(f'\r倒计时 {i} 秒', end='', flush=True)
            time.sleep(1)
        print('\r开始了！     ')
        auv = AdvancedAUVControl()
        auv.interactive_control()
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if auv is not None:
            auv.close()
        print("系统关闭")


if __name__ == "__main__":
    main()
