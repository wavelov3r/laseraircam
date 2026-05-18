import re
import select
import socket
import threading
import time

from settings import SERIAL_PROXY_LISTEN_HOST
from settings import SERIAL_PROXY_LISTEN_PORT
from settings import SERIAL_PROXY_TARGET_HOST
from settings import SERIAL_PROXY_TARGET_PORT


class LaserMonitorState:
	def __init__(self, logger=None):
		self._lock = threading.Lock()
		self._line_buffers = {"up": "", "down": ""}
		self._state = "idle"
		self._active = False
		self._last_error = ""
		self._sticky_error = ""
		self._last_error_ts = 0.0
		self._last_error_accumulate = False
		self._engrave_complete_active = False
		self._jogging = False
		self._laser_output_on = False
		self._last_event = "idle"
		self._last_command = ""
		self._last_activity_ts = 0.0
		self._job_started_ts = 0.0
		self._bytes_up = 0
		self._bytes_down = 0
		self._last_line = ""
		self._last_tx_command = ""
		self._last_rx_line = ""
		self._last_line_ts = 0.0
		self._last_s_value = 0
		self._pending_airassist_event = ""
		self._pending_bridge_commands = []
		self._repeat_ignore_window_s = 0.20
		self._engrave_complete_min_run_s = 30.0
		self._running_inactive_timeout_s = 35.0
		self._logger = logger or (lambda m: print(f"[laser-monitor] {m}", flush=True))

	def _log(self, msg):
		try:
			self._logger(msg)
		except Exception:
			pass

	def _running_duration_s(self, now=None):
		now_ts = now if now is not None else time.time()
		if self._job_started_ts <= 0:
			return 0.0
		return max(0.0, float(now_ts - self._job_started_ts))

	def _should_mark_engrave_complete(self, now=None):
		if self._jogging:
			return False
		return self._running_duration_s(now) >= self._engrave_complete_min_run_s

	def feed(self, direction, payload_bytes):
		if not payload_bytes:
			return
		now = time.time()
		text = payload_bytes.decode("utf-8", errors="ignore")
		with self._lock:
			if direction == "up":
				self._bytes_up += len(payload_bytes)
			else:
				self._bytes_down += len(payload_bytes)
			self._last_activity_ts = now
			# GRBL endpoints can terminate lines with "\n", "\r\n", or only "\r".
			# Normalizing keeps parsing reliable with no extra polling overhead.
			self._line_buffers[direction] += text.replace("\r", "\n")
			buffer = self._line_buffers[direction]
			parts = buffer.split("\n")
			self._line_buffers[direction] = parts[-1][-300:]
			for raw_line in parts[:-1]:
				self._apply_line(raw_line.strip(), direction)

	def _apply_line(self, line, direction):
		if not line:
			return
		now = time.time()
		if line == self._last_line and (now - self._last_line_ts) < self._repeat_ignore_window_s:
			return
		self._last_line = line
		self._last_line_ts = now
		upper = line.upper()
		self._last_command = line[:180]
		if direction == "up":
			self._last_tx_command = re.sub(r'\?{3,}', '?', line)[:180]
			_m7m8m9 = re.findall(r'(?<![A-Z0-9])M(7|8|9)(?![0-9])', upper)
			if _m7m8m9:
				last_code = _m7m8m9[-1]
				self._pending_airassist_event = "on" if last_code in ("7", "8") else "off"
				tx_preview = re.sub(r'\?+', '', line).strip()
				self._log(f"airassist gcode event M{last_code} detected (event={self._pending_airassist_event}) tx='{tx_preview[:120]}'")
			# Detect (BRIDGE:key=value key2=value2) gcode comments from LightBurn
			_bridge_cmds = re.findall(r'\(BRIDGE:([^)]*)\)', line, re.IGNORECASE)
			if _bridge_cmds:
				for cmd_str in _bridge_cmds:
					pairs = cmd_str.split()
					for pair in pairs:
						if '=' in pair:
							key, val = pair.split('=', 1)
							self._pending_bridge_commands.append({
								'key': key.strip().lower(),
								'value': val.strip()
							})
					self._log(f"BRIDGE gcode commands detected: {cmd_str}")
			# Track S (laser power) parameter from any upstream GCODE command.
			_sm = re.search(r'(?<![A-RT-Z])S(\d+(?:\.\d+)?)', upper)
			if _sm:
				try:
					self._last_s_value = float(_sm.group(1))
				except ValueError:
					pass
		else:
			self._last_rx_line = line[:180]

		# Parse GRBL status frames, including query-prefixed lines like '?<Door:1|...>'.
		lt = upper.find("<")
		gt = upper.find(">", lt + 1) if lt >= 0 else -1
		if lt >= 0 and gt > lt:
			frame_body = upper[lt + 1:gt]
			head = frame_body.split("|", 1)[0].strip()
			state_token = head.split(":", 1)[0]
			previous_state = self._state
			status_map = {
				"IDLE": "idle",
				"RUN": "running",
				"JOG": "running",
				"HOLD": "hold",
				"DOOR": "door",
				"ALARM": "error",
				"HOME": "home",
				"CHECK": "check",
				"SLEEP": "sleep",
			}
			mapped = status_map.get(state_token)
			if mapped:
				if mapped == "idle":
					self._jogging = False
					self._laser_output_on = False
					if previous_state == "running" and self._should_mark_engrave_complete(now):
						self._state = "engrave_complete"
						self._active = False
						self._last_event = "engrave_complete"
						self._engrave_complete_active = True
					elif self._engrave_complete_active:
						self._state = "engrave_complete"
						self._active = False
					else:
						self._state = "idle"
						self._active = False
						self._last_event = "status_idle"
						self._last_error_accumulate = False
					return
				self._state = mapped
				self._last_event = "status_" + mapped
				self._active = mapped == "running"
				if mapped != "running":
					self._jogging = False
				if mapped == "running":
					self._jogging = (state_token == "JOG")
					if self._jogging:
						self._laser_output_on = False
					self._engrave_complete_active = False
					if previous_state != "running":
						self._job_started_ts = now
					# Parse FS:feed,spindle field to update live laser output flag during RUN.
					if not self._jogging:
						try:
							_sp = next(f for f in frame_body.split("|") if f.startswith("FS:"))[3:].split(",")[1]
							self._laser_output_on = float(_sp) > 0
						except (StopIteration, IndexError, ValueError):
							pass
				if mapped == "door":
					# Opening the door acknowledges and resets engraved/error status.
					self._laser_output_on = False
					self._engrave_complete_active = False
					self._last_error = ""
					self._sticky_error = ""
					self._last_error_ts = 0.0
					self._last_error_accumulate = False
				if mapped == "error":
					self._engrave_complete_active = False
					if not self._last_error_accumulate:
						ts_str = time.strftime("%H:%M:%S")
						self._last_error = f"[{ts_str}] {head[:200]}"
						self._sticky_error = self._last_error
						self._last_error_ts = now
						self._last_error_accumulate = True
					else:
						self._last_error = self._last_error + " | " + head[:150]
						self._sticky_error = self._last_error
				elif mapped in ("hold", "door", "home", "check", "sleep"):
					self._laser_output_on = False
					self._last_error_accumulate = False
				return

		if "[MSG:PROGRAM END]" in upper:
			if self._state == "running":
				if self._should_mark_engrave_complete(now):
					self._state = "engrave_complete"
					self._last_event = "engrave_complete"
					self._engrave_complete_active = True
				else:
					self._state = "idle"
					self._last_event = "program_end"
					self._engrave_complete_active = False
			else:
				self._state = "idle"
				self._last_event = "program_end"
			self._active = False
			self._last_error_accumulate = False
			return

		if "ERROR" in upper or "ALARM" in upper:
			self._state = "error"
			self._active = False
			self._jogging = False
			self._laser_output_on = False
			self._engrave_complete_active = False
			if not self._last_error_accumulate:
				ts_str = time.strftime("%H:%M:%S")
				self._last_error = f"[{ts_str}] {line[:200]}"
				self._sticky_error = self._last_error
				self._last_error_ts = now
				self._last_error_accumulate = True
			else:
				self._last_error = self._last_error + " | " + line[:150]
				self._sticky_error = self._last_error
			self._last_event = "error"
			return
		if "M5" in upper or "M2" in upper or "M30" in upper:
			self._laser_output_on = False
			self._last_s_value = 0
			if self._state == "running":
				if self._should_mark_engrave_complete(now):
					self._state = "engrave_complete"
					self._last_event = "engrave_complete"
					self._engrave_complete_active = True
				else:
					self._state = "idle"
					self._last_event = "job_stop"
					self._engrave_complete_active = False
			else:
				self._state = "idle"
				self._last_event = "job_stop"
			self._active = False
			self._jogging = False
			self._last_error_accumulate = False
			return
		if "M3" in upper or "M4" in upper:
			self._laser_output_on = True
		if any(token in upper for token in ("M3", "M4", "G1", "G2", "G3")):
			if self._state != "running":
				self._job_started_ts = now
			self._state = "running"
			self._active = True
			self._last_event = "job_start"
			self._engrave_complete_active = False
			self._last_error_accumulate = False

	def get_laser_power_s(self):
		with self._lock:
			return int(round(self._last_s_value))

	def pop_airassist_event(self):
		with self._lock:
			event = str(self._pending_airassist_event or "")
			self._pending_airassist_event = ""
			return event

	def pop_bridge_commands(self):
		with self._lock:
			cmds = list(self._pending_bridge_commands)
			self._pending_bridge_commands.clear()
			return cmds

	def clear_error(self):
		with self._lock:
			self._last_error = ""
			self._sticky_error = ""
			self._last_error_accumulate = False
			self._engrave_complete_active = False
			if self._state in ("error", "engrave_complete"):
				self._state = "idle"
				self._last_event = "error_cleared"
			self._jogging = False
			self._laser_output_on = False

	def snapshot(self):
		now = time.time()
		with self._lock:
			if self._active and (now - self._last_activity_ts) > self._running_inactive_timeout_s and self._state != "error":
				self._active = False
				if self._state == "running":
					self._jogging = False
					self._laser_output_on = False
					if self._should_mark_engrave_complete(now):
						self._state = "engrave_complete"
						self._last_event = "engrave_complete"
						self._engrave_complete_active = True
					else:
						self._state = "idle"
						self._last_event = "idle_timeout"
						self._engrave_complete_active = False
				else:
					self._state = "idle"
					self._last_event = "idle_timeout"
			traffic_active = (now - self._last_activity_ts) <= 2.0
			# laser_active must reflect real laser output (M3/M4 ... M5), not axis motion/jog state.
			laser_active = bool(self._laser_output_on)
			sticky_error = str(self._sticky_error or self._last_error or "")
			state_value = self._state
			if sticky_error and state_value != "door":
				state_value = "error"
			return {
				"state": state_value,
				"laser_active": laser_active,
				"traffic_active": traffic_active,
				"laser_power_s": int(round(self._last_s_value)),
				"last_error": sticky_error,
				"last_error_ts": float(self._last_error_ts),
				"last_event": self._last_event,
				"last_command": self._last_command,
				"last_tx_command": self._last_tx_command,
				"last_rx_line": self._last_rx_line,
				"bytes_up": self._bytes_up,
				"bytes_down": self._bytes_down,
				"last_activity_ts": self._last_activity_ts,
				"job_started_ts": self._job_started_ts,
			}


class SerialTrafficProxy:
	def __init__(self, monitor, logger):
		self._monitor = monitor
		self._logger = logger
		self._lock = threading.Lock()
		self._running = False
		self._thread = None
		self._listen_socket = None
		self._maintenance_socket = None
		self._maintenance_poll_interval_s = 1.0
		self._passthrough_idle_timeout_s = 5.0
		self._passthrough_extend_on_realtime = False
		self._last_poll_ts = 0.0
		self._client_threads = []
		self._active_clients = 0
		self._passthrough_clients = 0
		self._config = None
		self._command_queue = []
		self._next_command_id = 1
		self._last_command_result = {}
		self._last_target_error = ""
		self._last_target_connect_ts = 0.0
		self._last_target_tx_ts = 0.0
		self._last_target_rx_ts = 0.0

	def _log(self, message):
		self._logger(message)

	def apply_config(self, config):
		normalized = (
			int(config.get("serial_proxy_enabled", 0)),
			str(config.get("serial_proxy_listen_host", SERIAL_PROXY_LISTEN_HOST)),
			int(config.get("serial_proxy_listen_port", SERIAL_PROXY_LISTEN_PORT)),
			str(config.get("serial_proxy_target_host", SERIAL_PROXY_TARGET_HOST)),
			int(config.get("serial_proxy_target_port", SERIAL_PROXY_TARGET_PORT)),
		)
		with self._lock:
			self._passthrough_extend_on_realtime = bool(int(config.get("passthrough_extend_on_realtime", 0)))
			if self._config == normalized:
				return
			self._config = normalized
		self.stop()
		if normalized[0] == 1:
			self.start()

	def start(self):
		with self._lock:
			if self._running:
				return
			self._running = True
		self._thread = threading.Thread(target=self._accept_loop, daemon=True)
		self._thread.start()

	def stop(self):
		with self._lock:
			was_running = self._running
			self._running = False
			listen_socket = self._listen_socket
			self._listen_socket = None
			maintenance_socket = self._maintenance_socket
			self._maintenance_socket = None
		if not was_running:
			return
		if listen_socket is not None:
			try:
				listen_socket.close()
			except Exception:
				pass
		if maintenance_socket is not None:
			try:
				maintenance_socket.close()
			except Exception:
				pass
		if self._thread is not None:
			self._thread.join(timeout=1.5)
			self._thread = None

	def _close_maintenance_socket(self):
		maintenance_socket = None
		with self._lock:
			maintenance_socket = self._maintenance_socket
			self._maintenance_socket = None
		if maintenance_socket is not None:
			try:
				maintenance_socket.close()
			except Exception:
				pass

	def _encode_command(self, command):
		text = str(command or "").strip()
		if text in ("?", "!", "~"):
			return text.encode("ascii", errors="ignore")
		if text.lower() in ("ctrl+x", "\x18"):
			return b"\x18"
		line = text.replace("\r", " ").replace("\n", " ").strip()
		if not line:
			return b""
		return (line + "\n").encode("utf-8", errors="ignore")

	def enqueue_command(self, command, source="api"):
		text = str(command or "").strip()
		if not text:
			raise ValueError("command is required")
		now = time.time()
		with self._lock:
			enabled = int(self._config[0]) if self._config else 0
			if not self._running or enabled != 1:
				raise RuntimeError("serial proxy is disabled")
			cmd_id = self._next_command_id
			self._next_command_id += 1
			self._command_queue.append({
				"id": cmd_id,
				"command": text,
				"source": str(source or "api")[:40],
				"queued_ts": now,
			})
			if len(self._command_queue) > 200:
				self._command_queue = self._command_queue[-200:]
			passthrough_clients = int(self._passthrough_clients)
			maintenance_connected = self._maintenance_socket is not None
			queue_depth = len(self._command_queue)
		return {
			"queued": True,
			"command_id": cmd_id,
			"queue_depth": queue_depth,
			"accepted_ts": now,
			"deferred": passthrough_clients > 0 or not maintenance_connected,
		}

	def clear_command_queue(self, source="api"):
		now = time.time()
		with self._lock:
			cleared_count = len(self._command_queue)
			self._command_queue = []
			queue_depth = 0
		return {
			"cleared": cleared_count,
			"queue_depth": queue_depth,
			"cleared_ts": now,
			"source": str(source or "api")[:40],
		}

	def get_link_state(self):
		with self._lock:
			enabled = int(self._config[0]) if self._config else 0
			active_clients = int(self._active_clients)
			passthrough_clients = int(self._passthrough_clients)
			extend_on_rt = bool(self._passthrough_extend_on_realtime)
			maintenance_connected = self._maintenance_socket is not None
			queue_depth = len(self._command_queue)
			last_command_result = dict(self._last_command_result)
			last_target_error = str(self._last_target_error or "")
			last_target_connect_ts = float(self._last_target_connect_ts)
			last_target_tx_ts = float(self._last_target_tx_ts)
			last_target_rx_ts = float(self._last_target_rx_ts)
		mode = "disconnected"
		if passthrough_clients > 0:
			mode = "client_passthrough" if extend_on_rt else "hybrid_passthrough"
		elif maintenance_connected:
			mode = "maintenance"
		return {
			"enabled": enabled == 1,
			"running": bool(self._running),
			"mode": mode,
			"active_clients": active_clients,
			"passthrough_clients": passthrough_clients,
			"maintenance_connected": maintenance_connected,
			"queue_depth": queue_depth,
			"last_command_result": last_command_result,
			"last_target_error": last_target_error,
			"last_target_connect_ts": last_target_connect_ts,
			"last_target_tx_ts": last_target_tx_ts,
			"last_target_rx_ts": last_target_rx_ts,
		}

	def _maintenance_tick(self, target_host, target_port):
		with self._lock:
			if self._passthrough_clients > 0:
				need_close = self._maintenance_socket is not None
			else:
				need_close = False
		if need_close:
			self._close_maintenance_socket()
			return

		with self._lock:
			maintenance_socket = self._maintenance_socket
		if maintenance_socket is None:
			try:
				sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				sock.settimeout(6)
				sock.connect((target_host, target_port))
				sock.setblocking(False)
				with self._lock:
					if self._passthrough_clients > 0 or not self._running:
						try:
							sock.close()
						except Exception:
							pass
						return
					self._maintenance_socket = sock
					self._last_target_connect_ts = time.time()
					self._last_target_error = ""
				maintenance_socket = sock
			except Exception as exc:
				with self._lock:
					self._last_target_error = f"maintenance connect failed: {exc}"
				return

		now = time.time()
		if (now - self._last_poll_ts) >= self._maintenance_poll_interval_s:
			try:
				maintenance_socket.sendall(b"?")
				with self._lock:
					self._last_target_tx_ts = time.time()
				self._last_poll_ts = now
			except Exception as exc:
				with self._lock:
					self._last_target_error = f"maintenance poll failed: {exc}"
				self._close_maintenance_socket()
				return

		while True:
			with self._lock:
				if self._passthrough_clients > 0 or not self._command_queue:
					break
				item = dict(self._command_queue[0])
			payload = self._encode_command(item.get("command", ""))
			if not payload:
				with self._lock:
					if self._command_queue:
						self._command_queue.pop(0)
				continue
			try:
				maintenance_socket.sendall(payload)
				self._monitor.feed("up", payload)
				now_ts = time.time()
				with self._lock:
					if self._command_queue:
						self._command_queue.pop(0)
					self._last_target_tx_ts = now_ts
					self._last_command_result = {
						"id": item.get("id"),
						"command": str(item.get("command", ""))[:180],
						"source": str(item.get("source", ""))[:40],
						"sent_ts": now_ts,
					}
			except Exception as exc:
				with self._lock:
					self._last_target_error = f"maintenance send failed: {exc}"
				self._close_maintenance_socket()
				break

		try:
			while True:
				ready, _, _ = select.select([maintenance_socket], [], [], 0)
				if not ready:
					break
				chunk = maintenance_socket.recv(65536)
				if not chunk:
					self._close_maintenance_socket()
					break
				self._monitor.feed("down", chunk)
				with self._lock:
					self._last_target_rx_ts = time.time()
		except Exception as exc:
			with self._lock:
				self._last_target_error = f"maintenance read failed: {exc}"
			self._close_maintenance_socket()

	def _accept_loop(self):
		enabled, listen_host, listen_port, target_host, target_port = self._config or (0, SERIAL_PROXY_LISTEN_HOST, SERIAL_PROXY_LISTEN_PORT, SERIAL_PROXY_TARGET_HOST, SERIAL_PROXY_TARGET_PORT)
		if enabled != 1:
			return
		server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		try:
			server.bind((listen_host, listen_port))
			server.listen(5)
			server.settimeout(1.0)
			with self._lock:
				self._listen_socket = server
			self._log(f"serial proxy listening on {listen_host}:{listen_port} -> {target_host}:{target_port}")
			while True:
				with self._lock:
					if not self._running:
						break
				self._maintenance_tick(target_host, target_port)
				try:
					client_sock, client_addr = server.accept()
				except socket.timeout:
					continue
				except Exception as exc:
					with self._lock:
						still_running = bool(self._running)
					if still_running:
						self._log(f"serial proxy accept failed: {exc}")
					break
				with self._lock:
					self._active_clients += 1
					active_clients = int(self._active_clients)
				self._log(f"bridge connected: {client_addr} (VSerialPort={active_clients})")
				t = threading.Thread(target=self._handle_client, args=(client_sock, client_addr, target_host, target_port), daemon=True)
				t.start()
				with self._lock:
					self._client_threads.append(t)
		except Exception as exc:
			self._log(f"serial proxy failed: {exc}")
		finally:
			try:
				server.close()
			except Exception:
				pass
			self._close_maintenance_socket()

	def _handle_client(self, client_sock, client_addr, target_host, target_port):
		start_ts = time.time()
		session_reason = "session_closed"
		session_where = "unknown"
		reconnect_count = 0
		passthrough_active = False
		try:
			while True:
				with self._lock:
					if not self._running:
						session_reason = "proxy_stopping"
						break

				waiting_since = time.time()
				first_chunk = b""
				while True:
					with self._lock:
						if not self._running:
							session_reason = "proxy_stopping"
							session_where = "up"
							break
					try:
						ready, _, _ = select.select([client_sock], [], [], 0.5)
					except Exception as exc:
						session_reason = str(exc)
						session_where = "up"
						break
					if not ready:
						# After a target drop (reconnect_count > 0) if the client only
						# sends realtime polls and never sends an activation payload,
						# give up after a timeout instead of spinning forever.
						if reconnect_count > 0 and (time.time() - waiting_since) > 8.0:
							session_reason = "reconnect_timeout"
							session_where = "up"
							break
						continue
					try:
						chunk = client_sock.recv(65536)
					except Exception as exc:
						session_reason = str(exc)
						session_where = "up"
						break
					if not chunk:
						session_reason = "client_closed"
						session_where = "up"
						break
					if self._is_passthrough_activation_payload(chunk):
						first_chunk = chunk
						break
					continue
					break

				if not first_chunk:
					break

				# Free the maintenance link before opening passthrough.
				# Some serial backends allow only one active TCP peer and may reset
				# the new session if maintenance is still attached.
				self._close_maintenance_socket()

				target_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				try:
					target_sock.settimeout(10)
					target_sock.connect((target_host, target_port))
					target_sock.settimeout(None)
					with self._lock:
						self._passthrough_clients += 1
						passthrough_active = True
						self._last_target_connect_ts = time.time()
						self._last_target_error = ""
				except Exception as exc:
					try:
						target_sock.close()
					except Exception:
						pass
					self._log(f"proxy connect failed from {client_addr}: {exc}")
					with self._lock:
						self._last_target_error = f"client connect failed: {exc}"
					if not self._client_socket_alive(client_sock):
						session_reason = "client_closed"
						session_where = "up"
						break
					time.sleep(0.20)
					reconnect_count += 1
					continue

				if reconnect_count > 0:
					self._log(f"serial proxy passthrough resumed: client={client_addr} target={target_host}:{target_port} reconnect={reconnect_count}")
				else:
					self._log(f"serial proxy passthrough started: client={client_addr} target={target_host}:{target_port}")

				try:
					target_sock.sendall(first_chunk)
					self._monitor.feed("up", first_chunk)
					with self._lock:
						self._last_target_tx_ts = time.time()
				except Exception as exc:
					with self._lock:
						self._last_target_error = f"relay socket error (up): {exc}"
					try:
						target_sock.close()
					except Exception:
						pass
					with self._lock:
						if passthrough_active:
							self._passthrough_clients = max(0, self._passthrough_clients - 1)
							passthrough_active = False
					if self._client_socket_alive(client_sock):
						reconnect_count += 1
						self._log(f"serial proxy target peer closed; reconnecting passthrough for client={client_addr} (reconnect={reconnect_count})")
						time.sleep(0.05)
						continue
					session_reason = str(exc)
					session_where = "up"
					break

				stop_event = threading.Event()
				stop_meta = {"reason": "", "where": ""}
				session_activity = {
					"last_up_ts": time.time(),
					"last_up_payload": self._payload_preview(first_chunk),
					"last_up_realtime_only": bool(self._is_realtime_only_payload(first_chunk)),
					"last_nontransport_up_ts": time.time(),
					"last_down_ts": time.time(),
				}
				target_write_lock = threading.Lock()
				up = threading.Thread(target=self._pump, args=(client_sock, target_sock, "up", stop_event, stop_meta, session_activity, target_write_lock), daemon=True)
				down = threading.Thread(target=self._pump, args=(target_sock, client_sock, "down", stop_event, stop_meta, session_activity), daemon=True)
				up.start()
				down.start()
				while not stop_event.is_set():
					with self._lock:
						extend_on_rt = bool(self._passthrough_extend_on_realtime)
						queue_item = dict(self._command_queue[0]) if self._command_queue else None
					now_ts = time.time()
					# LB disconnect detection: when LB closes its COM port, HW-VSP keeps the
					# TCP connection alive but sends only fff1 transport-control frames.
					# Keep the passthrough alive while the target is still talking back; only
					# close if both directions have been effectively idle for idle_timeout_s.
					last_nontransport_ts = float(session_activity.get("last_nontransport_up_ts", 0.0))
					last_down_ts = float(session_activity.get("last_down_ts", 0.0))
					if (
						last_nontransport_ts > 0
						and (now_ts - last_nontransport_ts) >= self._passthrough_idle_timeout_s
						and (last_down_ts <= 0 or (now_ts - last_down_ts) >= self._passthrough_idle_timeout_s)
					):
						stop_meta["reason"] = "transport_only_timeout"
						stop_meta["where"] = "up"
						stop_event.set()
						break
					# Keepalive mode: idle timeout — LB is present but no real commands for a while.
					if extend_on_rt:
						last_up_ts = float(session_activity.get("last_up_ts", 0.0))
						if (
							last_up_ts > 0
							and (now_ts - last_up_ts) >= self._passthrough_idle_timeout_s
							and (last_down_ts <= 0 or (now_ts - last_down_ts) >= self._passthrough_idle_timeout_s)
							and self._client_socket_alive(client_sock)
						):
							stop_meta["reason"] = "idle_timeout"
							stop_meta["where"] = "up"
							stop_event.set()
							break
					# Hybrid mode: inject queued local commands when LB is only polling (?).
					if (not extend_on_rt) and queue_item is not None and bool(session_activity.get("last_up_realtime_only", False)):
						laser_active = bool(self._monitor.snapshot().get("laser_active", False))
						if not laser_active:
							payload = self._encode_command(queue_item.get("command", ""))
							if payload:
								try:
									with target_write_lock:
										target_sock.sendall(payload)
									self._monitor.feed("up", payload)
									now_ts = time.time()
									session_activity["last_up_ts"] = now_ts
									session_activity["last_up_payload"] = self._payload_preview(payload)
									session_activity["last_up_realtime_only"] = bool(self._is_realtime_only_payload(payload))
									session_activity["last_nontransport_up_ts"] = now_ts
									with self._lock:
										if self._command_queue and int(self._command_queue[0].get("id", 0)) == int(queue_item.get("id", 0)):
											self._command_queue.pop(0)
										self._last_target_tx_ts = now_ts
										self._last_command_result = {
											"id": queue_item.get("id"),
											"command": str(queue_item.get("command", ""))[:180],
											"source": str(queue_item.get("source", ""))[:40],
											"sent_ts": now_ts,
										}
								except Exception as exc:
									with self._lock:
										self._last_target_error = f"passthrough mixed send failed: {exc}"
									stop_meta["reason"] = str(exc)
									stop_meta["where"] = "up"
									stop_event.set()
									break
							else:
								with self._lock:
									if self._command_queue and int(self._command_queue[0].get("id", 0)) == int(queue_item.get("id", 0)):
										self._command_queue.pop(0)
					time.sleep(0.05)
				up.join(timeout=0.3)
				down.join(timeout=0.3)

				reason = str(stop_meta.get("reason") or "session_closed")
				where = str(stop_meta.get("where") or "unknown")
				session_reason = reason
				session_where = where

				try:
					target_sock.close()
				except Exception:
					pass
				with self._lock:
					if passthrough_active:
						self._passthrough_clients = max(0, self._passthrough_clients - 1)
						passthrough_active = False

				if reason == "idle_timeout" and self._client_socket_alive(client_sock):
					last_up_payload = str(session_activity.get("last_up_payload", "") or "")
					last_up_rt = bool(session_activity.get("last_up_realtime_only", False))
					self._log(f"serial proxy passthrough idle timeout; waiting for new client traffic client={client_addr} last_up={last_up_payload} realtime_only={last_up_rt}")
					continue

				reason_lower = reason.lower()
				target_drop = (
					where == "down"
					and (
						reason == "peer_closed"
						or "connection reset by peer" in reason_lower
						or "errno 104" in reason_lower
					)
				)
				if target_drop and self._client_socket_alive(client_sock):
					reconnect_count += 1
					self._log(f"serial proxy target peer closed; reconnecting passthrough for client={client_addr} (reconnect={reconnect_count})")
					time.sleep(0.05)
					continue
				break
		finally:
			duration_s = max(0.0, time.time() - start_ts)
			with self._lock:
				if passthrough_active:
					self._passthrough_clients = max(0, self._passthrough_clients - 1)
					passthrough_active = False
			self._log(f"serial proxy passthrough ended: client={client_addr} where={session_where} reason={session_reason} reconnects={reconnect_count} duration={duration_s:.2f}s")
			try:
				client_sock.close()
			except Exception:
				pass
			with self._lock:
				self._active_clients = max(0, self._active_clients - 1)
				active_clients = int(self._active_clients)
			self._log(f"bridge disconnected: {client_addr} (active_clients={active_clients})")

	def _client_socket_alive(self, sock):
		try:
			sock.setblocking(False)
			try:
				peek = sock.recv(1, socket.MSG_PEEK)
				if peek == b"":
					return False
				return True
			except BlockingIOError:
				return True
			except InterruptedError:
				return True
			except Exception:
				return False
		finally:
			try:
				sock.setblocking(True)
			except Exception:
				pass

	def _is_passthrough_activation_payload(self, payload):
		if not payload:
			return False
		if self._is_realtime_only_payload(payload) and not self._is_transport_control_payload(payload):
			return True
		text = payload.decode("utf-8", errors="ignore").upper()
		if not text:
			return False
		if re.search(r'\$(?:H|X|I|N|G|J|RST|SLP|#|\$|\?)', text):
			return True
		if re.search(r'\b(?:G\d+|M\d+)\b', text):
			return True
		if re.search(r'\b(?:X|Y|Z|F|S|I|J|P|R|T)-?\d', text):
			return True
		if re.search(r'[\r\n;]', text) and re.search(r'[A-Z0-9]', text):
			return True
		return False

	def _is_realtime_only_payload(self, payload):
		if not payload:
			return False
		realtime = {ord('?'), ord('!'), ord('~'), 0x18, ord('\r'), ord('\n'), ord(' '), ord('\t')}
		return all((b in realtime) for b in payload)

	def _is_transport_control_payload(self, payload):
		if not payload:
			return False
		# HW-VSP transport keepalive/control frame (commonly 0xFF 0xF1).
		if payload in (b"\xff\xf1", b"\xf1\xff"):
			return True
		if all((b in (0xFF, 0xF1)) for b in payload):
			return True
		# Treat short high-bit binary control bursts as transport noise.
		if len(payload) <= 8 and all((b >= 0x80) for b in payload):
			return True
		return False

	def _payload_preview(self, payload):
		if not payload:
			return ""
		try:
			text = payload.decode("utf-8", errors="ignore")
		except Exception:
			text = ""
		text = text.replace("\r", "\\r").replace("\n", "\\n")
		text = re.sub(r'\s+', ' ', text).strip()
		if not text:
			text = payload[:24].hex()
		if len(text) > 80:
			text = text[:80] + "..."
		return text

	def _pump(self, source_sock, dest_sock, direction, stop_event, stop_meta, session_activity=None, send_lock=None):
		while not stop_event.is_set():
			try:
				ready, _, _ = select.select([source_sock], [], [], 1.0)
				if not ready:
					continue
				chunk = source_sock.recv(65536)
				if not chunk:
					if not stop_event.is_set():
						stop_meta["reason"] = "peer_closed"
						stop_meta["where"] = direction
					stop_event.set()
					break
				if send_lock is not None:
					with send_lock:
						dest_sock.sendall(chunk)
				else:
					dest_sock.sendall(chunk)
				self._monitor.feed(direction, chunk)
				now = time.time()
				if direction == "up" and session_activity is not None:
					realtime_only = self._is_realtime_only_payload(chunk)
					transport_only = self._is_transport_control_payload(chunk)
					payload_preview = self._payload_preview(chunk)
					session_activity["last_up_payload"] = payload_preview
					session_activity["last_up_realtime_only"] = bool(realtime_only or transport_only)
					with self._lock:
						extend_on_rt = self._passthrough_extend_on_realtime
					if not transport_only:
						session_activity["last_nontransport_up_ts"] = now
					if not transport_only and (not realtime_only or extend_on_rt):
						session_activity["last_up_ts"] = now
				elif direction == "down" and session_activity is not None:
					session_activity["last_down_ts"] = now
				with self._lock:
					if direction == "up":
						self._last_target_tx_ts = now
					else:
						self._last_target_rx_ts = now
			except Exception as exc:
				with self._lock:
					self._last_target_error = f"relay socket error ({direction}): {exc}"
				if not stop_event.is_set():
					stop_meta["reason"] = str(exc)
					stop_meta["where"] = direction
				stop_event.set()
				break
