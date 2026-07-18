"""
telemetry.py
────────────
MAVLink mesajlarını parse eder, tutarlı TelemetryState nesneleri tutar
ve EventBus üzerinden GUI'ye push eder.

v4: Her araç (İHA/İDA) için bağımsız parser instance'ı.
    "mavlink_msg_iha" / "mavlink_msg_ida" event'lerini dinler.
    "telemetry" event'i {"vehicle": ..., "payload": TelemetryState} şeklinde.

Bu sınıf GUI'ye DOKUNMAZ.
"""

import math
import time
import logging
from dataclasses import dataclass, field, asdict
from typing import Optional
from pymavlink import mavutil

from vehicle_modes import mode_id_to_name, frame_from_mav_type, frame_for_vehicle

log = logging.getLogger("telemetry")

# GPS fix kalite metinleri
GPS_FIX_TYPE: dict[int, str] = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D FIX",
    3: "3D FIX",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed",
}

# ArduPilot MAV_STATE
MAV_STATE_LABELS: dict[int, str] = {
    0: "UNINIT",
    1: "BOOT",
    2: "CALIBRATING",
    3: "STANDBY",
    4: "ACTIVE",
    5: "CRITICAL",
    6: "EMERGENCY",
    7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}


@dataclass
class TelemetryState:
    """Tek bir araç için anlık telemetri durumu."""

    # ── GPS ─────────────────────────
    lat:           float = 0.0
    lon:           float = 0.0
    alt_rel:       float = 0.0    # relative alt (m)
    alt_msl:       float = 0.0    # MSL alt (m)
    gps_fix:       int   = 0
    gps_fix_label: str   = "NO GPS"
    satellites:    int   = 0
    hdop:          float = 99.9

    # ── Hız & Yön ───────────────────
    ground_speed: float = 0.0    # m/s
    air_speed:    float = 0.0    # m/s
    vspeed:       float = 0.0    # m/s (climb rate)
    heading:      float = 0.0    # derece

    # ── Tutum ───────────────────────
    roll:  float = 0.0
    pitch: float = 0.0
    yaw:   float = 0.0

    # ── Batarya ─────────────────────
    battery_voltage:   float = 0.0   # V
    battery_current:   float = 0.0   # A
    battery_remaining: int   = -1    # % (-1 = bilinmiyor)

    # ── Sistem Durumu ────────────────
    armed:       bool  = False
    mode:        str   = "UNKNOWN"
    mode_id:     int   = -1
    sys_status:  str   = "UNINIT"
    flight_time: float = 0.0         # saniye (arm'dan bu yana)

    # ── Kontrol / Setpoint (kontrol grafikleri için) ──
    nav_bearing:    float = 0.0      # controller hedef heading (deg) — NAV_CONTROLLER_OUTPUT
    target_bearing: float = 0.0      # hedef waypoint bearing (deg)
    target_speed_error: float = 0.0  # hedef hız - mevcut hız (m/s) — NAV_CONTROLLER_OUTPUT.aspd_error
    has_nav_output: bool  = False    # NAV_CONTROLLER_OUTPUT alındı mı
    target_speed:     float = 0.0    # istenen hız (m/s) — POSITION_TARGET (guided hız kontrolü)
    target_vx:        float = 0.0    # istenen x hızı (m/s) — POSITION_TARGET.vx
    target_vy:        float = 0.0    # istenen y hızı (m/s) — POSITION_TARGET.vy
    has_target_speed: bool  = False  # geçerli bir hız hedefi var mı
    servo_raw:      list  = field(default_factory=list)  # servo PWM (us), kanal 1..N
    has_servo:      bool  = False    # SERVO_OUTPUT_RAW alındı mı
    params:         dict  = field(default_factory=dict)   # istenen parametreler (PARAM_VALUE)

    # ── Bağlantı ────────────────────
    last_heartbeat: float = 0.0
    rssi:           int   = 0
    msg:            object = field(default=None, repr=False)

    # ── Zaman damgası ────────────────
    ts: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("msg", None)  # msg serileştirilemez
        return d


class VehicleTelemetry:
    """
    Tek bir araç için MAVLink mesajlarını parse eden sınıf.
    EventBus'tan "mavlink_msg_{vehicle_id}" event'lerini dinler.
    "telemetry" event'i {"vehicle": vehicle_id, "payload": TelemetryState} olarak yayınlar.

    Mod adları araç tipine (Copter/Rover) göre çözülür; bkz. vehicle_modes.py.
    """

    PUBLISH_INTERVAL = 0.10  # GUI'ye en fazla 10 Hz telemetry yayını

    def __init__(self, event_bus, vehicle_id: str):
        self.bus        = event_bus
        self.vehicle_id = vehicle_id
        self.state      = TelemetryState()
        self._arm_start_time: float = 0.0
        self._last_publish_ts: float = 0.0

        # Firmware tipi: önce araç kimliğinden yapılandırılır, HEARTBEAT
        # geldikçe MAV_TYPE ile doğrulanır/güncellenir.
        self._frame: str = frame_for_vehicle(vehicle_id)

        # Araç-spesifik event'e abone ol
        self.bus.subscribe(f"mavlink_msg_{vehicle_id}", self._on_mavlink_msg)
        log.info(f"[{vehicle_id}] VehicleTelemetry başlatıldı (frame={self._frame})")

    # ──────────────────────────────────────
    #  HANDLER
    # ──────────────────────────────────────

    def _on_mavlink_msg(self, data: dict) -> None:
        msg_type = data.get("type")
        msg      = data.get("msg")
        ts       = data.get("ts", time.time())

        handler = self._handlers.get(msg_type)
        if handler and msg is not None:
            try:
                handler(self, msg, ts)
                self.state.ts = ts
                now = time.time()
                if msg_type == "HEARTBEAT" or now - self._last_publish_ts >= self.PUBLISH_INTERVAL:
                    self._last_publish_ts = now
                    # Payload'u vehicle_id ile birlikte yayınla
                    self.bus.publish("telemetry", {
                        "vehicle": self.vehicle_id,
                        "payload": self.state,
                    })
            except Exception as e:
                log.error(f"[{self.vehicle_id}] Telemetry parse hatası [{msg_type}]: {e}")

    # ──────────────────────────────────────
    #  MESAJ PARSER'LAR
    # ──────────────────────────────────────

    def _parse_heartbeat(self, msg, ts: float) -> None:
        self.state.last_heartbeat = ts

        was_armed = self.state.armed
        self.state.armed = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if self.state.armed and not was_armed:
            self._arm_start_time = ts
        if self.state.armed:
            self.state.flight_time = ts - self._arm_start_time

        mode_id = msg.custom_mode
        self.state.mode_id = mode_id

        # HEARTBEAT.type'tan firmware tipini doğrula (Copter/Rover).
        # Tanınırsa _frame güncellenir; tanınmazsa yapılandırılmış değer kalır.
        detected = frame_from_mav_type(getattr(msg, "type", None))
        if detected is not None:
            self._frame = detected

        # Mod adı araç tipine göre çözülür (örn. custom_mode=4 →
        # İHA'da GUIDED, İDA'da HOLD). Böylece görüntülenen mod Mission
        # Planner ile birebir eşleşir.
        self.state.mode = mode_id_to_name(
            self.vehicle_id, mode_id, frame=self._frame
        )
        self.state.sys_status = MAV_STATE_LABELS.get(msg.system_status, "UNKNOWN")
        self.state.msg = msg

    def _parse_global_position_int(self, msg, ts: float) -> None:
        self.state.lat     = msg.lat / 1e7
        self.state.lon     = msg.lon / 1e7
        self.state.alt_rel = msg.relative_alt / 1000.0
        self.state.alt_msl = msg.alt / 1000.0
        self.state.heading = (msg.hdg / 100.0) if msg.hdg != 65535 else self.state.heading
        self.state.vspeed  = msg.vz / 100.0   # cm/s → m/s

    def _parse_gps_raw_int(self, msg, ts: float) -> None:
        self.state.gps_fix       = msg.fix_type
        self.state.gps_fix_label = GPS_FIX_TYPE.get(msg.fix_type, "UNKNOWN")
        self.state.satellites    = msg.satellites_visible
        self.state.hdop          = (msg.eph / 100.0) if msg.eph != 65535 else 99.9

    def _parse_vfr_hud(self, msg, ts: float) -> None:
        self.state.ground_speed = msg.groundspeed
        self.state.air_speed    = msg.airspeed
        self.state.alt_rel      = msg.alt
        self.state.vspeed       = msg.climb

    def _parse_attitude(self, msg, ts: float) -> None:
        self.state.roll  = math.degrees(msg.roll)
        self.state.pitch = math.degrees(msg.pitch)
        self.state.yaw   = math.degrees(msg.yaw) % 360

    def _parse_sys_status(self, msg, ts: float) -> None:
        self.state.battery_voltage   = msg.voltage_battery / 1000.0
        self.state.battery_current   = msg.current_battery / 100.0
        self.state.battery_remaining = msg.battery_remaining

    def _parse_battery_status(self, msg, ts: float) -> None:
        if msg.voltages and msg.voltages[0] != 65535:
            self.state.battery_voltage = msg.voltages[0] / 1000.0
        if msg.current_battery != -1:
            self.state.battery_current = msg.current_battery / 100.0
        if msg.battery_remaining != -1:
            self.state.battery_remaining = msg.battery_remaining

    def _parse_statustext(self, msg, ts: float) -> None:
        severity_map = {
            0: "error", 1: "error", 2: "error",
            3: "warn",  4: "warn",  5: "info",
            6: "info",  7: "dim",
        }
        level = severity_map.get(msg.severity, "info")
        self.bus.publish("log", {
            "msg":     f"[FC] {msg.text.strip()}",
            "level":   level,
            "vehicle": self.vehicle_id,
        })

    def _parse_nav_controller_output(self, msg, ts: float) -> None:
        # Kontrol döngüsünün hedeflediği heading/bearing (derece).
        # Grafiklerde "heading isteği (setpoint)" olarak kullanılır.
        self.state.nav_bearing    = float(msg.nav_bearing)
        self.state.target_bearing = float(msg.target_bearing)
        self.state.target_speed_error = float(getattr(msg, "aspd_error", 0.0) or 0.0)
        self.state.has_nav_output = True
        target_speed, source = self._fallback_target_speed()
        if target_speed is not None:
            print(
                f"target hız = {target_speed:.3f} m/s "
                f"[{self.vehicle_id} NAV_CONTROLLER_OUTPUT source={source} "
                f"ground_speed={self.state.ground_speed:.3f} m/s "
                f"aspd_error={self.state.target_speed_error:.3f} m/s]",
                flush=True,
            )

    def _parse_servo_output_raw(self, msg, ts: float) -> None:
        # Servo/thruster PWM çıkışları (mikrosaniye). İlk 8 kanal saklanır.
        # Skid-steer Rover varsayılanı: kanal 1 = sol, kanal 3 = sağ thruster.
        self.state.servo_raw = [
            int(getattr(msg, f"servo{i}_raw", 0) or 0) for i in range(1, 9)
        ]
        self.state.has_servo = True

    def _parse_position_target(self, msg, ts: float) -> None:
        # İstenen hız: hedef hız vektörünün yatay büyüklüğü (vx, vy).
        # ArduPilot bu mesajı yalnızca GUIDED hız kontrolünde gönderir;
        # yalnızca konum hedefi varsa VX/VY "ignore" işaretlenir → hız yok.
        mask = int(getattr(msg, "type_mask", 0))
        VX_IGNORE, VY_IGNORE = 0x08, 0x10
        if (mask & VX_IGNORE) and (mask & VY_IGNORE):
            self.state.target_vx = 0.0
            self.state.target_vy = 0.0
            self.state.target_speed = 0.0
            self.state.has_target_speed = False
            target_speed, source = self._fallback_target_speed()
            target_text = (
                f"{target_speed:.3f} m/s [{source}]"
                if target_speed is not None else "ignored"
            )
            print(
                f"vx = ignored , vy = ignored , istenen hız = ignored "
                f", target hız = {target_text} "
                f"[{self.vehicle_id} {msg.get_type()} type_mask=0x{mask:04x}]",
                flush=True,
            )
            return
        vx = float(getattr(msg, "vx", 0.0) or 0.0)
        vy = float(getattr(msg, "vy", 0.0) or 0.0)
        self.state.target_vx = vx
        self.state.target_vy = vy
        self.state.target_speed = math.hypot(vx, vy)
        self.state.has_target_speed = True
        print(
            f"vx = {vx:.3f} m/s , vy = {vy:.3f} m/s , "
            f"istenen hız = {self.state.target_speed:.3f} m/s , "
            f"target hız = {self.state.target_speed:.3f} m/s [POSITION_TARGET] "
            f"[{self.vehicle_id} {msg.get_type()} type_mask=0x{mask:04x}]",
            flush=True,
        )

    def _fallback_target_speed(self):
        if self.state.has_nav_output and abs(self.state.target_speed_error) > 0.05:
            speed = self.state.ground_speed + self.state.target_speed_error
            return max(0.0, speed), "NAV_CONTROLLER_OUTPUT"

        p = self.state.params
        wp = p.get("WP_SPEED")
        if wp is not None and wp > 0:
            return float(wp), "WP_SPEED"
        if "CRUISE_SPEED" in p:
            return float(p["CRUISE_SPEED"]), "CRUISE_SPEED"
        if "WPNAV_SPEED" in p:
            return float(p["WPNAV_SPEED"]) / 100.0, "WPNAV_SPEED"
        return None, None

    def _parse_param_value(self, msg, ts: float) -> None:
        # İstenen parametreleri sakla (örn. WP_SPEED, CRUISE_SPEED).
        try:
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", "ignore")
            pid = pid.replace("\x00", "").strip()
            self.state.params[pid] = float(msg.param_value)
        except Exception:
            pass

    # Dispatcher tablosu
    _handlers = {
        "HEARTBEAT":                 _parse_heartbeat,
        "GLOBAL_POSITION_INT":       _parse_global_position_int,
        "GPS_RAW_INT":               _parse_gps_raw_int,
        "VFR_HUD":                   _parse_vfr_hud,
        "ATTITUDE":                  _parse_attitude,
        "SYS_STATUS":                _parse_sys_status,
        "BATTERY_STATUS":            _parse_battery_status,
        "STATUSTEXT":                _parse_statustext,
        "NAV_CONTROLLER_OUTPUT":     _parse_nav_controller_output,
        "SERVO_OUTPUT_RAW":          _parse_servo_output_raw,
        "POSITION_TARGET_GLOBAL_INT": _parse_position_target,
        "POSITION_TARGET_LOCAL_NED":  _parse_position_target,
        "PARAM_VALUE":               _parse_param_value,
    }


class TelemetryManager:
    """
    İHA ve İDA için ayrı VehicleTelemetry instance'ları yönetir.
    İleride yeni araç eklemek: vehicles listesine eklemek yeterli.
    """

    def __init__(self, event_bus, vehicle_ids: tuple[str, ...] = ("iha", "ida")):
        self._parsers: dict[str, VehicleTelemetry] = {
            vid: VehicleTelemetry(event_bus, vid)
            for vid in vehicle_ids
        }
        log.info("TelemetryManager başlatıldı")

    def get_state(self, vehicle_id: str) -> Optional[TelemetryState]:
        parser = self._parsers.get(vehicle_id)
        return parser.state if parser else None
