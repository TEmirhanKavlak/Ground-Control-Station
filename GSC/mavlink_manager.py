"""
mavlink_manager.py
──────────────────
Araç-spesifik MAVLink bağlantı yöneticisi.

v4: Her araç (İHA/İDA) kendi VehicleConnection instance'ına sahip.
    Tüm event'ler vehicle_id içerir.
    VehicleManager iki aracı tek noktadan yönetir.
"""

import threading
import time
import logging
from enum import Enum, auto
from typing import Optional

import serial.tools.list_ports
from pymavlink import mavutil

from vehicle_modes import (
    mode_name_to_id,
    mode_names_for_vehicle,
    frame_for_vehicle,
    COPTER_MODES,
)

log = logging.getLogger("mavlink")


# ─────────────────────────────────────────
#  BAĞLANTI DURUMU
# ─────────────────────────────────────────
class ConnState(Enum):
    DISCONNECTED = auto()
    CONNECTING   = auto()
    CONNECTED    = auto()
    RECONNECTING = auto()


# ─────────────────────────────────────────
#  FLIGHT MODLARI
# ─────────────────────────────────────────
#  Mod numaraları araç tipine göre değişir; tek doğruluk kaynağı
#  vehicle_modes.py modülüdür. Aşağıdaki FLIGHT_MODES yalnızca geriye
#  dönük uyumluluk için Copter haritasına işaret eder (eski importlar
#  kırılmasın diye). Araç-spesifik gönderim için VehicleConnection.set_mode
#  vehicle_modes.mode_name_to_id() kullanır.
FLIGHT_MODES: dict[str, int] = COPTER_MODES
MODE_ID_TO_NAME: dict[int, str] = {v: k for k, v in COPTER_MODES.items()}


# ─────────────────────────────────────────
#  GÖREV (MISSION) PROTOKOLÜ
# ─────────────────────────────────────────
#  Araçtan gelen bu mesajlar recv loop tarafından upload state machine'ine
#  yönlendirilir (telemetri bus'ına gönderilmez).
_MISSION_PROTOCOL_MSGS = frozenset({
    "MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
})


# ─────────────────────────────────────────
#  YARDIMCI FONKSİYON
# ─────────────────────────────────────────
def list_serial_ports() -> list[str]:
    """Mevcut seri portları listele (sıralı)."""
    try:
        ports = serial.tools.list_ports.comports()
        return sorted([p.device for p in ports])
    except Exception as e:
        log.error(f"Port listeleme hatası: {e}")
        return []


# ─────────────────────────────────────────
#  TEK ARAÇ BAĞLANTISI
# ─────────────────────────────────────────
class VehicleConnection:
    """
    Tek bir araç için MAVLink bağlantısı.
    Bağımsız connect/disconnect/reconnect döngüsü.
    Bağımsız telemetry event yayını.
    """

    HEARTBEAT_TIMEOUT = 5.0
    RECONNECT_DELAY   = 3.0
    RECV_TIMEOUT      = 0.1
    CONNECT_TIMEOUT   = 10

    INTERESTING_MSGS = frozenset({
        "GLOBAL_POSITION_INT", "HEARTBEAT", "SYS_STATUS",
        "BATTERY_STATUS", "GPS_RAW_INT", "VFR_HUD",
        "ATTITUDE", "STATUSTEXT",
        # Kontrol grafikleri için
        "NAV_CONTROLLER_OUTPUT", "SERVO_OUTPUT_RAW",
        "POSITION_TARGET_GLOBAL_INT", "POSITION_TARGET_LOCAL_NED",
        "PARAM_VALUE",
        # Görev (mission) upload protokolü
        "MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
    })

    def __init__(self, event_bus, vehicle_id: str):
        self.bus        = event_bus
        self.vehicle_id = vehicle_id

        self.port: Optional[str] = None
        self.baud: int           = 57600
        self.conn                = None
        self.state               = ConnState.DISCONNECTED

        self._running          = False
        self._connect_thread:  Optional[threading.Thread] = None
        self._hb_thread:       Optional[threading.Thread] = None
        self._last_heartbeat_ts: float = 0.0

        # Aynı bağlantıya farklı thread'lerden yazım yapılabildiğinden
        # (komutlar GUI thread'inden, köprü kendi worker thread'inden)
        # tüm gönderimler bu kilitle serileştirilir.
        self._send_lock = threading.Lock()

        # Görev (mission) upload durumu.
        #   _mission_lock  → aynı anda tek upload (worker thread'den tutulur)
        #   _mission       → aktif upload state dict (recv thread item gönderir)
        #   _mission_active→ AUTO moduna erken geçişi engellemek için bayrak
        self._mission_lock   = threading.Lock()
        self._mission         = None
        self._mission_active  = False

        # Kontrol grafikleri açıkken thruster (SERVO_OUTPUT_RAW) stream'i
        # talep edilir. Pencere kapalıyken bu veri araçtan İSTENMEZ.
        self._graph_streams   = False

    # ──────────────────────────────────────
    #  PUBLIC API
    # ──────────────────────────────────────

    def configure(self, port: str, baud: int) -> None:
        """Bağlantı parametrelerini ayarla."""
        self.port = port
        self.baud = baud

    def start(self) -> None:
        """Bağlantıyı başlat (non-blocking)."""
        if self._running:
            log.warning(f"[{self.vehicle_id}] Zaten çalışıyor")
            return
        if not self.port:
            self._publish_log("COM port seçilmedi!", "error")
            return
        self._running = True
        self._connect_thread = threading.Thread(
            target=self._connect_loop,
            daemon=True,
            name=f"mav-{self.vehicle_id}-connect",
        )
        self._connect_thread.start()

    def stop(self) -> None:
        """Bağlantıyı durdur."""
        self._running = False
        self._close_conn()
        self._set_state(ConnState.DISCONNECTED)

    def is_connected(self) -> bool:
        return self.state == ConnState.CONNECTED

    # ── Komutlar ──────────────────────────

    def arm(self) -> None:
        self._send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
        self._publish_log("ARM komutu gönderildi", "warn")

    def disarm(self, force: bool = False) -> None:
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0, param2=21196 if force else 0,
        )
        self._publish_log("DISARM komutu gönderildi", "info")

    def set_mode(self, mode_name: str) -> bool:
        if not self.conn:
            self._publish_log("Bağlantı yok, mod değiştirilemedi", "warn")
            return False

        mode_upper = mode_name.upper()

        # Görev yüklenirken AUTO'ya geçişi engelle: aksi halde araç henüz
        # tamamlanmamış / eski görevi çalıştırabilir.
        if mode_upper == "AUTO" and self._mission_active:
            self._publish_log(
                "Görev yükleniyor; AUTO ertelendi. Yükleme bitince tekrar deneyin.",
                "warn",
            )
            return False

        # Mod numarası araç tipine (Copter/Rover) göre çözülür.
        # Örn: GUIDED → İHA'da 4, İDA'da 15.
        mode_id = mode_name_to_id(self.vehicle_id, mode_name)
        if mode_id is None:
            frame = frame_for_vehicle(self.vehicle_id)
            self._publish_log(
                f"Bilinmeyen mod ({frame}): {mode_name}", "error"
            )
            return False

        # SET_MODE mesajını custom_mode bayrağıyla, gönderim kilidi altında yolla.
        # (pymavlink set_mode_apm ile aynı davranış; ancak _send_lock ile
        #  serileştirilir, böylece recv/diğer thread'lerle çakışmaz.)
        try:
            with self._send_lock:
                self.conn.mav.set_mode_send(
                    self.conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
        except Exception as e:
            log.error(f"[{self.vehicle_id}] set_mode hatası: {e}")
            self._publish_log(f"Mod gönderim hatası: {e}", "error")
            return False

        self._publish_log(
            f"Mod → {mode_name.upper()} (custom_mode={mode_id})", "info"
        )
        return True

    def takeoff(self, altitude_m: float = 10.0) -> None:
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param7=altitude_m
        )
        self._publish_log(f"Takeoff → {altitude_m}m", "warn")

    def rtl(self) -> None:
        self.set_mode("RTL")
        self._publish_log("RTL komutu gönderildi", "warn")

    def emergency_stop(self) -> None:
        self.disarm(force=True)
        self._publish_log("!!! ACİL DURDURMA !!!", "error")

    # ──────────────────────────────────────
    #  GÖREV (MISSION) UPLOAD — tam MAVLink protokolü
    # ──────────────────────────────────────

    def upload_mission(self, waypoints) -> bool:
        """
        Tüm waypoint listesini MAVLink mission protokolüyle araca yükler.

        waypoints: [(lat, lon, alt), ...]  (alt = home'a göre, metre)

        Akış (mavlink.io mission upload protokolü):
            1. MISSION_CLEAR_ALL          → eski görevi sil
            2. MISSION_COUNT(N+1)         → upload başlat
            3. Araç MISSION_REQUEST(_INT) → her seq için MISSION_ITEM_INT yolla
            4. Araç MISSION_ACK(ACCEPTED) → upload tamamlandı
            5. MISSION_SET_CURRENT(0)     → görev indeksini sıfırla

        ArduPilot'ta seq 0 = home pozisyonudur (araç kendi home'uyla
        doldurur); gerçek waypoint'ler seq 1..N'e yerleştirilir.

        Döner: True → MAV_MISSION_ACCEPTED alındı, False → hata/zaman aşımı.
        Bu metod BLOKLAR; GUI'yi dondurmamak için worker thread'den çağırın.
        """
        if not self.conn:
            self._publish_log("Görev gönderilemedi: bağlantı yok", "warn")
            return False

        # Boş liste → sadece mevcut görevi temizle
        if not waypoints:
            if self._mission_lock.acquire(blocking=False):
                try:
                    self._send_mission_clear_all()
                    self._publish_log("Görev temizlendi (MISSION_CLEAR_ALL)", "info")
                finally:
                    self._mission_lock.release()
            return True

        # Aynı anda tek upload
        if not self._mission_lock.acquire(blocking=False):
            self._publish_log("Önceki görev yükleme sürüyor, bekleyin", "warn")
            return False

        self._mission_active = True
        try:
            items = self._build_mission_items(waypoints)
            count = len(items)

            # 1) Eski görevi temizle (ack'i beklemeden kısa süre tanı).
            #    _mission henüz None olduğundan CLEAR_ALL ack'i yok sayılır.
            self._send_mission_clear_all()
            self._publish_log("Eski görev temizleniyor (MISSION_CLEAR_ALL)...", "info")
            time.sleep(0.4)

            # 2) Upload state machine'i kur ve MISSION_COUNT gönder
            self._mission = {
                "items":         items,
                "ack_type":      None,
                "done":          threading.Event(),
                "last_activity": time.time(),
            }
            self._publish_log(
                f"Görev yükleniyor: {len(waypoints)} waypoint (+home)...", "info"
            )
            self._send_mission_count(count)

            ok = self._await_mission(count)

            if ok:
                # 3) Görev indeksini başa al (AUTO ilk waypoint'ten başlasın)
                self._send_mission_set_current(0)
                self._publish_log(
                    f"Görev yüklendi ✓ ({len(waypoints)} wp) — AUTO'ya hazır", "ok"
                )
            else:
                at = self._mission.get("ack_type") if self._mission else None
                self._publish_log(
                    f"Görev yükleme BAŞARISIZ (ack={at}). AUTO'ya GEÇMEYİN!",
                    "error",
                )
            return ok
        except Exception as e:
            log.error(f"[{self.vehicle_id}] upload_mission hatası: {e}")
            self._publish_log(f"Görev yükleme hatası: {e}", "error")
            return False
        finally:
            self._mission        = None
            self._mission_active = False
            self._mission_lock.release()

    def send_waypoint(self, lat: float, lon: float, alt: float) -> bool:
        """
        Tek waypoint'i tam mission protokolüyle yükler (geriye uyumlu sarmalayıcı).
        Eski davranış (tek MISSION_ITEM_INT, seq=0, current=2) bir GUIDED 'şimdi
        buraya git' komutuydu ve AUTO görevini ETKİLEMİYORDU; artık gerçek bir
        görev yüklemesi yapılır.
        """
        return self.upload_mission([(lat, lon, alt)])

    def goto_guided(self, lat: float, lon: float, alt: float) -> None:
        """
        GUIDED modda 'şimdi buraya git' hedefi gönderir (mission upload DEĞİL).
        current=2, ArduPilot'ta anlık guided hedefi anlamına gelir.
        """
        if not self.conn:
            self._publish_log("GUIDED hedefi gönderilemedi: bağlantı yok", "warn")
            return
        with self._send_lock:
            self.conn.mav.mission_item_int_send(
                self.conn.target_system,
                self.conn.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 0,                       # current=2 → guided goto, autocontinue=0
                0, 0, 0, 0,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
            )
        self._publish_log(f"GUIDED hedef → {lat:.6f}, {lon:.6f}, {alt}m", "info")

    # ── Mission protokolü iç yardımcıları ──

    def _build_mission_items(self, waypoints, default_alt: float = 20.0):
        """
        waypoints → [(seq, frame, command, lat, lon, alt), ...]
        seq 0 = home placeholder (araç kendi home'uyla değiştirir).
        """
        items = []
        first_lat, first_lon, first_alt = waypoints[0]
        # seq 0: home placeholder (mutlak frame; değerler araçça override edilir)
        items.append((
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            first_lat, first_lon, first_alt or default_alt,
        ))
        # seq 1..N: gerçek waypoint'ler (home'a göre irtifa)
        for i, (lat, lon, alt) in enumerate(waypoints, start=1):
            items.append((
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                lat, lon, alt if alt else default_alt,
            ))
        return items

    def _handle_mission_msg(self, msg) -> None:
        """recv loop'tan gelen mission protokol mesajlarını işler."""
        up = self._mission
        if up is None:
            return
        t = msg.get_type()
        if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            self._mission_send_item(int(msg.seq))
        elif t == "MISSION_ACK":
            up["ack_type"] = int(msg.type)
            up["done"].set()

    def _mission_send_item(self, seq: int) -> None:
        """İstenen seq numaralı görev item'ını MISSION_ITEM_INT olarak yollar."""
        up = self._mission
        if up is None:
            return
        items = up["items"]
        if seq < 0 or seq >= len(items):
            log.warning(f"[{self.vehicle_id}] Geçersiz mission seq isteği: {seq}")
            return
        _seq, frame, command, lat, lon, alt = items[seq]
        up["last_activity"] = time.time()
        args = (
            self.conn.target_system,
            self.conn.target_component,
            seq, frame, command,
            0,                      # current (upload sırasında 0)
            1,                      # autocontinue
            0.0, 0.0, 0.0, 0.0,     # param1-4
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt),
        )
        with self._send_lock:
            try:
                self.conn.mav.mission_item_int_send(
                    *args, mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
            except TypeError:
                # Eski pymavlink: mission_type parametresi yok
                self.conn.mav.mission_item_int_send(*args)

    def _await_mission(self, count: int) -> bool:
        """
        MISSION_ACK gelene kadar bekler. Araç istekleri recv thread'de
        yanıtlanır; burada yalnızca sonuç beklenir ve takılma olursa
        MISSION_COUNT yeniden gönderilir.
        """
        up = self._mission
        overall      = max(15.0, count * 2.0)
        stall_limit  = 3.0
        max_retries  = 5
        retries      = 0
        deadline     = time.time() + overall

        while time.time() < deadline:
            if up["done"].wait(timeout=1.0):
                return up.get("ack_type") == mavutil.mavlink.MAV_MISSION_ACCEPTED
            # Yanıt yoksa (takılma) COUNT'u yeniden gönder
            if time.time() - up["last_activity"] > stall_limit:
                if retries >= max_retries:
                    break
                retries += 1
                self._publish_log(
                    f"Görev yükleme yanıtsız, tekrar deneniyor ({retries}/{max_retries})",
                    "warn",
                )
                self._send_mission_count(count)
                up["last_activity"] = time.time()
        return False

    def _send_mission_count(self, count: int) -> None:
        with self._send_lock:
            try:
                self.conn.mav.mission_count_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    count,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.conn.mav.mission_count_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    count,
                )

    def _send_mission_clear_all(self) -> None:
        with self._send_lock:
            try:
                self.conn.mav.mission_clear_all_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.conn.mav.mission_clear_all_send(
                    self.conn.target_system,
                    self.conn.target_component,
                )

    def _send_mission_set_current(self, seq: int) -> None:
        with self._send_lock:
            self.conn.mav.mission_set_current_send(
                self.conn.target_system,
                self.conn.target_component,
                int(seq),
            )

    def send_statustext(
        self,
        text: str,
        severity: int = mavutil.mavlink.MAV_SEVERITY_INFO,
    ) -> bool:
        """
        Araca STATUSTEXT mesajı gönderir (thread-safe).

        CommunicationBridge bu metodu kullanarak İHA'dan gelen "RENK:<kod>"
        bilgisini İDA'ya iletir. Gönderim _send_lock ile korunur; böylece
        recv_loop okurken başka bir thread'in yazması güvenlidir.

        Döner: True → gönderildi, False → bağlantı yok / hata.
        """
        if not self.conn:
            self._publish_log("STATUSTEXT gönderilemedi: bağlantı yok", "warn")
            return False
        # MAVLink STATUSTEXT alanı 50 bayt ile sınırlıdır
        payload = text.encode("utf-8")[:50]
        try:
            with self._send_lock:
                self.conn.mav.statustext_send(severity, payload)
            return True
        except Exception as e:
            log.error(f"[{self.vehicle_id}] STATUSTEXT gönderim hatası: {e}")
            self._publish_log(f"STATUSTEXT gönderim hatası: {e}", "error")
            return False

    # ──────────────────────────────────────
    #  INTERNAL
    # ──────────────────────────────────────

    def _connect_loop(self) -> None:
        """Bağlantı kurmaya çalış; kesilirse yeniden dene."""
        while self._running:
            self._set_state(ConnState.CONNECTING)
            self._publish_log(f"Bağlanıyor: {self.port} @ {self.baud}", "info")

            try:
                self.conn = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baud,
                    autoreconnect=False,
                    source_system=245,
                )
                self._publish_log("Heartbeat bekleniyor...", "dim")

                hb = self._wait_vehicle_heartbeat(self.CONNECT_TIMEOUT)
                if hb is None:
                    raise TimeoutError(f"Heartbeat gelmedi ({self.CONNECT_TIMEOUT}s)")

                self.conn.target_system = hb.get_srcSystem()
                self.conn.target_component = hb.get_srcComponent()
                self._last_heartbeat_ts = time.time()
                self._set_state(ConnState.CONNECTED)
                self._publish_log(
                    f"Bağlandı! Sistem: {self.conn.target_system}/{self.conn.target_component}", "ok"
                )

                self._request_data_streams()
                self._start_heartbeat_monitor()
                self._recv_loop()

            except Exception as e:
                log.error(f"[{self.vehicle_id}] Bağlantı hatası: {e}")
                self._publish_log(f"Bağlantı hatası: {e}", "error")
                self._close_conn()

            if not self._running:
                break

            self._set_state(ConnState.RECONNECTING)
            self._publish_log(
                f"{self.RECONNECT_DELAY}s sonra yeniden bağlanılacak...", "warn"
            )
            time.sleep(self.RECONNECT_DELAY)

    def _recv_loop(self) -> None:
        """Non-blocking mesaj okuma döngüsü."""
        while self._running and self.state == ConnState.CONNECTED:
            try:
                msg = self.conn.recv_match(
                    type=list(self.INTERESTING_MSGS),
                    blocking=True,
                    timeout=self.RECV_TIMEOUT,
                )
                if msg is None:
                    continue

                mtype = msg.get_type()
                if not self._is_from_target_system(msg):
                    continue

                if mtype == "HEARTBEAT":
                    if not self._is_vehicle_heartbeat(msg):
                        continue
                    self._last_heartbeat_ts = time.time()
                elif mtype in _MISSION_PROTOCOL_MSGS:
                    # Görev upload protokolü mesajları state machine'e gider,
                    # telemetri bus'ına yayınlanmaz.
                    self._handle_mission_msg(msg)
                    continue

                # Araç-spesifik event: "mavlink_msg_iha" / "mavlink_msg_ida"
                self.bus.publish(
                    f"mavlink_msg_{self.vehicle_id}",
                    {
                        "type":    mtype,
                        "msg":     msg,
                        "ts":      time.time(),
                        "vehicle": self.vehicle_id,
                    },
                )

            except Exception as e:
                log.error(f"[{self.vehicle_id}] recv_loop hatası: {e}")
                break

        self._set_state(ConnState.DISCONNECTED)
        self._publish_log("Bağlantı kesildi.", "warn")

    def _wait_vehicle_heartbeat(self, timeout: float):
        """Mission Planner/GCS heartbeat'lerini atlayıp gerçek aracı bekle."""
        deadline = time.time() + timeout
        while self._running and time.time() < deadline:
            remaining = max(0.1, deadline - time.time())
            msg = self.conn.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(self.RECV_TIMEOUT, remaining),
            )
            if msg is not None and self._is_vehicle_heartbeat(msg):
                return msg
        return None

    def _is_vehicle_heartbeat(self, msg) -> bool:
        if msg is None or msg.get_type() != "HEARTBEAT":
            return False

        mav_type = int(getattr(msg, "type", -1))
        autopilot = int(getattr(msg, "autopilot", -1))
        if mav_type == mavutil.mavlink.MAV_TYPE_GCS:
            return False
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return False
        return True

    def _is_from_target_system(self, msg) -> bool:
        target_system = getattr(self.conn, "target_system", 0)
        if not target_system:
            return True
        try:
            return int(msg.get_srcSystem()) == int(target_system)
        except Exception:
            return True

    def _start_heartbeat_monitor(self) -> None:
        self._hb_thread = threading.Thread(
            target=self._heartbeat_monitor,
            daemon=True,
            name=f"mav-{self.vehicle_id}-hb",
        )
        self._hb_thread.start()

    def _heartbeat_monitor(self) -> None:
        """Heartbeat timeout kontrolü."""
        while self._running and self.state == ConnState.CONNECTED:
            time.sleep(1.0)
            elapsed = time.time() - self._last_heartbeat_ts
            if elapsed > self.HEARTBEAT_TIMEOUT:
                log.warning(f"[{self.vehicle_id}] Heartbeat timeout ({elapsed:.1f}s)")
                self._publish_log(
                    f"Heartbeat zaman aşımı! ({elapsed:.1f}s)", "error"
                )
                self._close_conn()
                break

    def request_param(self, name: str) -> None:
        """Tek bir parametreyi araçtan iste (PARAM_REQUEST_READ)."""
        if not self.conn:
            return
        try:
            with self._send_lock:
                self.conn.mav.param_request_read_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    name.encode("ascii"),
                    -1,
                )
        except Exception as e:
            log.error(f"[{self.vehicle_id}] request_param({name}) hatası: {e}")

    def set_graph_streams(self, enabled: bool) -> None:
        """
        Kontrol grafikleri için ek telemetri stream'ini (thruster/servo PWM)
        aç/kapat. Pencere açılınca True, kapanınca False çağrılır.

        Bayrak kalıcıdır: yeniden bağlanmada _request_data_streams bunu
        dikkate alır. Araç bağlıysa stream isteği anında gönderilir.
        """
        self._graph_streams = bool(enabled)
        if not self.conn:
            return
        rate = 5 if enabled else 0
        start_stop = 1 if enabled else 0
        try:
            with self._send_lock:
                self.conn.mav.request_data_stream_send(
                    self.conn.target_system,
                    self.conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                    rate, start_stop,
                )
                self._send_graph_message_intervals_locked(enabled)
        except Exception as e:
            log.error(f"[{self.vehicle_id}] set_graph_streams hatası: {e}")

    def _send_graph_message_intervals_locked(self, enabled: bool) -> None:
        """Request live control-target messages. Caller must hold _send_lock."""
        interval = 200000 if enabled else -1   # 200000 us = 5Hz, -1 = stop
        msg_ids = (
            getattr(mavutil.mavlink, "MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT", 62),
            getattr(mavutil.mavlink, "MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT", 87),
            getattr(mavutil.mavlink, "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED", 85),
        )
        for msg_id in msg_ids:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id,
                interval, 0, 0, 0, 0, 0,
            )

    def _request_data_streams(self) -> None:
        """ArduPilot'tan belirli mesajları iste."""
        if not self.conn:
            return
        rates = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,         5),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,           2),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,      2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  2),
        ]
        # SERVO_OUTPUT_RAW (thruster PWM) yalnızca kontrol grafikleri
        # açıkken istenir — aksi halde gereksiz telemetri trafiği olmaz.
        if self._graph_streams:
            rates.append((mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 5))
        for stream_id, rate in rates:
            self.conn.mav.request_data_stream_send(
                self.conn.target_system,
                self.conn.target_component,
                stream_id, rate, 1,
            )
        if self._graph_streams:
            try:
                with self._send_lock:
                    self._send_graph_message_intervals_locked(True)
            except Exception as e:
                log.error(f"[{self.vehicle_id}] grafik mesaj aralığı hatası: {e}")
        log.info(f"[{self.vehicle_id}] Data stream istekleri gönderildi")

    def _send_command_long(self, command, param1=0, param2=0,
                           param3=0, param4=0, param5=0,
                           param6=0, param7=0) -> None:
        if not self.conn:
            self._publish_log("Komut gönderilemedi: bağlantı yok", "warn")
            return
        with self._send_lock:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command, 0,
                param1, param2, param3, param4, param5, param6, param7,
            )

    def _close_conn(self) -> None:
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None

    def _set_state(self, new_state: ConnState) -> None:
        self.state = new_state
        self.bus.publish(
            "conn_state",
            {"state": new_state, "vehicle": self.vehicle_id},
        )
        log.debug(f"[{self.vehicle_id}] Durum → {new_state.name}")

    def _publish_log(self, msg: str, level: str = "info") -> None:
        self.bus.publish(
            "log",
            {"msg": msg, "level": level, "vehicle": self.vehicle_id},
        )


# ─────────────────────────────────────────
#  ARAÇ YÖNETİCİSİ
# ─────────────────────────────────────────
class VehicleManager:
    """
    İHA ve İDA araçlarını tek noktadan yöneten sınıf.
    İleride yeni araç eklemek için vehicles dict'ini genişletmek yeterli.
    """

    def __init__(self, event_bus, vehicle_ids: tuple[str, ...] = ("iha", "ida")):
        self.bus = event_bus
        self.vehicles: dict[str, VehicleConnection] = {
            vid: VehicleConnection(event_bus, vid)
            for vid in vehicle_ids
        }

    def get(self, vehicle_id: str) -> Optional[VehicleConnection]:
        return self.vehicles.get(vehicle_id)

    def configure(self, vehicle_id: str, port: str, baud: int) -> None:
        v = self.get(vehicle_id)
        if v:
            v.configure(port, baud)

    def connect(self, vehicle_id: str) -> None:
        v = self.get(vehicle_id)
        if v:
            v.start()

    def disconnect(self, vehicle_id: str) -> None:
        v = self.get(vehicle_id)
        if v:
            v.stop()

    def stop_all(self) -> None:
        for v in self.vehicles.values():
            v.stop()

    # ── Araç komutları (aktif araca veya her ikisine) ──

    def arm(self, vehicle_id: str) -> None:
        self._dispatch("arm", vehicle_id)

    def disarm(self, vehicle_id: str, force: bool = False) -> None:
        if vehicle_id in self.vehicles:
            self.vehicles[vehicle_id].disarm(force=force)
        else:
            for v in self.vehicles.values():
                v.disarm(force=force)

    def set_mode(self, vehicle_id: str, mode: str) -> None:
        self._dispatch_with_arg("set_mode", vehicle_id, mode)

    def takeoff(self, vehicle_id: str, alt: float = 10.0) -> None:
        if vehicle_id in self.vehicles:
            self.vehicles[vehicle_id].takeoff(alt)
        else:
            for v in self.vehicles.values():
                v.takeoff(alt)

    def rtl(self, vehicle_id: str) -> None:
        self._dispatch("rtl", vehicle_id)

    def emergency_stop(self, vehicle_id: str) -> None:
        self._dispatch("emergency_stop", vehicle_id)

    def send_waypoint(self, vehicle_id: str, lat: float,
                      lon: float, alt: float) -> None:
        v = self.get(vehicle_id)
        if v:
            v.send_waypoint(lat, lon, alt)

    def upload_mission(self, vehicle_id: str, waypoints) -> bool:
        """Tüm waypoint listesini bir araca görev olarak yükler."""
        v = self.get(vehicle_id)
        if v:
            return v.upload_mission(waypoints)
        return False

    def goto_guided(self, vehicle_id: str, lat: float,
                    lon: float, alt: float) -> None:
        """GUIDED modda anlık 'buraya git' hedefi gönderir."""
        v = self.get(vehicle_id)
        if v:
            v.goto_guided(lat, lon, alt)

    def set_graph_streams(self, enabled: bool) -> None:
        """
        Tüm araçlarda kontrol-grafiği telemetri stream'ini aç/kapat.
        Kontrol Grafikleri penceresi açılınca/kapanınca çağrılır.
        """
        for v in self.vehicles.values():
            v.set_graph_streams(enabled)

    def request_param(self, vehicle_id: str, name: str) -> None:
        """Belirli bir araçtan tek parametre iste."""
        v = self.get(vehicle_id)
        if v:
            v.request_param(name)

    # ── Internal dispatch ──

    def _dispatch(self, method: str, vehicle_id: str) -> None:
        if vehicle_id in self.vehicles:
            getattr(self.vehicles[vehicle_id], method)()
        else:
            for v in self.vehicles.values():
                getattr(v, method)()

    def _dispatch_with_arg(self, method: str, vehicle_id: str, arg) -> None:
        if vehicle_id in self.vehicles:
            getattr(self.vehicles[vehicle_id], method)(arg)
        else:
            for v in self.vehicles.values():
                getattr(v, method)(arg)
