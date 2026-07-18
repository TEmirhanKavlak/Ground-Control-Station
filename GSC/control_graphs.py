"""
control_graphs.py
─────────────────
Gerçek zamanlı "Kontrol Grafikleri" penceresi (tk.Toplevel) — SADE & SAĞLAM.

Tasarım ilkeleri (donmayı önler):
  • ÖRNEKLEME ≠ MESAJ HIZI
      "telemetry" event'i saniyede 30-50+ kez gelir. _on_telemetry yalnızca
      en son payload'ı saklar (O(1)). Asıl nokta ekleme sabit hızda (_tick)
      yapılır → pencere başına ~300 nokta, deque ile sınırlı.
  • BLOKLAMAYAN ÇİZİM
      canvas.draw() (senkron, bloklayan) YERİNE canvas.draw_idle() kullanılır.
      Çizim yavaş olsa bile GUI thread'i kilitlenmez, istekler birikmez.
      flush_events() / blitting / NavigationToolbar gibi karmaşık ve sorun
      çıkarabilen yapılar KULLANILMAZ.
  • SADE GÖRÜNÜM
      3 paylaşımlı (sharex) eksen, kalıcı çizgi artist'leri (her karede
      ax.clear YOK, sadece set_data), alt eksende saat (HH:MM:SS) etiketi.

Veri kaynakları (TelemetryState):
  gerçek hız → ground_speed | gerçek yaw → heading (yoksa yaw)
  yaw isteği → nav_bearing (NAV_CONTROLLER_OUTPUT) ya da set_yaw_setpoint()
  hız isteği → set_speed_setpoint() (kontrol algoritması/görev)
  thruster   → servo_raw[kanal] (SERVO_OUTPUT_RAW), PWM olarak gösterilir
"""

import time
import logging
import datetime
from collections import deque
from typing import Optional, Callable

import tkinter as tk

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.dates as mdates
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from constants import (
    BG, PANEL_BG, PANEL_BDR, ACCENT, WARN, TEXT, TEXT_DIM, FONTS, IDA_COLOR,
)

log = logging.getLogger("control_graphs")

NAN = float("nan")


# ─────────────────────────────────────────
#  ZAMAN SERİSİ (deque)
# ─────────────────────────────────────────
class _TimeSeries:
    """(t, v) çiftleri. t = matplotlib datenum, v=None → NaN (boşluk)."""

    __slots__ = ("t", "v")

    def __init__(self, maxlen: int):
        self.t: deque = deque(maxlen=maxlen)
        self.v: deque = deque(maxlen=maxlen)

    def append(self, t: float, v: Optional[float]) -> None:
        self.t.append(t)
        self.v.append(float(v) if v is not None else NAN)

    def clear(self) -> None:
        self.t.clear()
        self.v.clear()

    def np(self):
        n = len(self.t)
        if n == 0:
            return np.empty(0), np.empty(0)
        return (np.fromiter(self.t, dtype=float, count=n),
                np.fromiter(self.v, dtype=float, count=n))


def _norm360(a: Optional[float]) -> Optional[float]:
    return None if a is None else float(a) % 360.0


def _angle_distance(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _wrap_segments_np(x: np.ndarray, y: np.ndarray):
    """0/360 sıçramalarındaki dikey çizgiyi kırmak için araya NaN ekler."""
    if x.size < 2:
        return x, y
    dy = np.abs(np.diff(y))
    brk = np.where(dy > 180.0)[0]
    if brk.size == 0:
        return x, y
    pos = brk + 1
    return np.insert(x, pos, np.nan), np.insert(y, pos, np.nan)


# ─────────────────────────────────────────
#  KONTROL GRAFİKLERİ PENCERESİ
# ─────────────────────────────────────────
class ControlGraphsWindow(tk.Toplevel):

    WINDOW_SECONDS     = 60      # gösterilen zaman penceresi (s)
    RENDER_INTERVAL_MS = 200     # örnekleme tick'i
    DRAW_EVERY         = 3       # her N tick'te bir ÇİZ (veri 200ms, render ~600ms)
    AUTOSCALE_EVERY    = 5       # hız y-eksenini her N çizimde bir ölçekle

    # Skid-steer Rover varsayılan thruster servo kanalları (1-indeksli)
    DEFAULT_LEFT_CH  = 1
    DEFAULT_RIGHT_CH = 3
    PWM_STOP = 1500

    # Otopilot hedeflerinin (yaw/hız isteği) anlamlı olduğu modlar.
    # Bu modlar dışında (MANUAL/HOLD/ACRO...) setpoint çizilmez.
    NAV_MODES = {
        "AUTO", "GUIDED", "RTL", "SMART_RTL", "LOITER",
        "FOLLOW", "DOCK", "CIRCLE", "AUTO_RTL",
    }
    PARAM_SPEED_MODES = {"AUTO", "AUTO_RTL"}
    SPEED_PARAMS = ("WP_SPEED", "CRUISE_SPEED", "WPNAV_SPEED")
    YAW_SP_STABLE_SAMPLES = 3
    YAW_SP_STABLE_EPS_DEG = 3.0
    SPEED_ERROR_EPS = 0.05

    def __init__(self, master, event_bus, get_active_vehicle: Callable[[], str], *,
                 telemetry_manager=None,
                 vehicle_manager=None,
                 window_seconds: int = WINDOW_SECONDS,
                 left_channel: int = DEFAULT_LEFT_CH,
                 right_channel: int = DEFAULT_RIGHT_CH,
                 on_close: Optional[Callable[[], None]] = None):
        super().__init__(master)

        self._bus           = event_bus
        self._get_active    = get_active_vehicle
        self._tmgr          = telemetry_manager
        self._vmgr          = vehicle_manager
        self.window_seconds = window_seconds
        self._left_idx      = max(0, left_channel - 1)
        self._right_idx     = max(0, right_channel - 1)
        self._on_close_cb   = on_close

        self._last_speed_sp = None    # son gecerlilik takibi; cizimde hold yapilmaz
        self._last_yaw_sp   = None    # son geçerli yaw isteği (hold)
        self._speed_sp: Optional[float] = None
        self._yaw_sp:   Optional[float] = None
        self._yaw_sp_candidate = None
        self._yaw_sp_candidate_count = 0

        self._latest_payload = None
        self._active_vid     = self._get_active()
        self._pending_reset  = False
        self._param_req_for: Optional[str] = None
        self._after_id: Optional[str] = None
        self._closing = False
        self._torn_down = False   # temizlik tek sefer çalışsın
        self._draw_ctr  = 0       # çizim seyreltme sayacı
        self._scale_ctr = 0       # autoscale seyreltme sayacı

        # Teşhis: setpoint kaynaklarının gelip gelmediğini bir kez raporla
        self._diag_done  = False
        self._diag_ticks = 0

        # Pencere
        self.title("📈 Kontrol Grafikleri")
        self.configure(bg=BG)
        self.geometry("760x620")
        self.minsize(560, 460)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        # Pencere HANGİ yolla kapanırsa kapansın (X, Alt+F4, ebeveyn destroy,
        # programatik) temizlik garanti çalışsın → arka planda iş kalmaz.
        self.bind("<Destroy>", self._on_destroy_event)

        # Zaman serileri (örnekleme hızına göre sınırlı)
        per_sec = max(1, int(1000 / self.RENDER_INTERVAL_MS))
        maxlen  = window_seconds * per_sec + 20
        self._s_speed_act = _TimeSeries(maxlen)
        self._s_speed_sp  = _TimeSeries(maxlen)
        self._s_speed_ref = _TimeSeries(maxlen)
        self._s_yaw_act   = _TimeSeries(maxlen)
        self._s_yaw_sp    = _TimeSeries(maxlen)
        self._s_thr_l     = _TimeSeries(maxlen)
        self._s_thr_r     = _TimeSeries(maxlen)

        self._win_days = window_seconds / 86400.0   # mdates birimi (gün)

        self._build_figure()

        # EventBus + (yalnızca açıkken) thruster stream'i
        self._bus.subscribe("telemetry", self._on_telemetry)
        # Setpoint'leri DOĞRUDAN kaynağından besleme kanalı (telemetri yerine):
        #   bus.publish("control_setpoint", {"speed": 2.5, "yaw": 90, "vehicle": "ida"})
        self._bus.subscribe("control_setpoint", self._on_control_setpoint)
        if self._vmgr is not None:
            try:
                self._vmgr.set_graph_streams(True)
            except Exception as e:
                log.error(f"set_graph_streams(True) hatası: {e}")

        self._after_id = self.after(self.RENDER_INTERVAL_MS, self._tick)
        log.info("ControlGraphsWindow açıldı (sade)")

    # ── Kontrol algoritması/görev entegrasyonu ──
    def set_speed_setpoint(self, value: Optional[float]) -> None:
        self._speed_sp = value

    def set_yaw_setpoint(self, value: Optional[float]) -> None:
        self._yaw_sp = value

    def set_thruster_channels(self, left_channel: int, right_channel: int) -> None:
        self._left_idx  = max(0, left_channel - 1)
        self._right_idx = max(0, right_channel - 1)

    def _on_control_setpoint(self, data: dict) -> None:
        """
        EventBus "control_setpoint" kanalı: setpoint'leri telemetri yerine
        DOĞRUDAN kaynağından (kontrol algoritması / komut gönderen kod) al.
        data: {"speed": m/s | None, "yaw": derece | None, "vehicle": opsiyonel}
        En yüksek öncelikli kaynaktır (telemetriyi geçersiz kılar).
        """
        if not isinstance(data, dict):
            return
        if self._closing:
            return
        vid = data.get("vehicle")
        if vid is not None and vid != self._get_active():
            return
        if "speed" in data:
            self._speed_sp = data["speed"]
        if "yaw" in data:
            self._yaw_sp = data["yaw"]

    # ──────────────────────────────────────
    #  FIGÜR (sade)
    # ──────────────────────────────────────
    def _build_figure(self) -> None:
        self._fig = Figure(figsize=(6.8, 5.8), dpi=90, facecolor=BG)
        self._fig.subplots_adjust(left=0.12, right=0.97, top=0.95,
                                  bottom=0.10, hspace=0.18)

        self._ax_speed = self._fig.add_subplot(3, 1, 1)
        self._ax_yaw   = self._fig.add_subplot(3, 1, 2, sharex=self._ax_speed)
        self._ax_thr   = self._fig.add_subplot(3, 1, 3, sharex=self._ax_speed)
        self._axes = (self._ax_speed, self._ax_yaw, self._ax_thr)

        for ax in self._axes:
            ax.set_facecolor(PANEL_BG)
            ax.grid(True, color=TEXT_DIM, alpha=0.2, lw=0.5)
            ax.tick_params(colors=TEXT_DIM, labelsize=7)
            for sp in ax.spines.values():
                sp.set_color(PANEL_BDR)

        # Etiketler yalnızca alt eksende (saat) → daha az çizim
        self._ax_speed.tick_params(labelbottom=False)
        self._ax_yaw.tick_params(labelbottom=False)
        self._ax_thr.xaxis_date()
        self._ax_thr.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

        # 1) Hız
        (self._ln_speed_act,) = self._ax_speed.plot([], [], color=ACCENT, lw=1.5,
                                                    label="Gerçek")
        (self._ln_speed_sp,) = self._ax_speed.plot([], [], color="red", lw=2.5, ls="--",
                                                    label="İstek")
        (self._ln_speed_ref,) = self._ax_speed.plot([], [], color=TEXT_DIM, lw=1.0,
                                                    ls=":", label="Param")
        self._ax_speed.set_ylabel("Hız (m/s)", color=TEXT, fontsize=8)
        self._ax_speed.set_ylim(0, 6)
        self._ax_speed.set_autoscaley_on(True)

        # 2) Yaw
        (self._ln_yaw_act,) = self._ax_yaw.plot([], [], color=ACCENT, lw=1.5,
                                               label="Gerçek")
        (self._ln_yaw_sp,)  = self._ax_yaw.plot([], [], color=WARN, lw=1.3,
                                               ls="-", drawstyle="steps-post",
                                               label="İstek")
        self._ax_yaw.set_ylabel("Yaw (°)", color=TEXT, fontsize=8)
        self._ax_yaw.set_ylim(0, 360)
        self._ax_yaw.set_yticks([0, 90, 180, 270, 360])

        # 3) Thruster (PWM)
        (self._ln_thr_l,) = self._ax_thr.plot([], [], color=ACCENT, lw=1.4,
                                             label="Sol")
        (self._ln_thr_r,) = self._ax_thr.plot([], [], color=IDA_COLOR, lw=1.4,
                                             label="Sağ")
        self._ax_thr.axhline(self.PWM_STOP, color=TEXT_DIM, lw=0.7, ls=":")
        self._ax_thr.set_ylabel("Thruster (PWM)", color=TEXT, fontsize=8)
        self._ax_thr.set_xlabel("Saat", color=TEXT, fontsize=8)
        self._ax_thr.set_ylim(1000, 2500)

        # Sade legend (çerçevesiz)
        for ax in self._axes:
            leg = ax.legend(loc="upper left", fontsize=7, frameon=False)
            if leg:
                for txt in leg.get_texts():
                    txt.set_color(TEXT_DIM)

        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().configure(bg=BG)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas.draw_idle()

    # ──────────────────────────────────────
    #  VERİ TOPLAMA — O(1)
    # ──────────────────────────────────────
    def _on_telemetry(self, data: dict) -> None:
        if self._closing:
            return
        active = self._get_active()
        if active != self._active_vid:
            self._active_vid = active
            self._pending_reset = True
            self._latest_payload = None
        if data.get("vehicle") != active:
            return
        self._latest_payload = data.get("payload")

    def _reset_series(self) -> None:
        for s in (self._s_speed_act, self._s_speed_sp, self._s_speed_ref, self._s_yaw_act,
                  self._s_yaw_sp, self._s_thr_l, self._s_thr_r):
            s.clear()
        self._last_speed_sp = None
        self._last_yaw_sp = None
        self._yaw_sp_candidate = None
        self._yaw_sp_candidate_count = 0

    # ──────────────────────────────────────
    #  TICK — örnekle + çiz (bloklamadan)
    # ──────────────────────────────────────
    def _tick(self) -> None:
        if self._closing or not self.winfo_exists():
            return
        try:
            if self._pending_reset:
                self._pending_reset = False
                self._reset_series()
                self._diag_done  = False
                self._diag_ticks = 0

            if self._vmgr is not None and self._param_req_for != self._active_vid:
                self._param_req_for = self._active_vid
                for pname in self.SPEED_PARAMS:
                    try:
                        self._vmgr.request_param(self._active_vid, pname)
                    except Exception:
                        pass

            self._maybe_diagnose()

            tnum = mdates.date2num(datetime.datetime.now())
            self._sample(tnum)
            self._refresh_lines()
            self._ax_speed.set_xlim(tnum - self._win_days, tnum)

            # Ağır olan canvas çizimini seyrelt → GUI thread'i tıkanmaz,
            # kapatma tıklaması her zaman işlenir (düşük güçlü makineler için).
            self._draw_ctr += 1
            if self._draw_ctr >= self.DRAW_EVERY:
                self._draw_ctr = 0
                # Hız y-eksenini sadece ara sıra ölçekle (ek yük + titreşim azalır)
                self._scale_ctr += 1
                if self._scale_ctr >= self.AUTOSCALE_EVERY:
                    self._scale_ctr = 0
                    self._ax_speed.relim()
                    self._ax_speed.autoscale_view(scalex=False, scaley=True)
                self._canvas.draw_idle()   # BLOKLAMAZ
        except Exception as e:
            log.error(f"tick hatası: {e}")
        finally:
            if not self._closing and self.winfo_exists():
                self._after_id = self.after(self.RENDER_INTERVAL_MS, self._tick)

    def _maybe_diagnose(self) -> None:
        """
        Açılıştan ~3 sn sonra, setpoint kaynaklarının gelip gelmediğini
        bir kez log'a yazar. Çizgiler boşsa nedenini görmeni sağlar.
        """
        if self._diag_done:
            return
        self._diag_ticks += 1
        if self._diag_ticks < max(1, int(3000 / self.RENDER_INTERVAL_MS)):
            return
        self._diag_done = True

        st = self._latest_payload
        if st is None:
            self._log("[Grafik] Aktif araçtan telemetri gelmiyor (bağlı mı?).", "warn")
            return

        mode = getattr(st, "mode", "?") or "?"
        has_nav = bool(getattr(st, "has_nav_output", False))
        live_speed, live_speed_src = self._live_speed_setpoint(
            st, mode, mode in self.NAV_MODES
        )
        param_speed = self._param_speed(st) if mode in self.PARAM_SPEED_MODES else None
        has_ts = bool(getattr(st, "has_target_speed", False))
        has_par = param_speed is not None
        pushed  = (self._speed_sp is not None) or (self._yaw_sp is not None)

        self._log(
            f"[Grafik] {self._active_vid}/{mode} → yaw isteği: "
            f"{'nav_bearing var' if has_nav else 'YOK'} | hız isteği: "
            f"{live_speed_src if live_speed is not None else ('sabit parametre' if has_par else 'YOK')}"
            f"{' | push aktif' if pushed else ''}",
            "info",
        )
        if live_speed is None and has_par:
            self._log(
                f"[Grafik] Canlı hız hedefi gelmiyor; kırmızı çizgi sabit hız "
                f"parametresinden çiziliyor ({param_speed:.2f} m/s).",
                "warn",
            )
        if not pushed and not has_nav and not has_ts and not has_par:
            self._log(
                "[Grafik] İstenen yaw/hız telemetride GELMİYOR. Araç AUTO/GUIDED "
                "değilse bu normaldir. Setpoint'i kaynağından besleyin: "
                "bus.publish('control_setpoint', {'speed': v, 'yaw': deg}).",
                "warn",
            )

    def _log(self, msg: str, level: str = "info") -> None:
        try:
            self._bus.publish("log", {"msg": msg, "level": level,
                                      "vehicle": self._active_vid})
        except Exception:
            pass

    def _sample(self, tnum: float) -> None:
        st = self._latest_payload
        if st is None:
            return

        mode = getattr(st, "mode", "") or ""
        in_nav = mode in self.NAV_MODES

        # ── Gerçek hız ──
        self._s_speed_act.append(tnum, getattr(st, "ground_speed", None))

        # ── Hız isteği: canlı setpoint, yoksa AUTO sabit hız parametresi ──
        ssp, source = self._display_speed_setpoint(st, mode, in_nav)
        if ssp is not None:
            self._last_speed_sp = ssp
        self._s_speed_sp.append(tnum, ssp)
        param_ref = self._param_speed(st) if mode in self.PARAM_SPEED_MODES else None
        self._s_speed_ref.append(tnum, param_ref if source != "parametre" else None)

        # ── Gerçek yaw (0=kuzey falsy sorununa karşı açık None kontrolü) ──

        yaw = getattr(st, "heading", None)
        if yaw is None:
            yaw = getattr(st, "yaw", None)
        self._s_yaw_act.append(tnum, _norm360(yaw))

        # ── Yaw isteği: push > stabil target_bearing; yeni değer yoksa son değeri tut.
        if self._yaw_sp is not None:
            ysp = _norm360(self._yaw_sp)
            self._last_yaw_sp = ysp
            self._yaw_sp_candidate = None
            self._yaw_sp_candidate_count = 0
        elif in_nav and getattr(st, "has_nav_output", False):
            ysp = self._stable_nav_yaw_sp(getattr(st, "target_bearing", None))
        else:
            ysp = None

        self._s_yaw_sp.append(tnum, ysp if ysp is not None else self._last_yaw_sp)

        # ── Thruster (PWM) ──
        servo = getattr(st, "servo_raw", None) or []
        lp = servo[self._left_idx]  if len(servo) > self._left_idx  else None
        rp = servo[self._right_idx] if len(servo) > self._right_idx else None
        self._s_thr_l.append(tnum, lp or None)
        self._s_thr_r.append(tnum, rp or None)

    def _param_speed(self, st) -> Optional[float]:
        p = getattr(st, "params", None) or {}
        wp = p.get("WP_SPEED")
        if wp is not None and wp > 0:
            return wp
        if "CRUISE_SPEED" in p:
            return p["CRUISE_SPEED"]
        if "WPNAV_SPEED" in p:
            return p["WPNAV_SPEED"] / 100.0
        return None

    def _live_speed_setpoint(self, st, mode: str, in_nav: bool):
        """Return only live speed setpoints; static AUTO params are not live."""
        if self._speed_sp is not None:
            return self._speed_sp, "push"
        if getattr(st, "has_target_speed", False):
            return getattr(st, "target_speed", None), "POSITION_TARGET"
        if in_nav and getattr(st, "has_nav_output", False):
            speed_err = float(getattr(st, "target_speed_error", 0.0) or 0.0)
            if abs(speed_err) > self.SPEED_ERROR_EPS:
                speed = float(getattr(st, "ground_speed", 0.0) or 0.0) + speed_err
                return max(0.0, speed), "NAV hız hatası"
        return None, None

    def _display_speed_setpoint(self, st, mode: str, in_nav: bool):
        live, source = self._live_speed_setpoint(st, mode, in_nav)
        if live is not None:
            return live, source
        if mode in self.PARAM_SPEED_MODES:
            param = self._param_speed(st)
            if param is not None:
                return param, "parametre"
        return None, None

    def _stable_nav_yaw_sp(self, raw_yaw: Optional[float]) -> Optional[float]:
        ysp = _norm360(raw_yaw)
        if ysp is None:
            return self._last_yaw_sp

        if self._last_yaw_sp is None:
            self._last_yaw_sp = ysp
            self._yaw_sp_candidate = None
            self._yaw_sp_candidate_count = 0
            return self._last_yaw_sp

        if _angle_distance(ysp, self._last_yaw_sp) <= self.YAW_SP_STABLE_EPS_DEG:
            self._yaw_sp_candidate = None
            self._yaw_sp_candidate_count = 0
            return self._last_yaw_sp

        if (self._yaw_sp_candidate is None or
                _angle_distance(ysp, self._yaw_sp_candidate) > self.YAW_SP_STABLE_EPS_DEG):
            self._yaw_sp_candidate = ysp
            self._yaw_sp_candidate_count = 1
        else:
            self._yaw_sp_candidate = ysp
            self._yaw_sp_candidate_count += 1

        if self._yaw_sp_candidate_count >= self.YAW_SP_STABLE_SAMPLES:
            self._last_yaw_sp = self._yaw_sp_candidate
            self._yaw_sp_candidate = None
            self._yaw_sp_candidate_count = 0

        return self._last_yaw_sp

    def _refresh_lines(self) -> None:
        self._ln_speed_act.set_data(*self._s_speed_act.np())
        self._ln_speed_sp.set_data(*self._s_speed_sp.np())
        self._ln_speed_ref.set_data(*self._s_speed_ref.np())
        self._ln_yaw_act.set_data(*_wrap_segments_np(*self._s_yaw_act.np()))
        # Setpoint zaten örnekleme aşamasında kararlı hale getiriliyor. Burada
        # NaN ile sıçramaları kırmak hedef çizgisinde gereksiz boşluklar
        # oluşturuyordu. Basamak çizimi değişimleri kesintisiz gösterir.
        self._ln_yaw_sp.set_data(*self._s_yaw_sp.np())
        self._ln_thr_l.set_data(*self._s_thr_l.np())
        self._ln_thr_r.set_data(*self._s_thr_r.np())

    # Geriye dönük API — ASLA _tick() çağırıp paralel after-zinciri DOĞURMAZ.
    def update_graphs(self, reschedule: bool = True) -> None:
        if not self._closing and self.winfo_exists():
            try:
                self._canvas.draw_idle()
            except Exception:
                pass

    # ──────────────────────────────────────
    #  TEMİZ KAPANIŞ (her yolu yakalar, tek sefer çalışır)
    # ──────────────────────────────────────
    def _cleanup(self) -> None:
        """Timer + abonelik + stream temizliği. İdempotent; destroy ETMEZ."""
        if self._torn_down:
            return
        self._torn_down = True
        self._closing   = True

        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        try:
            self._bus.unsubscribe("telemetry", self._on_telemetry)
        except Exception:
            pass
        try:
            self._bus.unsubscribe("control_setpoint", self._on_control_setpoint)
        except Exception:
            pass
        if self._vmgr is not None:
            try:
                self._vmgr.set_graph_streams(False)
            except Exception:
                pass
        try:
            import matplotlib.pyplot as plt
            plt.close(self._fig)
        except Exception:
            pass
        if callable(self._on_close_cb):
            try:
                self._on_close_cb()
            except Exception:
                pass
        log.info("ControlGraphsWindow temizlendi")

    def _on_destroy_event(self, event) -> None:
        # Pencere herhangi bir yolla yok edilirken temizliği garanti et
        if event.widget is self:
            self._cleanup()

    def _on_close(self) -> None:
        self._cleanup()
        try:
            self.destroy()
        except Exception:
            pass
