"""
gui.py
──────
YKİ Ana GUI — Çift Araç Destekli Modüler Bileşen Mimarisi.

Bileşenler:
    VehicleSelector    — İHA / İDA segmented control
    ConnectionPanel    — COM port, baudrate, connect/disconnect (her araç ayrı)
    HeaderPanel        — Başlık + araç seçici + bağlantı paneli + saat
    LogPanel           — Çift log (İHA üstte, İDA altta, ayrı scroll/clear)
    HudCanvas          — Tek araç HUD canvas (yeniden kullanılabilir)
    DualHudPanel       — İki HUD yan yana
    TelemetryMiniPanel — Aktif araca ait metrik kartları
    ControlPanel       — ARM/DISARM/mod/takeoff/RTL/acil
    WaypointListPanel  — Waypoint listesi + gönder
    GCSApplication     — Ana koordinatör

v4: İHA + İDA bağımsız telemetry, log, HUD, bağlantı.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import datetime
import logging
import threading
from typing import Optional

from mavlink_manager import (
    ConnState, VehicleManager, list_serial_ports
)
from vehicle_modes import mode_names_for_vehicle
from map_controller import MapController
from control_graphs import ControlGraphsWindow
from constants import (
    BG, PANEL_BG, PANEL_BDR, ACCENT, ACCENT2, OK, WARN,
    TEXT, TEXT_DIM, COORD_BG, BTN_H,
    IHA_COLOR, IDA_COLOR,
    FONTS, VEHICLE_LABELS, VEHICLE_IDS,
    DEFAULT_BAUD, BAUD_OPTIONS,
    WINDOW_TITLE, WINDOW_GEOMETRY, WINDOW_MIN, APP_VERSION,
    POLL_INTERVAL_MS, lighten,
)

log = logging.getLogger("gui")


# ═══════════════════════════════════════════════════════════════
#  YARDIMCI — ortak widget fabrikası
# ═══════════════════════════════════════════════════════════════

def make_panel(parent, title: str):
    """Başlıklı, kenarlıklı panel. (outer, inner) döndürür."""
    outer = tk.Frame(parent, bg=PANEL_BDR, padx=1, pady=1)
    inner = tk.Frame(outer, bg=PANEL_BG)
    inner.pack(fill="both", expand=True)
    inner.rowconfigure(2, weight=1)
    inner.columnconfigure(0, weight=1)

    tk.Label(
        inner, text=title, bg=PANEL_BG, fg=ACCENT,
        font=FONTS["panel"], anchor="w", padx=8, pady=4,
    ).grid(row=0, column=0, sticky="ew")
    tk.Frame(inner, bg=PANEL_BDR, height=1).grid(row=1, column=0, sticky="ew")

    return outer, inner


def make_btn(parent, text: str, color: str,
             command=None, width: int = None) -> tk.Button:
    """Cyber-theme düğme."""
    kwargs = dict(
        text=text, bg=color, fg=BG,
        activebackground=color, activeforeground=BG,
        font=FONTS["mono"], relief="flat", bd=0,
        padx=10, pady=7, cursor="hand2", command=command,
    )
    if width:
        kwargs["width"] = width
    btn = tk.Button(parent, **kwargs)
    btn.bind("<Enter>", lambda e: btn.configure(bg=lighten(color)))
    btn.bind("<Leave>", lambda e: btn.configure(bg=color))
    return btn


# ═══════════════════════════════════════════════════════════════
#  ARAÇ SEÇİCİ (Segmented Control)
# ═══════════════════════════════════════════════════════════════

class VehicleSelector(tk.Frame):
    """
    İHA / İDA segmented control.
    Seçim değişince on_change(vehicle_id) çağrılır.
    """

    def __init__(self, parent, on_change=None, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._on_change    = on_change
        self._current      = "iha"
        self._buttons: dict[str, tk.Button] = {}
        self._build()

    def _build(self):
        tk.Label(
            self, text="AKTİF ARAÇ:", bg=BG, fg=TEXT_DIM,
            font=FONTS["label"],
        ).pack(side="left", padx=(0, 6))

        container = tk.Frame(self, bg=PANEL_BDR, padx=1, pady=1)
        container.pack(side="left")
        inner = tk.Frame(container, bg=PANEL_BG)
        inner.pack()

        for i, vid in enumerate(VEHICLE_IDS):
            label = VEHICLE_LABELS[vid]

            btn = tk.Button(
                inner,
                text=label,
                font=FONTS["seg"],
                relief="flat", bd=0,
                padx=16, pady=6,
                cursor="hand2",
                command=lambda v=vid: self._select(v),
            )
            btn.grid(row=0, column=i * 2, padx=1, pady=1)
            self._buttons[vid] = btn

            if i < len(VEHICLE_IDS) - 1:
                tk.Frame(inner, bg=PANEL_BDR, width=1).grid(
                    row=0, column=i * 2 + 1, sticky="ns"
                )

        self._refresh_styles()

    def _select(self, vehicle_id: str):
        if vehicle_id == self._current:
            return
        self._current = vehicle_id
        self._refresh_styles()
        if self._on_change:
            self._on_change(vehicle_id)

    def _refresh_styles(self):
        for vid, btn in self._buttons.items():
            active = vid == self._current
            color  = IHA_COLOR if vid == "iha" else IDA_COLOR
            if active:
                btn.configure(bg=color, fg=BG, activebackground=lighten(color))
            else:
                btn.configure(bg=BTN_H, fg=TEXT_DIM,
                              activebackground=BTN_H, activeforeground=TEXT_DIM)

    @property
    def current(self) -> str:
        return self._current


# ═══════════════════════════════════════════════════════════════
#  BAĞLANTI PANELİ
# ═══════════════════════════════════════════════════════════════

class ConnectionPanel(tk.Frame):
    """
    Sağ üstte yer alan çift araç bağlantı paneli.
    Her araç için: COM seçici + baudrate + connect/disconnect + durum.
    """

    def __init__(self, parent, vehicle_manager: VehicleManager,
                 on_log=None, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._vmgr   = vehicle_manager
        self._on_log = on_log

        self._port_vars:   dict[str, tk.StringVar] = {}
        self._baud_vars:   dict[str, tk.StringVar] = {}
        self._port_combos: dict[str, ttk.Combobox] = {}
        self._conn_dots:   dict[str, tk.Label]     = {}
        self._conn_lbls:   dict[str, tk.Label]     = {}

        self._build()

    def _build(self):
        container = tk.Frame(self, bg=PANEL_BDR, padx=1, pady=1)
        container.pack(fill="both")
        inner = tk.Frame(container, bg=PANEL_BG, padx=8, pady=6)
        inner.pack(fill="both")

        tk.Label(
            inner, text="◎ BAĞLANTI", bg=PANEL_BG, fg=ACCENT,
            font=FONTS["panel"],
        ).grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 4))
        tk.Frame(inner, bg=PANEL_BDR, height=1).grid(
            row=1, column=0, columnspan=5, sticky="ew", pady=(0, 6)
        )

        # İHA ve İDA'yı aynı satıra koy
        for col_i, vid in enumerate(VEHICLE_IDS):
            vehicle_frame = tk.Frame(inner, bg=PANEL_BG)
            vehicle_frame.grid(
                row=2,
                column=col_i,
                padx=8,
                pady=2,
                sticky="n"
            )

            self._build_vehicle_row(vehicle_frame, vid, 0)


    def _build_vehicle_row(self, parent, vid: str, row: int):
        color = IHA_COLOR if vid == "iha" else IDA_COLOR
        label = VEHICLE_LABELS[vid]

        # Araç etiketi
        tk.Label(
            parent, text=f"{label}:", bg=PANEL_BG, fg=color,
            font=FONTS["mono_sm"], width=4, anchor="w",
        ).grid(row=row, column=0, padx=(0, 4), pady=2)

        # COM / TCP seçici
        port_var = tk.StringVar(value="tcp:127.0.0.1:5762")
        self._port_vars[vid] = port_var

        SIM_PORTS = [
            "tcp:127.0.0.1:5762",   # Mission Planner simulation
        ]

        ports = SIM_PORTS + (list_serial_ports() or ["---"])

        port_cb = ttk.Combobox(
            parent,
            textvariable=port_var,
            values=ports,
            width=18,
            font=FONTS["mono_sm"],
            state="readonly",
            postcommand=lambda v=vid: self._refresh_ports(v),
        )
        port_cb.grid(row=row, column=1, padx=2, pady=2)
        self._port_combos[vid] = port_cb

        # Baud seçici
        baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self._baud_vars[vid] = baud_var
        baud_cb = ttk.Combobox(
            parent, textvariable=baud_var,
            values=[str(b) for b in BAUD_OPTIONS],
            width=7, font=FONTS["mono_sm"], state="readonly",
        )
        baud_cb.grid(row=row, column=2, padx=2, pady=2)

        # Buton grubu
        btn_frame = tk.Frame(parent, bg=PANEL_BG)
        btn_frame.grid(row=row, column=3, padx=(4, 0), pady=2)

        conn_btn = tk.Button(
            btn_frame, text="BAĞLAN", bg=OK, fg=BG,
            font=FONTS["mono_sm"], relief="flat", padx=7, pady=3,
            cursor="hand2",
            command=lambda v=vid: self._connect(v),
        )
        conn_btn.pack(side="left", padx=(0, 2))

        disc_btn = tk.Button(
            btn_frame, text="KES", bg=ACCENT2, fg=BG,
            font=FONTS["mono_sm"], relief="flat", padx=7, pady=3,
            cursor="hand2",
            command=lambda v=vid: self._disconnect(v),
        )
        disc_btn.pack(side="left", padx=(0, 6))

        # Durum göstergesi
        dot = tk.Label(btn_frame, text="●", fg=TEXT_DIM, bg=PANEL_BG,
                       font=("Courier New", 10))
        dot.pack(side="left")
        lbl = tk.Label(btn_frame, text="YOK", fg=TEXT_DIM, bg=PANEL_BG,
                       font=FONTS["mono_sm"])
        lbl.pack(side="left", padx=2)

        self._conn_dots[vid] = dot
        self._conn_lbls[vid] = lbl
    def _refresh_ports(self, vid):
        """Dropdown her açıldığında seri portları yeniden tara."""
        sim = ["tcp:127.0.0.1:5762"]
        ports = sim + (list_serial_ports() or ["---"])
        self._port_combos[vid].configure(values=ports)
    def _connect(self, vid: str):
        port    = self._port_vars[vid].get()
        baud_str = self._baud_vars[vid].get()
        if port in ("---", ""):
            if self._on_log:
                self._on_log(f"{VEHICLE_LABELS[vid]}: COM port seçin!", "warn", vid)
            return
        baud = int(baud_str) if baud_str.isdigit() else DEFAULT_BAUD
        self._vmgr.configure(vid, port, baud)
        self._vmgr.connect(vid)
        if self._on_log:
            self._on_log(
                f"{VEHICLE_LABELS[vid]} bağlanıyor: {port} @ {baud}", "info", vid
            )

    def _disconnect(self, vid: str):
        self._vmgr.disconnect(vid)
        if self._on_log:
            self._on_log(f"{VEHICLE_LABELS[vid]} bağlantısı kesildi", "warn", vid)

    def update_conn_state(self, vid: str, state: ConnState):
        """Bağlantı durumu değişince GUI'yi güncelle (GUI thread)."""
        mapping = {
            ConnState.DISCONNECTED: (TEXT_DIM, "YOK"),
            ConnState.CONNECTING:   (WARN,     "BAĞL..."),
            ConnState.CONNECTED:    (OK,        "BAĞLI"),
            ConnState.RECONNECTING: (WARN,      "TEKRAR"),
        }
        color, text = mapping.get(state, (TEXT_DIM, "?"))
        if dot := self._conn_dots.get(vid):
            dot.configure(fg=color)
        if lbl := self._conn_lbls.get(vid):
            lbl.configure(text=text, fg=color)


# ═══════════════════════════════════════════════════════════════
#  HEADER PANELİ
# ═══════════════════════════════════════════════════════════════

class HeaderPanel(tk.Frame):
    """
    Üst başlık:
      Sol  → Logo + başlık
      Orta → Araç seçici
      Sağ  → Saat + bağlantı paneli
    """

    def __init__(self, parent, vehicle_manager: VehicleManager,
                 on_vehicle_change=None, on_log=None, **kwargs):
        super().__init__(parent, bg=BG, pady=10, **kwargs)
        self._on_vehicle_change = on_vehicle_change
        self.conn_panel: Optional[ConnectionPanel]   = None
        self.vehicle_selector: Optional[VehicleSelector] = None
        self._build(vehicle_manager, on_log)

    def _build(self, vehicle_manager: VehicleManager, on_log):
        # Sol: başlık
        tk.Label(
            self,
            text="◈ ATÜ YGM KAAN ERTUĞRUL TAKIMI — YKİ",
            bg=BG, fg=ACCENT, font=FONTS["title"], anchor="w",
        ).pack(side="left")

        # Sağ: saat
        self._clock_lbl = tk.Label(self, bg=BG, fg=TEXT_DIM, font=FONTS["mono"])
        self._clock_lbl.pack(side="right", padx=(12, 0))
        self._tick_clock()

        # Sağ: bağlantı paneli
        self.conn_panel = ConnectionPanel(
            self, vehicle_manager=vehicle_manager, on_log=on_log
        )
        self.conn_panel.pack(side="right", padx=(12, 12))

        # Orta: araç seçici
        self.vehicle_selector = VehicleSelector(
            self, on_change=self._on_vehicle_change
        )
        self.vehicle_selector.pack(side="left", padx=(32, 0))

    def _tick_clock(self):
        self._clock_lbl.configure(
            text=datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
        )
        self.after(1000, self._tick_clock)

    @property
    def active_vehicle(self) -> str:
        return self.vehicle_selector.current if self.vehicle_selector else "iha"

    def update_conn_state(self, vid: str, state: ConnState):
        if self.conn_panel:
            self.conn_panel.update_conn_state(vid, state)


# ═══════════════════════════════════════════════════════════════
#  LOG PANELİ (çift log)
# ═══════════════════════════════════════════════════════════════

class LogPanel(tk.Frame):
    """
    Dikey olarak ikiye bölünmüş log paneli.
    Üst: İHA logları  |  Alt: İDA logları
    Ayrı scroll, ayrı clear, ayrı renk.
    """

    LOG_TAG_COLORS = {
        "info":  TEXT,
        "ok":    OK,
        "warn":  WARN,
        "error": ACCENT2,
        "dim":   TEXT_DIM,
        "ts":    TEXT_DIM,
    }

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._log_boxes: dict[str, tk.Text] = {}
        self._build()

    def _build(self):
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        for row_i, vid in enumerate(VEHICLE_IDS):
            self._build_log_section(vid, row_i)

    def _build_log_section(self, vid: str, row: int):
        color = IHA_COLOR if vid == "iha" else IDA_COLOR
        label = VEHICLE_LABELS[vid]

        outer = tk.Frame(self, bg=PANEL_BDR, padx=1, pady=1)
        outer.grid(row=row, column=0, sticky="nsew",
                   pady=(0, 3) if row == 0 else (3, 0))

        inner = tk.Frame(outer, bg=PANEL_BG)
        inner.pack(fill="both", expand=True)
        inner.rowconfigure(2, weight=1)
        inner.columnconfigure(0, weight=1)

        # Başlık
        hdr = tk.Frame(inner, bg=PANEL_BG)
        hdr.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 2))

        tk.Label(hdr, text=f"◎ LOG — {label}", bg=PANEL_BG, fg=color,
                 font=FONTS["panel"]).pack(side="left")

        tk.Button(
            hdr, text="TEMİZLE",
            bg=BTN_H, fg=TEXT_DIM,
            font=FONTS["mono_sm"], relief="flat", padx=6, pady=2,
            cursor="hand2",
            command=lambda v=vid: self._clear(v),
        ).pack(side="right")

        tk.Frame(inner, bg=PANEL_BDR, height=1).grid(
            row=1, column=0, columnspan=2, sticky="ew"
        )

        # Log text
        log_box = tk.Text(
            inner, bg=PANEL_BG, fg=TEXT,
            font=FONTS["mono_sm"], relief="flat", bd=0,
            wrap="word", state="disabled",
            selectbackground=BTN_H,
            highlightthickness=0,
        )
        log_box.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)

        for tag, clr in self.LOG_TAG_COLORS.items():
            log_box.tag_config(tag, foreground=clr)
        log_box.tag_config("vehicle", foreground=color, font=FONTS["mono_sm"])

        sb = tk.Scrollbar(inner, command=log_box.yview, bg=PANEL_BG,
                          troughcolor=PANEL_BG)
        sb.grid(row=2, column=1, sticky="ns")
        log_box.configure(yscrollcommand=sb.set)

        self._log_boxes[vid] = log_box

    def append(self, vid: str, msg: str, level: str = "info"):
        """Belirtilen araç log kutusuna mesaj ekle."""
        # Bilinmeyen vehicle_id → ilk araca düş
        if vid not in self._log_boxes:
            vid = VEHICLE_IDS[0]
        box = self._log_boxes[vid]
        ts  = datetime.datetime.now().strftime("%H:%M:%S")
        lbl = VEHICLE_LABELS.get(vid, vid.upper())

        box.configure(state="normal")
        box.insert("end", f"[{ts}]", "ts")
        box.insert("end", f"[{lbl}]", "vehicle")
        box.insert("end", f"[{level.upper()}] ", level)
        box.insert("end", msg + "\n", level)
        box.see("end")
        box.configure(state="disabled")

    def _clear(self, vid: str):
        box = self._log_boxes.get(vid)
        if box:
            box.configure(state="normal")
            box.delete("1.0", "end")
            box.configure(state="disabled")


# ═══════════════════════════════════════════════════════════════
#  HUD CANVAS (tek araç, yeniden kullanılabilir)
# ═══════════════════════════════════════════════════════════════

class HudCanvas(tk.Frame):
    """
    Tek araç yapay ufuk HUD canvas.
    Pitch/Roll + Altitude bandı + Speed bandı + Compass şeridi.
    """

    def __init__(self, parent, vehicle_id: str, **kwargs):
        super().__init__(parent, bg="#05070d", **kwargs)
        self.vehicle_id = vehicle_id
        color = IHA_COLOR if vehicle_id == "iha" else IDA_COLOR
        label = VEHICLE_LABELS[vehicle_id]

        tk.Label(
            self,
            text=f"◎ {label} — FLIGHT HUD",
            bg="#05070d", fg=color, font=FONTS["panel"],
        ).pack(anchor="w", padx=10, pady=(8, 2))

        self._canvas = tk.Canvas(self, bg="#05070d", highlightthickness=0)
        self._canvas.pack(fill="both", expand=True, padx=4, pady=(0, 4))

        self._cur = {"pitch": 0.0, "roll": 0.0, "heading": 0.0,
                     "alt": 0.0, "spd": 0.0, "vspd": 0.0}
        self._tgt = dict(self._cur)
        self._color     = color
        self._animating = False

        self._canvas.bind(
            "<Configure>",
            lambda e: self._draw(self._cur["pitch"], self._cur["roll"])
        )

    def update(self, state) -> None:
        """TelemetryState → HUD güncelle (smooth animasyon)."""
        self._tgt["pitch"]   = state.pitch
        self._tgt["roll"]    = state.roll
        self._tgt["heading"] = state.heading
        self._tgt["alt"]     = state.alt_rel
        self._tgt["spd"]     = state.ground_speed
        self._tgt["vspd"]    = state.vspeed
        if not self._animating:
            self._animating = True
            self._animate()

    def _animate(self):
        alpha   = 0.18
        changed = False
        for k in self._cur:
            diff = self._tgt[k] - self._cur[k]
            if abs(diff) > 0.01:
                self._cur[k] += diff * alpha
                changed = True
        self._draw(self._cur["pitch"], self._cur["roll"])
        if changed:
            self._canvas.after(30, self._animate)
        else:
            self._animating = False

    def _draw(self, pitch: float, roll: float):
        import math
        c = self._canvas
        W = c.winfo_width()
        H = c.winfo_height()
        if W < 20 or H < 20:
            return

        c.delete("all")

        ACCENT_C = self._color
        OK_C     = "#00c896"
        WARN_C   = "#f6ad55"
        ERR_C    = "#db2d61"
        TEXT_C   = "#dde3f0"
        DIM_C    = "#3d4a60"
        BG_C     = "#05070d"
        SKY_C    = "#1a3a5c"
        GND_C    = "#3d2c1e"

        cx, cy = W // 2, H // 2
        R      = math.radians(roll)
        cos_r, sin_r = math.cos(R), math.sin(R)
        pitch_px     = pitch * 4.5
        horizon_half = max(W, H) * 1.5

        def rot(x, y):
            return (cx + x * cos_r - y * sin_r,
                    cy + x * sin_r + y * cos_r)

        # Gökyüzü
        sky_pts = [
            rot(-horizon_half,  pitch_px),
            rot( horizon_half,  pitch_px),
            rot( horizon_half, -horizon_half),
            rot(-horizon_half, -horizon_half),
        ]
        c.create_polygon(*[v for pt in sky_pts for v in pt],
                         fill=SKY_C, outline="")

        # Zemin
        gnd_pts = [
            rot(-horizon_half, pitch_px),
            rot( horizon_half, pitch_px),
            rot( horizon_half, horizon_half),
            rot(-horizon_half, horizon_half),
        ]
        c.create_polygon(*[v for pt in gnd_pts for v in pt],
                         fill=GND_C, outline="")

        # Ufuk çizgisi
        h_left  = rot(-horizon_half, pitch_px)
        h_right = rot( horizon_half, pitch_px)
        c.create_line(*h_left, *h_right, fill=ACCENT_C, width=2)

        # Pitch merdiveni
        for step in range(-40, 50, 10):
            if step == 0:
                continue
            py     = step * 4.5 - pitch_px
            length = 60 if abs(step) % 20 == 0 else 35
            lx1, ly1 = rot(-length, -py)
            lx2, ly2 = rot( length, -py)
            clr = "#4a9ecc" if step > 0 else "#cc7a4a"
            c.create_line(lx1, ly1, lx2, ly2, fill=clr, width=1)
            if abs(step) % 20 == 0:
                tx, ty = rot(length + 5, -py)
                c.create_text(tx, ty, text=f"{step:+d}",
                              fill=TEXT_C, font=("Courier New", 8), anchor="w")

        # Roll ark + tikler
        arc_r = min(W, H) * 0.40
        c.create_oval(cx - arc_r, cy - arc_r, cx + arc_r, cy + arc_r,
                      outline=DIM_C, width=1)
        for a in range(0, 360, 10):
            rad = math.radians(a)
            r1  = arc_r
            r2  = arc_r - (14 if a % 30 == 0 else 7)
            x1  = cx + math.sin(rad) * r1
            y1  = cy - math.cos(rad) * r1
            x2  = cx + math.sin(rad) * r2
            y2  = cy - math.cos(rad) * r2
            clr = ACCENT_C if a % 30 == 0 else DIM_C
            c.create_line(x1, y1, x2, y2, fill=clr, width=1)

        # Bank üçgeni
        tri_r = arc_r + 10
        c.create_polygon(
            cx, cy - tri_r,
            cx - 6, cy - tri_r + 11,
            cx + 6, cy - tri_r + 11,
            fill=ACCENT_C, outline="",
        )

        # Merkez rötikül
        span = arc_r * 0.55
        c.create_line(cx - span, cy, cx - 18, cy, fill=OK_C, width=2)
        c.create_line(cx + 18,   cy, cx + span, cy, fill=OK_C, width=2)
        c.create_line(cx, cy - 14, cx, cy + 14, fill=OK_C, width=2)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                      fill="#ffffff", outline="")

        alt  = self._cur["alt"]
        spd  = self._cur["spd"]
        vspd = self._cur["vspd"]
        hdg  = self._cur["heading"]

        bH = min(H * 0.62, 230)
        bY = cy - bH / 2
        bW = 44

        # ALT bandı (sağ)
        ax = W - bW - 8
        c.create_rectangle(ax, bY, ax + bW, bY + bH,
                           fill="#080a0f", outline=DIM_C)
        c.create_text(ax + bW // 2, bY - 8,
                      text="ALT", fill=ACCENT_C,
                      font=("Courier New", 8, "bold"))
        alt_range = 20
        for tick in range(-alt_range, alt_range + 1, 5):
            av = round(alt / 5) * 5 + tick
            ty = cy - (av - alt) * (bH / (alt_range * 2))
            if ty < bY or ty > bY + bH:
                continue
            maj = tick % 10 == 0
            c.create_line(ax, ty, ax + (10 if maj else 6), ty,
                          fill=(TEXT_C if maj else DIM_C), width=1)
            if maj:
                c.create_text(ax + 14, ty, text=f"{av:.0f}",
                              fill=TEXT_C, font=("Courier New", 8),
                              anchor="w")
        c.create_rectangle(ax - 2, cy - 9, ax + bW + 2, cy + 9,
                           fill=ACCENT_C)
        c.create_text(ax + bW // 2, cy + 1,
                      text=f"{alt:.1f}", fill=BG_C,
                      font=("Courier New", 10, "bold"))
        c.create_text(ax + bW // 2, bY + bH + 14,
                      text=f"VS {vspd:+.1f}",
                      fill=(OK_C if vspd >= 0 else ERR_C),
                      font=("Courier New", 8))

        # SPD bandı (sol)
        sx = 8
        c.create_rectangle(sx, bY, sx + bW, bY + bH,
                           fill="#080a0f", outline=DIM_C)
        c.create_text(sx + bW // 2, bY - 8,
                      text="SPD", fill=OK_C,
                      font=("Courier New", 8, "bold"))
        spd_range = 15
        for tick in range(-spd_range, spd_range + 1, 5):
            sv = round(spd / 5) * 5 + tick
            if sv < 0:
                continue
            ty = cy - (sv - spd) * (bH / (spd_range * 2))
            if ty < bY or ty > bY + bH:
                continue
            maj = tick % 10 == 0
            c.create_line(sx + bW, ty, sx + bW - (10 if maj else 6), ty,
                          fill=(TEXT_C if maj else DIM_C), width=1)
            if maj:
                c.create_text(sx + bW - 12, ty, text=f"{sv:.0f}",
                              fill=TEXT_C, font=("Courier New", 8),
                              anchor="e")
        c.create_rectangle(sx - 2, cy - 9, sx + bW + 2, cy + 9,
                           fill=OK_C)
        c.create_text(sx + bW // 2, cy + 1,
                      text=f"{spd:.1f}", fill=BG_C,
                      font=("Courier New", 10, "bold"))

        # Compass şeridi
        dirs = {0: "N", 45: "NE", 90: "E", 135: "SE",
                180: "S", 225: "SW", 270: "W", 315: "NW"}
        comp_w = min(W * 0.52, 240)
        comp_x = cx - comp_w // 2
        comp_y = bY - 34
        comp_h = 26
        c.create_rectangle(comp_x, comp_y, comp_x + comp_w, comp_y + comp_h,
                           fill="#080a0f", outline=DIM_C)
        deg_per_px = 180 / comp_w
        for d in range(-180, 181, 10):
            hval = int((hdg + d) % 360 + 360) % 360
            px   = comp_x + comp_w // 2 + d / deg_per_px
            if px < comp_x or px > comp_x + comp_w:
                continue
            is_card = hval % 90 == 0
            is_maj  = hval % 45 == 0
            tk_h    = 12 if is_card else (8 if is_maj else 4)
            clr     = ACCENT_C if is_card else (TEXT_C if is_maj else DIM_C)
            c.create_line(px, comp_y + comp_h, px,
                          comp_y + comp_h - tk_h, fill=clr, width=1)
            if is_maj:
                lbl2 = dirs.get(hval, str(hval))
                c.create_text(
                    px, comp_y + 10, text=lbl2, fill=clr,
                    font=("Courier New", 8, "bold" if is_card else "normal"),
                )
        c.create_polygon(
            cx, comp_y + comp_h + 5,
            cx - 4, comp_y + comp_h,
            cx + 4, comp_y + comp_h,
            fill=ACCENT_C,
        )
        c.create_rectangle(cx - 20, comp_y, cx + 20, comp_y + 14,
                           fill="#080a0f", outline="")
        c.create_text(cx, comp_y + 10, text=f"{int(hdg):03d}°",
                      fill=ACCENT_C, font=("Courier New", 9, "bold"))

        # Pitch / Roll etiketler
        pc = OK_C if abs(pitch) < 12 else (WARN_C if abs(pitch) < 25 else ERR_C)
        c.create_rectangle(cx - 44, H - 26, cx + 44, H - 8,
                           fill="#080a0f", outline="")
        c.create_text(cx, H - 14, text=f"PITCH {pitch:+.1f}°",
                      fill=pc, font=("Courier New", 9, "bold"))

        rc = OK_C if abs(roll) < 15 else (WARN_C if abs(roll) < 30 else ERR_C)
        c.create_rectangle(cx - 44, arc_r + cy - arc_r + 22,
                           cx + 44, arc_r + cy - arc_r + 36,
                           fill="#080a0f", outline="")
        c.create_text(cx, arc_r + cy - arc_r + 32,
                      text=f"ROLL {roll:+.1f}°",
                      fill=rc, font=("Courier New", 9, "bold"))


# ═══════════════════════════════════════════════════════════════
#  DUAL HUD PANELİ
# ═══════════════════════════════════════════════════════════════

class DualHudPanel(tk.Frame):
    """İHA (sol) ve İDA (sağ) HUD'larını yan yana gösterir."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg="#05070d", **kwargs)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(2, weight=1)
        self.rowconfigure(0, weight=1)

        self._huds: dict[str, HudCanvas] = {}

        for col, vid in enumerate(VEHICLE_IDS):
            hud = HudCanvas(self, vehicle_id=vid)
            hud.grid(row=0, column=col * 2, sticky="nsew")
            self._huds[vid] = hud

        # Separator
        tk.Frame(self, bg=PANEL_BDR, width=2).grid(
            row=0, column=1, sticky="ns", padx=2
        )

    def update(self, vid: str, state):
        if hud := self._huds.get(vid):
            hud.update(state)


# ═══════════════════════════════════════════════════════════════
#  TELEMETRİ MİNİ PANELİ (aktif araç metrikleri)
# ═══════════════════════════════════════════════════════════════

class TelemetryMiniPanel(tk.Frame):
    """
    Sağ panelde aktif araca ait telemetri metrik kartları.
    active_vehicle değişince update_vehicle_label() çağrılmalı.
    """

    METRICS = [
        ("İRTİFA",      "alt_rel",          "m"),
        ("HIZ",         "ground_speed",      "m/s"),
        ("HEADING",     "heading",           "°"),
        ("BATARYA",     "battery_remaining", "%"),
        ("GPS FIX",     "gps_fix_label",     ""),
        ("UYDU",        "satellites",        ""),
        ("PITCH",       "pitch",             "°"),
        ("ROLL",        "roll",              "°"),
        ("CLIMB",       "vspeed",            "m/s"),
        ("UÇUŞ SRSİ",  "flight_time",       ""),
    ]

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._vars: dict[str, tk.StringVar] = {}
        self._arm_dot:   Optional[tk.Label] = None
        self._arm_lbl:   Optional[tk.Label] = None
        self._mode_lbl:  Optional[tk.Label] = None
        self._title_lbl: Optional[tk.Label] = None
        self._build()

    def _build(self):
        outer, inner = make_panel(self, "◎ TELEMETRİ — İHA")
        outer.pack(fill="both", expand=True)

        # Başlık label referansını sakla
        for w in inner.winfo_children():
            if isinstance(w, tk.Label):
                self._title_lbl = w
                break

        inner.columnconfigure(0, weight=1)
        inner.columnconfigure(1, weight=1)

        for i, (label, attr, unit) in enumerate(self.METRICS):
            row = i // 2 + 2
            col = i % 2

            cell = tk.Frame(inner, bg=BTN_H, padx=8, pady=5)
            cell.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
            cell.columnconfigure(0, weight=1)

            tk.Label(cell, text=label, bg=BTN_H, fg=TEXT_DIM,
                     font=FONTS["label"], anchor="w").pack(fill="x")

            var = tk.StringVar(value="---")
            self._vars[attr] = var
            tk.Label(cell, textvariable=var, bg=BTN_H, fg=ACCENT,
                     font=FONTS["value"], anchor="w").pack(fill="x")

            if unit:
                tk.Label(cell, text=unit, bg=BTN_H, fg=TEXT_DIM,
                         font=FONTS["label"], anchor="e").pack(fill="x")

        # Arm + mod satırı
        next_row = len(self.METRICS) // 2 + 2 + (1 if len(self.METRICS) % 2 else 0)
        status_row = tk.Frame(inner, bg=PANEL_BG, pady=6)
        status_row.grid(row=next_row, column=0, columnspan=2,
                        sticky="ew", padx=3, pady=(6, 3))

        self._arm_dot = tk.Label(
            status_row, text="●", bg=PANEL_BG,
            fg=ACCENT2, font=("Courier New", 14)
        )
        self._arm_dot.pack(side="left", padx=8)

        self._arm_lbl = tk.Label(
            status_row, text="---", bg=PANEL_BG, fg=ACCENT2,
            font=("Courier New", 10, "bold")
        )
        self._arm_lbl.pack(side="left")

        self._mode_lbl = tk.Label(
            status_row, text="---", bg=PANEL_BG, fg=WARN,
            font=("Courier New", 10, "bold")
        )
        self._mode_lbl.pack(side="right", padx=8)

    def update_vehicle_label(self, vid: str):
        """Aktif araç değişince başlığı güncelle."""
        color = IHA_COLOR if vid == "iha" else IDA_COLOR
        label = VEHICLE_LABELS[vid]
        if self._title_lbl:
            self._title_lbl.configure(
                text=f"◎ TELEMETRİ — {label}", fg=color
            )

    def update(self, state):
        """TelemetryState → widget güncelle."""

        def fmt(val, attr: str) -> str:
            if attr == "battery_remaining":
                return f"{val}%" if val >= 0 else "---"
            if attr == "flight_time":
                mins = int(val // 60)
                secs = int(val % 60)
                return f"{mins:02d}:{secs:02d}"
            if attr in ("alt_rel", "ground_speed", "vspeed",
                        "pitch", "roll"):
                return f"{val:.1f}"
            if attr == "heading":
                return f"{val:.0f}"
            return str(val)

        for attr, var in self._vars.items():
            val = getattr(state, attr, None)
            if val is not None:
                var.set(fmt(val, attr))

        # Arm durumu
        armed = getattr(state, "armed", False)
        msg   = getattr(state, "msg", None)
        valid = (msg is not None
                 and hasattr(msg, "get_type")
                 and msg.get_type() == "HEARTBEAT"
                 and msg.type != 27)

        if self._arm_dot and self._arm_lbl and self._mode_lbl:
            if valid:
                if armed:
                    self._arm_dot.configure(fg=OK)
                    self._arm_lbl.configure(text="ARMED", fg=OK)
                else:
                    self._arm_dot.configure(fg=ACCENT2)
                    self._arm_lbl.configure(text="DISARMED", fg=ACCENT2)
                mode = getattr(state, "mode", "---")
                self._mode_lbl.configure(text=mode, fg=WARN)


# ═══════════════════════════════════════════════════════════════
#  KONTROL PANELİ
# ═══════════════════════════════════════════════════════════════

class ControlPanel(tk.Frame):
    """
    ARM/DISARM, mod seçici, takeoff, RTL, acil durdurma.
    Komutlar active_vehicle'a (veya "all" ise her ikisine) gönderilir.
    """

    def __init__(self, parent, vehicle_manager: VehicleManager,
                 get_active_vehicle, on_log=None, bridge=None, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._vmgr       = vehicle_manager
        self._get_active = get_active_vehicle
        self._on_log     = on_log
        self._bridge     = bridge          # CommunicationBridge (görev köprüsü)
        self._build()

    def _build(self):
        outer, inner = make_panel(self, "◎ KONTROL")
        outer.pack(fill="both", expand=True)

        # ARM / DISARM
        arm_row = tk.Frame(inner, bg=PANEL_BG)
        arm_row.grid(row=2, column=0, sticky="ew", padx=6, pady=(6, 3))
        arm_row.columnconfigure(0, weight=1)
        arm_row.columnconfigure(1, weight=1)

        make_btn(arm_row, "⚡ ARM",    OK,       self._cmd_arm
                 ).grid(row=0, column=0, sticky="ew", padx=(0, 3))
        make_btn(arm_row, "⏹ DISARM", TEXT_DIM, self._cmd_disarm
                 ).grid(row=0, column=1, sticky="ew")

        # Mod seçici
        mode_row = tk.Frame(inner, bg=PANEL_BG)
        mode_row.grid(row=3, column=0, sticky="ew", padx=6, pady=3)

        tk.Label(mode_row, text="MOD:", bg=PANEL_BG, fg=TEXT_DIM,
                 font=FONTS["label"]).pack(side="left")

        self._mode_var = tk.StringVar(value="---")
        # Mod listesi aktif araca göre doldurulur (İHA→Copter, İDA→Rover).
        self._mode_cb = ttk.Combobox(
            mode_row, textvariable=self._mode_var,
            values=mode_names_for_vehicle(self._get_active()),
            state="readonly", width=14,
        )
        self._mode_cb.pack(side="left", padx=6)
        make_btn(mode_row, "SET", BTN_H, self._cmd_set_mode,
                 width=5).pack(side="left")

        # Komut butonları
        cmds = [
            ("🛫 TAKEOFF",  OK,      self._cmd_takeoff),
            ("🏠 RTL",      WARN,    self._cmd_rtl),
            ("🔴 ACİL DUR", ACCENT2, self._cmd_emergency),
        ]
        for i, (text, color, cmd) in enumerate(cmds):
            make_btn(inner, text, color, cmd).grid(
                row=4 + i, column=0, sticky="ew", padx=6, pady=2
            )

        # ── GÖREV KÖPRÜSÜ (İHA → İDA RENK aktarımı) ──
        if self._bridge is not None:
            tk.Frame(inner, bg=PANEL_BDR, height=1).grid(
                row=7, column=0, sticky="ew", padx=6, pady=(8, 4)
            )
            tk.Label(
                inner, text="GÖREV — RENK KÖPRÜSÜ (İHA→İDA)",
                bg=PANEL_BG, fg=TEXT_DIM, font=FONTS["label"],
            ).grid(row=8, column=0, sticky="w", padx=6)

            self._mission_btn = make_btn(
                inner, "🎨 GÖREVİ BAŞLAT", OK, self._cmd_toggle_mission
            )
            self._mission_btn.grid(row=9, column=0, sticky="ew", padx=6, pady=(2, 6))
            self._refresh_mission_btn()

    def _vid(self) -> str:
        return self._get_active()

    # ── Görev köprüsü aç/kapa ──
    def _cmd_toggle_mission(self):
        if self._bridge is None:
            return
        active = self._bridge.toggle()
        self._refresh_mission_btn()
        if active:
            self._log("Görev köprüsü AKTİF (İHA→İDA renk aktarımı)", "ok")
        else:
            self._log("Görev köprüsü PASİF", "warn")

    def _refresh_mission_btn(self):
        """Buton metnini/rengini köprü durumuna göre günceller."""
        if self._bridge is None or not hasattr(self, "_mission_btn"):
            return
        if self._bridge.is_enabled():
            txt, col = "⏹ GÖREVİ DURDUR", ACCENT2
        else:
            txt, col = "🎨 GÖREVİ BAŞLAT", OK
        self._mission_btn.configure(text=txt, bg=col, activebackground=col)
        # Hover renklerini de yeni renge bağla
        self._mission_btn.bind("<Enter>", lambda e: self._mission_btn.configure(bg=lighten(col)))
        self._mission_btn.bind("<Leave>", lambda e: self._mission_btn.configure(bg=col))

    def _log(self, msg: str, level: str = "info"):
        if self._on_log:
            self._on_log(msg, level, self._vid())

    def _cmd_arm(self):
        if not messagebox.askyesno("ARM", "Aracı ARM etmek istiyor musunuz?"):
            return
        self._vmgr.arm(self._vid())
        self._log("ARM komutu gönderildi", "warn")

    def _cmd_disarm(self):
        self._vmgr.disarm(self._vid())
        self._log("DISARM komutu gönderildi", "info")

    def refresh_modes(self):
        """
        Aktif araca göre mod listesini yenile (İHA→Copter, İDA→Rover).
        Araç değişiminde MainWindow tarafından çağrılır. Geçerli seçim yeni
        araçta yoksa seçim sıfırlanır.
        """
        if not getattr(self, "_mode_cb", None):
            return
        names = mode_names_for_vehicle(self._get_active())
        self._mode_cb.configure(values=names)
        if self._mode_var.get() not in names:
            self._mode_var.set("---")

    def _cmd_set_mode(self):
        mode = self._mode_var.get()
        if mode == "---":
            return
        self._vmgr.set_mode(self._vid(), mode)
        self._log(f"Mod → {mode}", "info")

    def _cmd_takeoff(self):
        if not messagebox.askyesno("TAKEOFF", "Takeoff komutu gönderilsin mi?"):
            return
        self._vmgr.takeoff(self._vid(), alt=10.0)
        self._log("Takeoff → 10m", "warn")

    def _cmd_rtl(self):
        if not messagebox.askyesno("RTL", "Return to Launch gönderilsin mi?"):
            return
        self._vmgr.rtl(self._vid())
        self._log("RTL komutu gönderildi", "warn")

    def _cmd_emergency(self):
        if messagebox.askyesno(
            "⚠️ ACİL DURDURMA",
            "MOTORLAR DURDURULACAK!\nDevam etmek istiyor musunuz?",
            icon="warning",
        ):
            self._vmgr.emergency_stop(self._vid())
            self._log("!!! ACİL DURDURMA !!!", "error")


# ═══════════════════════════════════════════════════════════════
#  WAYPOINT LİSTESİ
# ═══════════════════════════════════════════════════════════════

class WaypointListPanel(tk.Frame):
    def __init__(self, parent, get_map_ctrl, get_active_vehicle,
                 vehicle_manager, on_log=None, **kwargs):
        super().__init__(parent, bg=BG, **kwargs)
        self._get_map  = get_map_ctrl
        self._get_vid  = get_active_vehicle
        self._vmgr     = vehicle_manager
        self._on_log   = on_log
        self._build()

    def _build(self):
        outer, inner = make_panel(self, "◎ WAYPOINT LİSTESİ")
        outer.pack(fill="both", expand=True)

        list_frame = tk.Frame(inner, bg=PANEL_BG)
        list_frame.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)

        self._listbox = tk.Listbox(
            list_frame, bg=BTN_H, fg=TEXT,
            font=FONTS["mono_sm"], relief="flat",
            selectbackground=ACCENT, selectforeground=BG,
            activestyle="none", bd=0, highlightthickness=0, height=6,
        )
        self._listbox.grid(row=0, column=0, sticky="nsew")

        sb = tk.Scrollbar(list_frame, command=self._listbox.yview,
                          bg=PANEL_BG)
        sb.grid(row=0, column=1, sticky="ns")
        self._listbox.config(yscrollcommand=sb.set)

        btn_row = tk.Frame(inner, bg=PANEL_BG)
        btn_row.grid(row=3, column=0, sticky="ew", padx=6, pady=4)

        make_btn(btn_row, "SİL",       ACCENT2,  self._remove_selected, width=6
                 ).pack(side="left")
        make_btn(btn_row, "TÜMÜNÜ SİL", TEXT_DIM, self._clear_all, width=12
                 ).pack(side="left", padx=4)
        make_btn(btn_row, "GÖREVİ GÖNDER", OK,  self._send_mission, width=14
                 ).pack(side="right")

    def add(self, wp):
        self._listbox.insert(
            "end",
            f"WP{wp.idx:02d}  {wp.lat:.5f}, {wp.lon:.5f}  {wp.alt:.0f}m",
        )

    def _remove_selected(self):
        sel = self._listbox.curselection()
        mc  = self._get_map()
        if not sel or not mc:
            return
        wps = mc.get_waypoints()
        if sel[0] < len(wps):
            mc.remove_waypoint(wps[sel[0]].idx)
            self._listbox.delete(sel[0])

    def _clear_all(self):
        mc = self._get_map()
        if mc:
            mc.clear_waypoints()
            self._listbox.delete(0, "end")

    def _send_mission(self):
        """
        Listedeki TÜM waypoint'leri tek bir görev olarak araca yükler.
        Yükleme MAVLink mission protokolüyle yapılır ve GUI'yi dondurmamak
        için ayrı bir thread'de çalışır. AUTO'ya geçiş, yükleme tamamlanana
        (MISSION_ACK) kadar VehicleConnection tarafından engellenir.
        """
        mc = self._get_map()
        if not mc:
            return
        wps = mc.get_waypoints()
        if not wps:
            if self._on_log:
                self._on_log("Gönderilecek waypoint yok", "warn", self._get_vid())
            return

        vid   = self._get_vid()
        items = [(w.lat, w.lon, w.alt) for w in wps]

        def _worker():
            # upload_mission kendi ilerleme/başarı loglarını bus'a yayınlar.
            self._vmgr.upload_mission(vid, items)

        threading.Thread(
            target=_worker, daemon=True, name=f"mission-upload-{vid}"
        ).start()


# ═══════════════════════════════════════════════════════════════
#  ANA UYGULAMA
# ═══════════════════════════════════════════════════════════════

class GCSApplication:
    """
    YKİ — Ana GUI koordinatörü.
    Bileşenleri oluşturur, EventBus event'lerini yönlendirir.
    """

    def __init__(self, root: tk.Tk, event_bus, vehicle_manager: VehicleManager,
                 bridge=None):
        self.root   = root
        self.bus    = event_bus
        self.vmgr   = vehicle_manager
        self.bridge = bridge          # CommunicationBridge (görev köprüsü)

        self._map_ctrl: Optional[MapController] = None
        self._telemetry_cache: dict[str, object] = {}
        self._graphs_win: Optional[ControlGraphsWindow] = None

        self._configure_root()
        self._build_ui()
        self._subscribe_events()
        self._start_poll()

        log.info("GCSApplication hazır")

    def set_map_controller(self, ctrl: MapController):
        self._map_ctrl = ctrl

    # ── Root ────────────────────────────

    def _configure_root(self):
        self.root.title(WINDOW_TITLE)
        self.root.configure(bg=BG)
        self.root.geometry(WINDOW_GEOMETRY)
        self.root.minsize(*WINDOW_MIN)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "TCombobox",
            fieldbackground=PANEL_BG, background=PANEL_BG,
            foreground=TEXT, selectbackground=BTN_H,
            arrowcolor=ACCENT, bordercolor=PANEL_BDR,
            font=FONTS["mono"],
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", PANEL_BG)],
            foreground=[("readonly", TEXT)],
        )

    # ── UI İnşası ───────────────────────

    def _build_ui(self):
        # Header
        self._header = HeaderPanel(
            self.root,
            vehicle_manager=self.vmgr,
            on_vehicle_change=self._on_vehicle_change,
            on_log=self._on_log_request,
        )
        self._header.pack(fill="x", padx=16)

        tk.Frame(self.root, bg=PANEL_BDR, height=1).pack(fill="x", padx=16)

        # Body (3 kolon)
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=(10, 0))

        body.columnconfigure(0, minsize=220)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, minsize=220)
        body.rowconfigure(0, weight=1)

        self._build_left_panel(body)
        self._build_center_panel(body)
        self._build_right_panel(body)

        # Status bar
        tk.Frame(self.root, bg=PANEL_BDR, height=1).pack(
            fill="x", padx=16, pady=(8, 0)
        )
        self._build_statusbar()

    def _build_left_panel(self, parent):
        """Sol: Çift log paneli."""
        self._log_panel = LogPanel(parent)
        self._log_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

    def _build_center_panel(self, parent):
        """Orta: Harita + Dual HUD + araç koordinatları + harita araçları."""
        outer, inner = make_panel(parent, "◎ HARİTA / KAMERA")
        outer.grid(row=0, column=1, sticky="nsew", padx=(0, 8))

        inner.columnconfigure(0, weight=1)
        inner.rowconfigure(2, weight=3)   # harita
        inner.rowconfigure(3, weight=2)   # dual HUD

        # Map frame
        self.map_frame = tk.Frame(inner, bg="#000000")
        self.map_frame.grid(row=2, column=0, sticky="nsew")
        self.map_frame.rowconfigure(0, weight=1)
        self.map_frame.columnconfigure(0, weight=1)

        # Dual HUD
        self._dual_hud = DualHudPanel(inner)
        self._dual_hud.grid(row=3, column=0, sticky="nsew")

        # GPS koordinat barı
        coord_bar = tk.Frame(inner, bg=COORD_BG, pady=4)
        coord_bar.grid(row=4, column=0, sticky="ew")

        tk.Label(
            coord_bar, text="GPS", bg=COORD_BG, fg=TEXT_DIM,
            font=FONTS["label"], padx=8,
        ).pack(side="left")

        self._coord_var = tk.StringVar(value="Lat: ---  Lon: ---  Alt: ---m")
        tk.Label(
            coord_bar, textvariable=self._coord_var,
            bg=COORD_BG, fg=ACCENT, font=FONTS["mono_sm"], padx=8,
        ).pack(side="left")

        self._gps_vehicle_lbl = tk.Label(
            coord_bar, text="[İHA]",
            bg=COORD_BG, fg=IHA_COLOR, font=FONTS["mono_sm"], padx=8,
        )
        self._gps_vehicle_lbl.pack(side="right")

        # Harita araç araçları
        map_tools = tk.Frame(inner, bg=PANEL_BG, pady=4)
        map_tools.grid(row=5, column=0, sticky="ew")

        self._follow_btn = make_btn(
            map_tools, "📍 TAKİP: AÇIK", OK, self._toggle_follow, width=14
        )
        self._follow_btn.pack(side="left", padx=6)

        self._wp_mode_var = tk.BooleanVar(value=False)
        self._wp_btn = make_btn(
            map_tools, "📌 WP MODU", TEXT_DIM,
            self._toggle_waypoint_mode, width=12
        )
        self._wp_btn.pack(side="left", padx=2)

        make_btn(
            map_tools, "🔲 TRAIL SİL", TEXT_DIM,
            self._clear_trail, width=12
        ).pack(side="left", padx=2)

        make_btn(
            map_tools, "📈 GRAFİKLER", ACCENT,
            self._open_control_graphs, width=12
        ).pack(side="left", padx=2)

        # Tile seçici
        tile_frame = tk.Frame(map_tools, bg=PANEL_BG)
        tile_frame.pack(side="right", padx=8)
        tk.Label(
            tile_frame, text="HARİTA:", bg=PANEL_BG, fg=TEXT_DIM,
            font=FONTS["label"],
        ).pack(side="left")
        self._tile_var = tk.StringVar(value="OpenStreetMap")
        tile_cb = ttk.Combobox(
            tile_frame, textvariable=self._tile_var,
            values=["OpenStreetMap", "Satellite"],
            state="readonly", width=14, font=FONTS["mono_sm"],
        )
        tile_cb.pack(side="left", padx=4)
        tile_cb.bind("<<ComboboxSelected>>", self._on_tile_change)

    def _build_right_panel(self, parent):
        """Sağ: Telemetri + Kontrol + Waypoint listesi."""
        right = tk.Frame(parent, bg=BG)
        right.grid(row=0, column=2, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)

        self._telem_panel = TelemetryMiniPanel(right)
        self._telem_panel.grid(row=0, column=0, sticky="nsew", pady=(0, 8))

        self._ctrl_panel = ControlPanel(
            right,
            vehicle_manager=self.vmgr,
            get_active_vehicle=lambda: self._header.active_vehicle,
            on_log=self._on_log_request,
            bridge=self.bridge,
        )
        self._ctrl_panel.grid(row=1, column=0, sticky="ew", pady=(0, 8))

        self._wp_list = WaypointListPanel(
            right,
            get_map_ctrl=lambda: self._map_ctrl,
            get_active_vehicle=lambda: self._header.active_vehicle,
            vehicle_manager=self.vmgr,
            on_log=self._on_log_request,
        )
        self._wp_list.grid(row=2, column=0, sticky="nsew")

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=BG, pady=5)
        bar.pack(fill="x", padx=16)

        self._status_lbl = tk.Label(
            bar, text="Hazır", fg=TEXT_DIM, bg=BG, font=FONTS["mono_sm"]
        )
        self._status_lbl.pack(side="left", padx=4)

        tk.Label(
            bar,
            text=f"v{APP_VERSION}  |  ArduPilot GCS  |  İHA + İDA Çift Araç",
            fg=TEXT_DIM, bg=BG, font=FONTS["mono_sm"],
        ).pack(side="right")

    # ── EventBus ────────────────────────

    def _subscribe_events(self):
        self.bus.subscribe("log",        self._on_log)
        self.bus.subscribe("telemetry",  self._on_telemetry)
        self.bus.subscribe("conn_state", self._on_conn_state)

    def _on_log(self, data: dict):
        if not isinstance(data, dict):
            return
        msg   = data.get("msg", "")
        level = data.get("level", "info")
        vid   = data.get("vehicle") or VEHICLE_IDS[0]
        # Bilinmeyen vehicle → ilk araç
        if vid not in VEHICLE_IDS:
            vid = VEHICLE_IDS[0]
        self._log_panel.append(vid, msg, level)
        self._status_lbl.configure(
            text=f"[{VEHICLE_LABELS.get(vid, vid)}] {msg}"
        )

    def _on_telemetry(self, data):
        if isinstance(data, dict):
            vid   = data.get("vehicle", "iha")
            state = data.get("payload", data)
        else:
            state = data
            vid   = "iha"

        if not hasattr(state, "lat"):
            return

        # Cache güncelle
        self._telemetry_cache[vid] = state

        # Her araç kendi HUD'unu güncelle
        self._dual_hud.update(vid, state)

        # Aktif araç → sağ panel
        active = self._header.active_vehicle
        if vid == active:
            self._telem_panel.update(state)
            self._coord_var.set(
                f"Lat: {state.lat:.6f}  Lon: {state.lon:.6f}"
                f"  Alt: {state.alt_rel:.1f}m"
            )

    def _on_conn_state(self, data: dict):
        if not isinstance(data, dict):
            return
        vid   = data.get("vehicle", "iha")
        state = data.get("state", ConnState.DISCONNECTED)
        self._header.update_conn_state(vid, state)

        iha_ok = (v := self.vmgr.get("iha")) and v.is_connected()
        ida_ok = (v := self.vmgr.get("ida")) and v.is_connected()

        if iha_ok and ida_ok:
            self._status_lbl.configure(text="İHA + İDA bağlı ✓", fg=OK)
        elif iha_ok:
            self._status_lbl.configure(text="İHA bağlı", fg=IHA_COLOR)
        elif ida_ok:
            self._status_lbl.configure(text="İDA bağlı", fg=IDA_COLOR)
        else:
            self._status_lbl.configure(text="Bağlantı yok", fg=TEXT_DIM)

    # ── Araç değişimi ───────────────────

    def _on_vehicle_change(self, vid: str):
        color = IHA_COLOR if vid == "iha" else IDA_COLOR
        label = VEHICLE_LABELS[vid]

        self._gps_vehicle_lbl.configure(text=f"[{label}]", fg=color)
        self._telem_panel.update_vehicle_label(vid)

        # Mod combobox'ını yeni aracın firmware'ine göre güncelle.
        if getattr(self, "_ctrl_panel", None):
            self._ctrl_panel.refresh_modes()

        if self._map_ctrl:
            self._map_ctrl.set_follow_mode(
                self._map_ctrl.follow_mode,
                vid
            )
            self._map_ctrl.center_on_vehicle(vid)

        state = self._telemetry_cache.get(vid)

        if state:
            self._telem_panel.update(state)
            self._coord_var.set(
                f"Lat: {state.lat:.6f}  Lon: {state.lon:.6f}"
                f"  Alt: {state.alt_rel:.1f}m"
            )
        else:
            self._coord_var.set("Lat: ---  Lon: ---  Alt: ---m")

        log.info(f"Aktif araç: {vid}")

    # ── Log yardımcısı ──────────────────

    def _on_log_request(self, msg: str, level: str = "info",
                         vid: Optional[str] = None):
        if vid is None:
            vid = self._header.active_vehicle
        if vid not in VEHICLE_IDS:
            vid = VEHICLE_IDS[0]
        self._log_panel.append(vid, msg, level)

    # ── Harita araçları ─────────────────

    def on_waypoint_added(self, wp):
        self._wp_list.add(wp)

    def _open_control_graphs(self):
        """
        Kontrol Grafikleri penceresini açar. Zaten açıksa öne getirir
        (tek örnek). Pencere kapanınca referans otomatik temizlenir.
        """
        if self._graphs_win is not None and self._graphs_win.winfo_exists():
            self._graphs_win.deiconify()
            self._graphs_win.lift()
            self._graphs_win.focus_force()
            return

        self._graphs_win = ControlGraphsWindow(
            self.root,
            self.bus,
            get_active_vehicle=lambda: self._header.active_vehicle,
            vehicle_manager=self.vmgr,
            on_close=self._on_graphs_closed,
        )

    def _on_graphs_closed(self):
        self._graphs_win = None

    def _toggle_follow(self):
        if not self._map_ctrl:
            return
        state = self._map_ctrl.toggle_follow_mode()
        text  = "📍 TAKİP: AÇIK" if state else "📍 TAKİP: KAPALI"
        color = OK if state else TEXT_DIM
        self._follow_btn.configure(text=text, bg=color)

    def _toggle_waypoint_mode(self):
        if not self._map_ctrl:
            return
        new = not self._wp_mode_var.get()
        self._wp_mode_var.set(new)
        self._map_ctrl.enable_waypoint_mode(new)
        self._wp_btn.configure(bg=ACCENT if new else TEXT_DIM)
        self._on_log_request("WP modu " + ("aktif" if new else "kapalı"), "dim")

    def _clear_trail(self):
        if self._map_ctrl:
            self._map_ctrl.clear_trail()
            self._on_log_request("Trail temizlendi", "dim")

    def _on_tile_change(self, _event=None):
        if self._map_ctrl:
            self._map_ctrl.set_tile_server(self._tile_var.get())

    # ── Poll & Kapat ────────────────────

    def _start_poll(self):
        self.bus.poll()
        self.root.after(POLL_INTERVAL_MS, self._start_poll)

    def _on_close(self):
        log.info("Uygulama kapatılıyor...")
        if self._graphs_win is not None and self._graphs_win.winfo_exists():
            try:
                self._graphs_win._on_close()
            except Exception:
                pass
        self.vmgr.stop_all()
        self.root.destroy()