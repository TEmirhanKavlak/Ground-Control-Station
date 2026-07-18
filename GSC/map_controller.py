"""
map_controller.py
─────────────────
TkinterMapView üzerine kurulu çift araçlı harita yöneticisi.

v4: İHA ve İDA için ayrı marker, ayrı trail, ayrı renk.
    Telemetry eventleri {"vehicle": ..., "payload": state} formatında gelir.
"""

import logging
import math
import tkinter as tk
from dataclasses import dataclass, field
from typing import Optional

from PIL import Image, ImageTk
from tkintermapview import TkinterMapView

from constants import IHA_COLOR, IDA_COLOR, VEHICLE_IDS

log = logging.getLogger("map")


@dataclass
class Waypoint:
    idx:    int
    lat:    float
    lon:    float
    alt:    float  = 20.0
    marker: object = field(default=None, repr=False)


@dataclass
class VehicleMapState:
    """Tek bir aracın harita durumu."""
    vehicle_id:   str
    color:        str
    trail_coords: list = field(default_factory=list)
    trail_path:   object = field(default=None, repr=False)
    marker:       object = field(default=None, repr=False)
    wake_markers: list   = field(default_factory=list)
    first_fix:    bool   = True
    lat:          float  = 0.0
    lon:          float  = 0.0
    home_marker:  object = field(default=None, repr=False)
    home_set:     bool   = False


VEHICLE_MAP_COLORS: dict[str, str] = {
    "iha": IHA_COLOR,
    "ida": IDA_COLOR,
}


class MapController:
    """
    TkinterMapView sarmalayan çift araçlı harita controller.
    SADECE GUI thread'inden çağrılmalı.
    """

    TRAIL_MAX_POINTS = 2000
    TRAIL_MIN_DIST_M = 1.0
    MARKER_WP_COLOR  = "#00e5ff"

    def __init__(self, parent_frame: tk.Frame, event_bus,
                 vehicle_manager, on_waypoint_add=None):
        self.bus         = event_bus
        self.vehicle_mgr = vehicle_manager
        self.on_wp_add   = on_waypoint_add

        # Harita widget
        self.map = TkinterMapView(parent_frame, corner_radius=0)
        self.map.grid(row=0, column=0, sticky="nsew")

        # Araç durumları
        self._vehicles: dict[str, VehicleMapState] = {
            vid: VehicleMapState(vehicle_id=vid, color=VEHICLE_MAP_COLORS[vid])
            for vid in VEHICLE_IDS
        }

        # Harita ayarları
        self.follow_mode    = True
        self.follow_vehicle = "iha"

        # Araç iconları
        self._vehicle_base_images = {
            "iha": Image.open("assets/drone.png").convert("RGBA"),
            "ida": Image.open("assets/ship.png").convert("RGBA"),
        }
        # Tkinter PhotoImage cache
        self._vehicle_tk_images: dict[str, Optional[ImageTk.PhotoImage]] = {
            vid: None for vid in VEHICLE_IDS
        }

        # Waypoints
        self._waypoints:  list[Waypoint] = []
        self._wp_counter: int            = 0

        # Waypoint modu
        self._waypoint_mode = False
        self.map.add_right_click_menu_command(
            label="Waypoint Ekle",
            command=self._on_map_right_click,
            pass_coords=True,
        )

        # Başlangıç görünümü (Adana civarı)
        self.map.set_position(37.0, 35.32)
        self.map.set_zoom(15)

        # Tile seçenekleri
        self._tile_servers = {
            "OpenStreetMap": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
            "Satellite":     "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga",
        }

        # EventBus subscriptions
        self.bus.subscribe("telemetry", self._on_telemetry)

    # ──────────────────────────────────────
    #  PUBLIC API
    # ──────────────────────────────────────

    def set_follow_mode(self, enabled: bool, vehicle_id: str = "iha") -> None:
        self.follow_mode    = enabled
        self.follow_vehicle = vehicle_id

    def toggle_follow_mode(self) -> bool:
        self.follow_mode = not self.follow_mode
        return self.follow_mode

    def set_tile_server(self, name: str) -> None:
        url = self._tile_servers.get(name)
        if url:
            self.map.set_tile_server(url)

    def enable_waypoint_mode(self, enabled: bool) -> None:
        self._waypoint_mode = enabled
        if enabled:
            self.map.add_left_click_map_command(self._on_map_left_click)
        else:
            self.map.add_left_click_map_command(None)

    def center_on_vehicle(self, vehicle_id: str = "iha") -> None:
        vs = self._vehicles.get(vehicle_id)
        if vs and (vs.lat or vs.lon):
            self.map.set_position(vs.lat, vs.lon)

    def add_waypoint(self, lat: float, lon: float, alt: float = 20.0,
                     send_to_vehicle: Optional[str] = None) -> Waypoint:
        self._wp_counter += 1
        wp = Waypoint(idx=self._wp_counter, lat=lat, lon=lon, alt=alt)
        wp.marker = self.map.set_marker(
            lat, lon,
            text=f"WP{wp.idx}",
            marker_color_circle=self.MARKER_WP_COLOR,
            marker_color_outside=self.MARKER_WP_COLOR,
        )
        self._waypoints.append(wp)

        if send_to_vehicle and self.vehicle_mgr:
            self.vehicle_mgr.send_waypoint(send_to_vehicle, lat, lon, alt)

        if self.on_wp_add:
            self.on_wp_add(wp)

        log.info(f"Waypoint {wp.idx} eklendi: {lat:.6f}, {lon:.6f}, {alt}m")
        return wp

    def remove_waypoint(self, idx: int) -> None:
        wp = next((w for w in self._waypoints if w.idx == idx), None)
        if not wp:
            return
        if wp.marker:
            wp.marker.delete()
        self._waypoints.remove(wp)

    def clear_waypoints(self) -> None:
        for wp in self._waypoints:
            if wp.marker:
                wp.marker.delete()
        self._waypoints.clear()
        self._wp_counter = 0

    def get_waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    def clear_trail(self, vehicle_id: Optional[str] = None) -> None:
        targets = [vehicle_id] if vehicle_id else list(VEHICLE_IDS)
        for vid in targets:
            vs = self._vehicles.get(vid)
            if vs:
                vs.trail_coords.clear()
                if vs.trail_path:
                    vs.trail_path.delete()
                    vs.trail_path = None

    # ──────────────────────────────────────
    #  INTERNAL — Telemetry
    # ──────────────────────────────────────

    def _on_telemetry(self, data) -> None:
        """
        Telemetry event: {"vehicle": "iha"/"ida", "payload": TelemetryState}
        """
        if isinstance(data, dict):
            vehicle_id = data.get("vehicle", "iha")
            state      = data.get("payload", data)
        else:
            state      = data
            vehicle_id = "iha"

        if hasattr(state, "lat"):
            lat = state.lat
            lon = state.lon
            hdg = state.heading
        elif isinstance(state, dict):
            lat = state.get("lat", 0.0)
            lon = state.get("lon", 0.0)
            hdg = state.get("heading", 0.0)
        else:
            return

        if lat == 0.0 and lon == 0.0:
            return

        vs = self._vehicles.get(vehicle_id)
        if not vs:
            return

        # İlk fix: merkezi İHA'ya kaydır
        if vs.first_fix:
            if vehicle_id == "iha":
                self.map.set_position(lat, lon)
            self._set_home(vs, lat, lon)
            vs.first_fix = False

        self._update_marker(vs, lat, lon, hdg)
        self._update_trail(vs, lat, lon)

        if self.follow_mode and vehicle_id == self.follow_vehicle:
            self.map.set_position(lat, lon)

        vs.lat = lat
        vs.lon = lon

    def _set_home(self, vs: VehicleMapState, lat: float, lon: float) -> None:
        if vs.home_marker:
            vs.home_marker.delete()
        color = VEHICLE_MAP_COLORS[vs.vehicle_id]
        label = "🏠 İHA HOME" if vs.vehicle_id == "iha" else "🏠 İDA HOME"
        vs.home_marker = self.map.set_marker(
            lat, lon,
            text=label,
            marker_color_circle=color,
            marker_color_outside=color,
        )
        vs.home_set = True

    def _update_marker(self, vs, lat, lon, heading):
        try:
            base_img = self._vehicle_base_images.get(vs.vehicle_id)
            if not base_img:
                return

            rotated = base_img.rotate(
                -heading,
                expand=True,
                resample=Image.BICUBIC
            )

            if vs.vehicle_id == "iha":
                size = (52, 52)   # drone biraz daha büyük
            else:
                size = (42, 42)   # gemi normal kalsın

            resized = rotated.resize(size, Image.LANCZOS)

            self._vehicle_tk_images[vs.vehicle_id] = ImageTk.PhotoImage(resized)
            icon = self._vehicle_tk_images[vs.vehicle_id]

            if vs.marker is None:
                vs.marker = self.map.set_marker(
                    lat,
                    lon,
                    text="",
                    icon=icon
                )
            else:
                vs.marker.set_position(lat, lon)
                vs.marker.change_icon(icon)

        except Exception as e:
            log.error(f"[{vs.vehicle_id}] Marker güncelleme hatası: {e}")

    def _update_trail(self, vs: VehicleMapState,
                      lat: float, lon: float) -> None:
        if vs.trail_coords:
            last_lat, last_lon = vs.trail_coords[-1]
            dist = self._haversine(last_lat, last_lon, lat, lon)
            if dist < self.TRAIL_MIN_DIST_M:
                return

        vs.trail_coords.append((lat, lon))

        if len(vs.trail_coords) > self.TRAIL_MAX_POINTS:
            vs.trail_coords = vs.trail_coords[-self.TRAIL_MAX_POINTS:]

        if len(vs.trail_coords) >= 2:
            try:
                if vs.trail_path:
                    vs.trail_path.delete()
                vs.trail_path = self.map.set_path(
                    vs.trail_coords,
                    color=VEHICLE_MAP_COLORS[vs.vehicle_id],
                    width=2,
                )
            except Exception as e:
                log.error(f"[{vs.vehicle_id}] Trail güncelleme hatası: {e}")

    def _on_map_right_click(self, coords) -> None:
        lat, lon = coords
        self.add_waypoint(lat, lon)

    def _on_map_left_click(self, coords) -> None:
        if self._waypoint_mode:
            lat, lon = coords
            self.add_waypoint(lat, lon)

    @staticmethod
    def _haversine(lat1: float, lon1: float,
                   lat2: float, lon2: float) -> float:
        R    = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a    = (math.sin(dphi / 2) ** 2
                + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))