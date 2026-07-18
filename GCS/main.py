"""
 
"""

import logging
import sys
import tkinter as tk

# ─── Logging konfigürasyonu ───────────────
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("gcs.log", encoding="utf-8"),
    ],
)
log = logging.getLogger("main")

# ─── Modüller ─────────────────────────────
from event_bus       import EventBus
from mavlink_manager import VehicleManager
from telemetry       import TelemetryManager
from map_controller  import MapController
from gui             import GCSApplication
from communication_bridge import CommunicationBridge


def main():
    log.info("YKİ GCS başlatılıyor...")

    # ── 1. EventBus ───────────────────────
    bus = EventBus()

    # ── 2. VehicleManager (İHA + İDA) ─────
    vmgr = VehicleManager(event_bus=bus)

    # ── 3. TelemetryManager ───────────────
    # Her araç için bağımsız parser
    _telem = TelemetryManager(event_bus=bus)   # noqa: F841

    # ── 3b. CommunicationBridge (İHA → İDA) ─
    # İHA'dan gelen "RENK:<kod>" STATUSTEXT mesajlarını İDA'ya iletir.
    # Mevcut bağlantıları yeniden kullanır; yeni seri port AÇMAZ.
    bridge = CommunicationBridge(
        event_bus=bus,
        vehicle_manager=vmgr,
        source="iha",
        target="ida",
        prefix="RENK:",
        start_enabled=False,   # Görev arayüzdeki butonla başlatılır
    )
    bridge.start()

    # ── 4. Tkinter root ───────────────────
    root = tk.Tk()

    # ── 5. GCS GUI ────────────────────────
    app = GCSApplication(root=root, event_bus=bus,
                         vehicle_manager=vmgr, bridge=bridge)

    # ── 6. MapController ──────────────────
    map_ctrl = MapController(
        parent_frame=app.map_frame,
        event_bus=bus,
        vehicle_manager=vmgr,
        on_waypoint_add=app.on_waypoint_added,
    )
    app.set_map_controller(map_ctrl)

    # ── 7. Mainloop ───────────────────────
    log.info("Tkinter mainloop başlatılıyor")
    root.mainloop()

    # ── 8. Temiz kapanış ──────────────────
    bridge.stop()
    log.info("Uygulama kapatıldı")


if __name__ == "__main__":
    main()