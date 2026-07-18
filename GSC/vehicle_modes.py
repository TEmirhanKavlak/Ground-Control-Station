"""
vehicle_modes.py
────────────────
Araç-tipine duyarlı ArduPilot uçuş/sürüş modu haritaları.

SORUN:
    ArduPilot'ta MAVLink `custom_mode` değerleri araç tipine (firmware)
    göre DEĞİŞİR. Örn:
        • Copter (İHA):  GUIDED = 4,  AUTO = 3,  RTL = 6
        • Rover  (İDA):  GUIDED = 15, AUTO = 10, RTL = 11, HOLD = 4

    Tek bir sabit harita kullanılırsa, İDA'ya GUIDED (Copter'da 4) gönderilir
    ve Rover bunu HOLD (Rover'da 4) olarak yorumlar. Mission Planner'da mod
    "HOLD" görünür. Bu modül bu eşleşmeyi araç tipine göre düzeltir.

KAYNAK (ArduPilot, master):
    Copter: ArduCopter/mode.h
    Rover:  Rover/Parameters.cpp / Rover/mode.h

Tüm gönderme (set_mode) ve görüntüleme (heartbeat parse) kodu mod
değerlerini BURADAN almalıdır. Hiçbir yerde sabit mod numarası kullanılmaz.
"""

from typing import Optional

# ─────────────────────────────────────────
#  FRAME (FIRMWARE) TİPLERİ
# ─────────────────────────────────────────
FRAME_COPTER = "copter"
FRAME_ROVER  = "rover"

# Araç kimliği → firmware tipi (uygulamanın açık yapılandırması)
#   "iha" = İHA  → ArduCopter
#   "ida" = İDA  → ArduRover (USV)
VEHICLE_FRAME: dict[str, str] = {
    "iha": FRAME_COPTER,
    "ida": FRAME_ROVER,
}

# Tanınmayan araçlar için varsayılan
DEFAULT_FRAME = FRAME_COPTER


# ─────────────────────────────────────────
#  COPTER (İHA) MOD HARİTASI  — name → custom_mode
# ─────────────────────────────────────────
COPTER_MODES: dict[str, int] = {
    "STABILIZE":     0,
    "ACRO":          1,
    "ALT_HOLD":      2,
    "AUTO":          3,
    "GUIDED":        4,
    "LOITER":        5,
    "RTL":           6,
    "CIRCLE":        7,
    "LAND":          9,
    "DRIFT":        11,
    "SPORT":        13,
    "FLIP":         14,
    "AUTOTUNE":     15,
    "POSHOLD":      16,
    "BRAKE":        17,
    "THROW":        18,
    "AVOID_ADSB":   19,
    "GUIDED_NOGPS": 20,
    "SMART_RTL":    21,
    "FLOWHOLD":     22,
    "FOLLOW":       23,
    "ZIGZAG":       24,
    "SYSTEMID":     25,
    "AUTOROTATE":   26,
    "AUTO_RTL":     27,
    "TURTLE":       28,
}

# ─────────────────────────────────────────
#  ROVER (İDA) MOD HARİTASI  — name → custom_mode
# ─────────────────────────────────────────
ROVER_MODES: dict[str, int] = {
    "MANUAL":        0,
    "ACRO":          1,
    "STEERING":      3,
    "HOLD":          4,
    "LOITER":        5,
    "FOLLOW":        6,
    "SIMPLE":        7,
    "DOCK":          8,
    "CIRCLE":        9,
    "AUTO":         10,
    "RTL":          11,
    "SMART_RTL":    12,
    "GUIDED":       15,
    "INITIALISING": 16,
}

# frame tipi → mod haritası
MODE_MAPS: dict[str, dict[str, int]] = {
    FRAME_COPTER: COPTER_MODES,
    FRAME_ROVER:  ROVER_MODES,
}

# Ters haritalar: custom_mode → name (görüntüleme için)
COPTER_ID_TO_NAME: dict[int, str] = {v: k for k, v in COPTER_MODES.items()}
ROVER_ID_TO_NAME:  dict[int, str] = {v: k for k, v in ROVER_MODES.items()}

ID_TO_NAME_MAPS: dict[str, dict[int, str]] = {
    FRAME_COPTER: COPTER_ID_TO_NAME,
    FRAME_ROVER:  ROVER_ID_TO_NAME,
}


# ─────────────────────────────────────────
#  MAV_TYPE → FRAME (otomatik algılama, sağlamlık katmanı)
# ─────────────────────────────────────────
#  HEARTBEAT.type alanından firmware tipini tahmin etmek için. Açık
#  VEHICLE_FRAME yapılandırması birincildir; bu yalnızca doğrulama/yedektir.
#  (mavutil.mavlink.MAV_TYPE_* sabitleri)
_ROVER_MAV_TYPES  = {10, 11}            # GROUND_ROVER, SURFACE_BOAT
_COPTER_MAV_TYPES = {2, 3, 4, 13, 14, 15}  # QUAD, COAX, HELI, HEXA, OCTO, TRICOPTER


def frame_from_mav_type(mav_type: Optional[int]) -> Optional[str]:
    """HEARTBEAT.type değerinden firmware tipini döndür (bilinmiyorsa None)."""
    if mav_type is None:
        return None
    if mav_type in _ROVER_MAV_TYPES:
        return FRAME_ROVER
    if mav_type in _COPTER_MAV_TYPES:
        return FRAME_COPTER
    return None


# ─────────────────────────────────────────
#  GENEL YARDIMCILAR
# ─────────────────────────────────────────
def frame_for_vehicle(vehicle_id: str) -> str:
    """Araç kimliğinden firmware tipini döndür."""
    return VEHICLE_FRAME.get(vehicle_id, DEFAULT_FRAME)


def mode_map_for_vehicle(vehicle_id: str) -> dict[str, int]:
    """Araca uygun name → custom_mode haritasını döndür."""
    return MODE_MAPS[frame_for_vehicle(vehicle_id)]


def mode_names_for_vehicle(vehicle_id: str) -> list[str]:
    """Araca uygun mod adlarının listesi (GUI combobox için)."""
    return list(mode_map_for_vehicle(vehicle_id).keys())


def mode_name_to_id(vehicle_id: str, mode_name: str) -> Optional[int]:
    """
    Araca göre mod adını custom_mode değerine çevir.
    Bilinmeyen mod → None.
    """
    return mode_map_for_vehicle(vehicle_id).get(mode_name.upper())


def mode_id_to_name(vehicle_id: str, mode_id: int,
                    frame: Optional[str] = None) -> str:
    """
    Araca göre custom_mode değerini okunabilir mod adına çevir.
    Bilinmeyen değer → "MODE_<id>".

    frame verilirse (ör. HEARTBEAT'ten otomatik algılanmışsa) onu kullanır;
    aksi halde vehicle_id'ye göre yapılandırılmış frame'i kullanır.
    """
    f = frame or frame_for_vehicle(vehicle_id)
    return ID_TO_NAME_MAPS.get(f, COPTER_ID_TO_NAME).get(mode_id, f"MODE_{mode_id}")
