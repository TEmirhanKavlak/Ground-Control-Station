"""
constants.py
────────────
Uygulama genelinde kullanılan renk, font ve sabit değerler.
Tüm UI bileşenleri buradan import eder — hiçbir yerde inline renk yok.
"""

# ─────────────────────────────────────────
#  RENK PALETİ
# ─────────────────────────────────────────
BG        = "#0a0c12"
PANEL_BG  = "#10131c"
PANEL_BDR = "#1a1f2e"
ACCENT    = "#00e5ff"      # Cyan  — birincil vurgu
ACCENT2   = "#db2d61"      # Kırmızı — hata / uyarı
OK        = "#00c896"      # Yeşil — başarı / bağlı
WARN      = "#f6ad55"      # Sarı — uyarı / bağlanıyor
TEXT      = "#dde3f0"
TEXT_DIM  = "#3d4a60"
COORD_BG  = "#080a0f"
BTN_H     = "#1e2640"

# Araç renkleri
IHA_COLOR = "#00e5ff"      # Cyan
IDA_COLOR = "#f6ad55"      # Turuncu

# ─────────────────────────────────────────
#  FONTLAR
# ─────────────────────────────────────────
FONTS: dict[str, tuple] = {
    "title"  : ("Courier New", 13, "bold"),
    "panel"  : ("Courier New", 8,  "bold"),
    "mono"   : ("Courier New", 10),
    "mono_lg": ("Courier New", 13, "bold"),
    "mono_sm": ("Courier New", 8),
    "value"  : ("Courier New", 14, "bold"),
    "label"  : ("Courier New", 8),
    "seg"    : ("Courier New", 9, "bold"),
}

# ─────────────────────────────────────────
#  ARAÇ KİMLİKLERİ
# ─────────────────────────────────────────
VEHICLE_IDS    = ("ida", "iha")
VEHICLE_LABELS = {
    "iha": "İHA",
    "ida": "İDA",
}
VEHICLE_COLORS = {
    "iha": IHA_COLOR,
    "ida": IDA_COLOR,
}

# ─────────────────────────────────────────
#  BAĞLANTI VARSAYILANLARI
# ─────────────────────────────────────────
DEFAULT_BAUD = 57600
BAUD_OPTIONS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

# ─────────────────────────────────────────
#  GUI AYARLARI
# ─────────────────────────────────────────
POLL_INTERVAL_MS = 50
WINDOW_TITLE     = "YKİ — Yer Kontrol İstasyonu  |  ATÜ YGM KAAN ERTUĞRUL TAKIMI"
WINDOW_GEOMETRY  = "1500x860"
WINDOW_MIN       = (1200, 700)
APP_VERSION      = "4.1.0"


def lighten(hex_color: str, amount: int = 25) -> str:
    """Bir hex rengi belirtilen miktarda açar."""
    c = hex_color.lstrip("#")
    r, g, b = int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)
    return f"#{min(255, r+amount):02x}{min(255, g+amount):02x}{min(255, b+amount):02x}"