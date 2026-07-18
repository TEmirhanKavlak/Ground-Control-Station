"""
communication_bridge.py
───────────────────────
İHA → İDA haberleşme köprüsü (modüler, thread-safe).

AMAÇ
────
Orijinal bağımsız script, İHA'dan gelen "RENK:<kod>" STATUSTEXT mesajlarını
okuyup aynısını İDA'ya iletiyordu. O script kendi BAĞIMSIZ seri bağlantılarını
açıyordu — bu, YKİ zaten aynı portlara bağlıyken çakışmaya yol açar
(tek seri porta iki ayrı okuyucu).

Bu modül aynı işi YKİ'nin MEVCUT bağlantılarını yeniden kullanarak yapar:

    [İHA VehicleConnection._recv_loop]  (mavlink okuma thread'i)
            │  publish("mavlink_msg_iha", {type, msg, ts, vehicle})
            ▼
        [EventBus]
            │  poll()  (GUI thread, her ~50ms)
            ▼
    CommunicationBridge._on_source_msg   → STATUSTEXT + "RENK:" filtrele
            │  queue.put(kod)             (GUI thread'i ASLA bloklamaz)
            ▼
    CommunicationBridge._worker          (kendi daemon thread'i)
            │  vmgr.get("ida").send_statustext("RENK:<kod>")
            ▼
        [İDA VehicleConnection]  → seri porta yaz

THREAD GÜVENLİĞİ
────────────────
• GUI thread (poll) yalnızca queue.Queue'ya yazar → bloklamaz, donmaz.
• Asıl seri gönderim ayrı bir daemon worker thread'inde yapılır.
• İDA'ya gönderim VehicleConnection.send_statustext() içinde bir kilitle
  (send lock) korunur; aynı bağlantıya yazan recv_loop ile çakışmaz.

Bu sınıf GUI'ye DOKUNMAZ — sadece EventBus üzerinden haberleşir.
"""

import queue
import threading
import logging
import time
from typing import Callable, Optional

log = logging.getLogger("bridge")


# ─────────────────────────────────────────
#  HANGİ VERİ AKTARILACAK? (açık tanım)
# ─────────────────────────────────────────
# İHA'dan İDA'ya YALNIZCA aşağıdaki kurala uyan STATUSTEXT iletilir:
#   • Mesaj tipi : "STATUSTEXT"
#   • Metin önek : "RENK:"   (örn. "RENK:1")
# Bu önek ile başlamayan hiçbir mesaj köprüden geçmez.
DEFAULT_SOURCE       = "iha"
DEFAULT_TARGET       = "ida"
DEFAULT_PREFIX       = "RENK:"
DEFAULT_READY_PROMPT = "RENK BEKLENİYOR"

# Kod → okunabilir renk (sadece loglama/insan içindir, gönderime etkisi yok)
RENKLER: dict[str, str] = {
    "0": "KIRMIZI",
    "1": "YEŞİL",
    "2": "SİYAH",
}


class CommunicationBridge:
    """
    İHA → İDA tek yönlü STATUSTEXT köprüsü.

    Parametreler
    ────────────
    event_bus          : Uygulamanın EventBus örneği.
    vehicle_manager    : VehicleManager örneği (İHA + İDA bağlantılarını tutar).
    source             : Kaynak araç kimliği (varsayılan "iha").
    target             : Hedef araç kimliği (varsayılan "ida").
    prefix             : İletilecek STATUSTEXT öneki (varsayılan "RENK:").
    color_map          : Kod → renk adı eşlemesi (loglama için).
    ready_prompt       : Kaynak bağlandığında ona gönderilecek bilgi metni
                         (orijinal script'teki "RENK BEKLENİYOR"). None → gönderilmez.
    forward_duplicates : True  → her eşleşen mesaj iletilir (orijinal davranış).
                         False → arka arkaya aynı kod tekrar gelirse iletilmez.
    """

    def __init__(
        self,
        event_bus,
        vehicle_manager,
        source: str = DEFAULT_SOURCE,
        target: str = DEFAULT_TARGET,
        prefix: str = DEFAULT_PREFIX,
        color_map: Optional[dict] = None,
        ready_prompt: Optional[str] = DEFAULT_READY_PROMPT,
        forward_duplicates: bool = True,
        start_enabled: bool = False,
    ):
        self.bus     = event_bus
        self.vmgr    = vehicle_manager
        self.source  = source
        self.target  = target
        self.prefix  = prefix
        self.colors  = color_map if color_map is not None else dict(RENKLER)
        self.ready_prompt       = ready_prompt
        self.forward_duplicates = forward_duplicates

        # Görev durumu: GUI'deki butonla aç/kapa yapılır.
        # Pasifken köprü abone kalır ama hiçbir veri İDA'ya iletilmez.
        self._enabled = start_enabled

        # GUI thread → worker thread arası güvenli aktarım kuyruğu
        self._queue: queue.Queue = queue.Queue()
        self._worker_thread: Optional[threading.Thread] = None
        self._running = False

        # Tekrar bastırma için son iletilen kod
        self._last_code: Optional[str] = None
        # ready_prompt'un kaynak bağlantısı başına yalnızca bir kez gitmesi için
        self._ready_sent = False
        # Kaynak (İHA) o an bağlı mı? (enable anında ready göndermek için)
        self._source_connected = False

        # İstatistikler (GUI'de göstermek isteyenler için)
        self.stats = {
            "received":  0,   # önek eşleşen toplam mesaj
            "forwarded": 0,   # İDA'ya başarıyla iletilen
            "skipped":   0,   # tekrar olduğu için atlanan
            "errors":    0,   # gönderim hatası
        }

    # ──────────────────────────────────────
    #  YAŞAM DÖNGÜSÜ
    # ──────────────────────────────────────

    def start(self) -> None:
        """Köprüyü başlat: event'lere abone ol + worker thread'i çalıştır."""
        if self._running:
            log.warning("CommunicationBridge zaten çalışıyor")
            return

        self._running = True

        # 1) Kaynak araçtan gelen ham MAVLink mesajlarını dinle (GUI thread'inde)
        self.bus.subscribe(f"mavlink_msg_{self.source}", self._on_source_msg)
        # 2) Kaynak bağlandığında "RENK BEKLENİYOR" yollayabilmek için durumu dinle
        self.bus.subscribe("conn_state", self._on_conn_state)

        # 3) Asıl gönderimi yapan arka plan thread'i
        self._worker_thread = threading.Thread(
            target=self._worker,
            daemon=True,
            name="comm-bridge-worker",
        )
        self._worker_thread.start()

        log.info(
            "CommunicationBridge yüklendi (PASİF): %s → %s  (önek='%s')",
            self.source, self.target, self.prefix,
        )
        # NOT: Burada köprü yalnızca EventBus'a abone olur; bağlantı KURMAZ,
        # veri İLETMEZ. Görev, arayüzdeki butonla enable() çağrılınca başlar.
        durum = "AKTİF" if self._enabled else "PASİF — görev butonu bekleniyor"
        self._publish_bridge_log(
            f"Köprü hazır ({durum}): {self.source.upper()} → {self.target.upper()}",
            "ok" if self._enabled else "dim",
        )

    def stop(self) -> None:
        """Köprüyü durdur (abonelikleri kaldır, worker'ı sonlandır)."""
        if not self._running:
            return
        self._running = False

        # Aboneliklerden çık (var olmayan abone hatasını EventBus zaten yutuyor)
        try:
            self.bus.unsubscribe(f"mavlink_msg_{self.source}", self._on_source_msg)
            self.bus.unsubscribe("conn_state", self._on_conn_state)
        except Exception:
            pass

        # Worker'ı uyandırmak için sentinel koy
        self._queue.put_nowait(None)
        log.info("CommunicationBridge durduruldu")

    # ──────────────────────────────────────
    #  GÖREV AÇ / KAPA  (GUI butonu buraya bağlanır)
    # ──────────────────────────────────────

    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        """Görevi aktif et: bundan sonra eşleşen mesajlar İDA'ya iletilir."""
        if self._enabled:
            return
        self._enabled = True
        log.info("Görev köprüsü AKTİF edildi")
        self._publish_bridge_log("Görev köprüsü AKTİF", "ok")
        # Kaynak zaten bağlıysa 'RENK BEKLENİYOR' bilgisini şimdi gönder
        self._maybe_send_ready()

    def disable(self) -> None:
        """Görevi durdur: köprü abone kalır ama hiçbir veri iletmez."""
        if not self._enabled:
            return
        self._enabled = False
        log.info("Görev köprüsü PASİF edildi")
        self._publish_bridge_log("Görev köprüsü PASİF", "warn")

    def toggle(self) -> bool:
        """Durumu ters çevir; yeni durumu (True=aktif) döndür."""
        if self._enabled:
            self.disable()
        else:
            self.enable()
        return self._enabled

    # ──────────────────────────────────────
    #  KAYNAK MESAJ HANDLER'I  (GUI thread)
    # ──────────────────────────────────────

    def _on_source_msg(self, data: dict) -> None:
        """
        EventBus.poll() içinde (GUI thread) çağrılır.
        Burada YALNIZCA filtreleme + kuyruğa atma yapılır; ASLA seri I/O yapılmaz.
        Böylece arayüz donmaz.
        """
        if not isinstance(data, dict):
            return
        # Görev pasifse hiçbir şey iletme
        if not self._enabled:
            return
        if data.get("type") != "STATUSTEXT":
            return

        msg = data.get("msg")
        if msg is None:
            return

        try:
            text = (msg.text or "").strip()
        except Exception as e:
            log.error("STATUSTEXT metni okunamadı: %s", e)
            return

        # Sadece önekimizle başlayan mesajları köprüden geçir
        if not text.startswith(self.prefix):
            return

        # "RENK:1" → "1"
        kod = text[len(self.prefix):].strip()
        if not kod:
            log.warning("Boş renk kodu, atlanıyor: %r", text)
            return

        self.stats["received"] += 1

        renk_adi = self.colors.get(kod, "Bilinmiyor")
        log.info("[%s] Renk alındı: %s → %s", self.source, kod, renk_adi)
        self._publish_bridge_log(
            f"{self.source.upper()}'dan renk alındı: {kod} ({renk_adi})", "info"
        )

        # Gönderimi worker'a devret (bloklamadan)
        self._queue.put_nowait(kod)

    # ──────────────────────────────────────
    #  GÖNDERİM WORKER'I  (ayrı daemon thread)
    # ──────────────────────────────────────

    def _worker(self) -> None:
        """Kuyruğu boşaltır ve renk kodlarını İDA'ya gönderir."""
        while self._running:
            try:
                kod = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if kod is None:          # stop() sentinel'i
                break

            # Tekrar bastırma
            if not self.forward_duplicates and kod == self._last_code:
                self.stats["skipped"] += 1
                log.debug("Aynı kod tekrar geldi, atlanıyor: %s", kod)
                continue

            self._forward_to_target(kod)

    def _forward_to_target(self, kod: str) -> None:
        """Tek bir renk kodunu hedef araca (İDA) STATUSTEXT olarak gönder."""
        renk_adi = self.colors.get(kod, "Bilinmiyor")
        payload  = f"{self.prefix}{kod}"   # "RENK:1"

        target = self.vmgr.get(self.target)
        if target is None:
            self.stats["errors"] += 1
            log.error("Hedef araç bulunamadı: %s", self.target)
            self._publish_bridge_log(
                f"HATA: '{self.target}' aracı tanımlı değil", "error"
            )
            return

        if not target.is_connected():
            self.stats["errors"] += 1
            log.warning("[%s] bağlı değil, '%s' iletilemedi", self.target, payload)
            self._publish_bridge_log(
                f"{self.target.upper()} bağlı değil — '{payload}' iletilemedi",
                "warn",
            )
            return

        try:
            ok = target.send_statustext(payload)
            if ok:
                self.stats["forwarded"] += 1
                self._last_code = kod
                log.info("[bridge] Gönderildi: %s → %s (%s)",
                         payload, self.target, renk_adi)
                self._publish_bridge_log(
                    f"İletildi → {self.target.upper()}: {payload} ({renk_adi})",
                    "ok",
                )
            else:
                self.stats["errors"] += 1
                self._publish_bridge_log(
                    f"{self.target.upper()}'ya gönderim başarısız: {payload}",
                    "error",
                )
        except Exception as e:
            self.stats["errors"] += 1
            log.error("[bridge] Gönderim hatası: %s", e, exc_info=True)
            self._publish_bridge_log(
                f"Gönderim hatası ({payload}): {e}", "error"
            )

    # ──────────────────────────────────────
    #  KAYNAK BAĞLANDIĞINDA "HAZIR" METNİ
    # ──────────────────────────────────────

    def _on_conn_state(self, data: dict) -> None:
        """
        Orijinal script açılışta İHA'ya 'RENK BEKLENİYOR' yolluyordu.
        Aynı davranışı, İHA bağlı VE görev aktifken bir kez tekrarlıyoruz.
        """
        if not isinstance(data, dict) or data.get("vehicle") != self.source:
            return

        state = data.get("state")
        state_name = getattr(state, "name", str(state))

        if state_name == "CONNECTED":
            self._source_connected = True
            self._maybe_send_ready()
        else:
            # Bağlantı düştü → bir sonraki CONNECTED'da ready tekrar gönderilsin
            self._source_connected = False
            self._ready_sent = False

    def _maybe_send_ready(self) -> None:
        """Koşullar uygunsa kaynağa 'RENK BEKLENİYOR' gönder (tek sefer)."""
        if not self.ready_prompt or not self._enabled:
            return
        if not self._source_connected or self._ready_sent:
            return

        src = self.vmgr.get(self.source)
        if src and src.is_connected():
            try:
                src.send_statustext(self.ready_prompt)
                self._ready_sent = True
                log.info("[%s] '%s' gönderildi", self.source, self.ready_prompt)
                self._publish_bridge_log(
                    f"{self.source.upper()}'ya '{self.ready_prompt}' gönderildi",
                    "dim",
                )
            except Exception as e:
                log.error("Ready prompt gönderilemedi: %s", e)

    # ──────────────────────────────────────
    #  YARDIMCI
    # ──────────────────────────────────────

    def _publish_bridge_log(self, msg: str, level: str = "info") -> None:
        """
        Köprü olaylarını hem özel 'bridge' event'i hem de standart 'log' event'i
        olarak yayınlar. 'log' sayesinde mevcut LogPanel'de otomatik görünür
        (GUI'de hiçbir değişiklik gerektirmez). Hedef aracın log sekmesine düşer.
        """
        self.bus.publish("bridge", {"msg": msg, "level": level})
        self.bus.publish("log", {
            "msg":     f"[KÖPRÜ] {msg}",
            "level":   level,
            "vehicle": self.target,
        })