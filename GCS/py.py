from pymavlink import mavutil
import matplotlib.pyplot as plt
import math
import time

print("Bağlanılıyor: tcp:127.0.0.1:5762 ...")
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
master.wait_heartbeat()
print(f"Bağlantı kuruldu (sistem: {master.target_system})")

# --- POSITION_TARGET_GLOBAL_INT mesajını 10 Hz akıtmasını iste ---
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,
    100000,  # 100000 us = 10 Hz
    0, 0, 0, 0, 0
)
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print("Mesaj akışı talebi:", ack)

# --- AUTO moduna geç ---
mode_id = master.mode_mapping()['AUTO']
master.mav.set_mode_send(master.target_system,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print("AUTO moduna geçildi.\n")

times, actual_speeds, target_speeds = [], [], []
start_time = time.time()

print("Veri toplanıyor (Ctrl+C ile durdurup grafiği çizer)...\n")
try:
    while True:
        # Anlık hız
        msg_hud = master.recv_match(type='VFR_HUD', blocking=False)
        # Otopilotun o anki hedef hız vektörü
        msg_target = master.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=False)

        if msg_target is not None:
            # vx, vy: cm/s değil, doğrudan m/s (ArduPilot bu mesajda m/s kullanır)
            target_speed = math.hypot(msg_target.vx, msg_target.vy)
        else:
            target_speed = None

        if msg_hud is not None and target_speed is not None:
            t = time.time() - start_time
            times.append(t)
            actual_speeds.append(msg_hud.groundspeed)
            target_speeds.append(target_speed)
            print(f"t={t:5.1f}s | Anlık: {msg_hud.groundspeed:.2f} m/s | "
                  f"MAVLink Hedef: {target_speed:.2f} m/s")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nDurduruldu, grafik çiziliyor...")

plt.figure(figsize=(10, 5))
plt.plot(times, actual_speeds, label='Anlık Hız', linewidth=1.5)
plt.plot(times, target_speeds, label='Hedef Hız (MAVLink)', linewidth=2, linestyle='--')
plt.xlabel('Zaman (s)')
plt.ylabel('Hız (m/s)')
plt.title('Anlık Hız vs MAVLink Hedef Hız')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('hiz_grafigi.png')
plt.show()