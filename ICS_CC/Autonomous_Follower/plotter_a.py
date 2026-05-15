"""
Telemetry plotter for Autonomous Vehicle Cascade Controller.
Reads from path_reciever.ino over serial and plots live.

    pip install pyserial matplotlib

Change PORT to match your receiver Arduino (Mac: ls /dev/tty.usb*)

CSV format from path_reciever.ino:
  currentVel, desiredPos, currentX, currentY, velocityCommand, pwmCommand, heading
"""

import csv
import signal
import sys
import time
import threading
import serial
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ─── Config ───────────────────────────────────────────────────────────────────
PORT        = '/dev/cu.usbmodem1051DB35C79C2'   # <-- change this
BAUD        = 115200
WINDOW_SECS = 30      # seconds of history shown on time-series charts

# Keep in sync with pathX/pathY in follower.ino
WP_X = [0.0, 2.0, 4.0, 6.0, 8.0]
WP_Y = [0.0, 0.0, 2.0, 0.0, 2.0]

# ─── CSV parser ───────────────────────────────────────────────────────────────
# path_reciever.ino prints:
#   currentVel, desiredPos, currentX, currentY, velocityCommand, pwmCommand, heading
def parse_line(line):
    parts = line.strip().split(',')
    if len(parts) != 7:
        return None
    try:
        return {
            'vel':    float(parts[0]),
            'dpos':   float(parts[1]),
            'x':      float(parts[2]),
            'y':      float(parts[3]),
            'velcmd': float(parts[4]),
            'pwmcmd': float(parts[5]),
            'heading': float(parts[6]),
        }
    except ValueError:
        return None

# ─── CSV output ───────────────────────────────────────────────────────────────
CSV_FILE = 'telemetry.csv'
_csv_file   = open(CSV_FILE, 'w', newline='')
_csv_writer = csv.writer(_csv_file)
_csv_writer.writerow(['time_s', 'velocity', 'velocity_command', 'pwm_command',
                      'heading', 'desired_pos', 'x', 'y'])

# ─── Thread-safe data store ───────────────────────────────────────────────────
_lock = threading.Lock()
_data = []
_t0   = time.time()

def _reader():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {PORT}: {e}")
        return
    print(f"[OK] Connected to {PORT}")
    print(f"[OK] Logging to {CSV_FILE}")
    while True:
        try:
            raw = ser.readline().decode(errors='replace').strip()
        except Exception:
            continue
        if not raw:
            continue
        pkt = parse_line(raw)
        if pkt is None:
            print(f"[INFO] {raw}")
            continue
        pkt['t'] = time.time() - _t0

        print(f"t={pkt['t']:7.2f}s  vel={pkt['vel']:6.3f} m/s  "
              f"pwmCmd={pkt['pwmcmd']:6.1f}  heading={pkt['heading']:6.1f}°")

        _csv_writer.writerow([
            f"{pkt['t']:.3f}", pkt['vel'], pkt['velcmd'], pkt['pwmcmd'],
            pkt['heading'], pkt['dpos'], pkt['x'], pkt['y'],
        ])
        _csv_file.flush()

        with _lock:
            _data.append(pkt)

threading.Thread(target=_reader, daemon=True).start()

# ─── Active waypoint (best match between dpos and distance to each waypoint) ──
def active_waypoint(x, y, dpos):
    best_i, best_diff = 0, float('inf')
    for i, (wx, wy) in enumerate(zip(WP_X, WP_Y)):
        diff = abs(np.sqrt((wx - x)**2 + (wy - y)**2) - dpos)
        if diff < best_diff:
            best_diff, best_i = diff, i
    return best_i

# ─── Figure layout (2×2) ──────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 10))
fig.suptitle('Autonomous Vehicle Telemetry', fontweight='bold', fontsize=13)

# ── Panel 1: Position map ─────────────────────────────────────────────────────
ax_map = fig.add_subplot(2, 2, 1)
ax_map.set_title('Position Map')
ax_map.set_xlabel('X (m)')
ax_map.set_ylabel('Y (m)')
ax_map.set_aspect('equal', adjustable='datalim')
ax_map.grid(True, alpha=0.3)

ax_map.plot(WP_X, WP_Y, color='salmon', linestyle='--', linewidth=1, zorder=2)
ax_map.plot(WP_X, WP_Y, 'ro', markersize=9, zorder=3)
for i, (wx, wy) in enumerate(zip(WP_X, WP_Y)):
    ax_map.annotate(f'P{i}', (wx, wy), xytext=(5, 5),
                    textcoords='offset points', fontsize=8, color='darkred')

trail_line,  = ax_map.plot([], [], color='steelblue', linewidth=1.5, zorder=4)
pos_dot,     = ax_map.plot([], [], 'o', color='steelblue', markersize=9, zorder=5)
target_dot,  = ax_map.plot([], [], '*', color='gold', markersize=16,
                            markeredgecolor='darkorange', markeredgewidth=1.2, zorder=6)
target_line, = ax_map.plot([], [], color='gold', linestyle=':', linewidth=1.5, zorder=4)
heading_arrow = ax_map.annotate('', xy=(0, 0), xytext=(0, 0),
    arrowprops=dict(arrowstyle='->', color='limegreen', lw=2.0), zorder=7)

ax_map.legend(handles=[
    mpatches.Patch(color='steelblue', label='Path taken'),
    plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='gold',
               markersize=12, markeredgecolor='darkorange', label='Active target'),
    mpatches.Patch(color='salmon', label='Waypoints'),
    plt.Line2D([0], [0], color='limegreen', lw=2, label='Heading'),
], fontsize=7, loc='upper left')

# ── Panel 2: Velocity ─────────────────────────────────────────────────────────
ax_vel = fig.add_subplot(2, 2, 2)
ax_vel.set_title('Velocity vs Time')
ax_vel.set_xlabel('Time (s)')
ax_vel.set_ylabel('m/s')
ax_vel.grid(True, alpha=0.3)

vel_line,    = ax_vel.plot([], [], color='limegreen', linewidth=1.8, label='Current vel')
velcmd_line, = ax_vel.plot([], [], color='orange', linewidth=1.8, linestyle='--',
                           label='Velocity command')
dpos_line,   = ax_vel.plot([], [], color='steelblue', linewidth=1.2, linestyle=':',
                           label='Dist to target (m)')
ax_vel.legend(fontsize=8)

# ── Panel 3: PWM ──────────────────────────────────────────────────────────────
ax_pwm = fig.add_subplot(2, 2, 3)
ax_pwm.set_title('PWM vs Time')
ax_pwm.set_xlabel('Time (s)')
ax_pwm.set_ylabel('PWM')
ax_pwm.grid(True, alpha=0.3)

pwmcmd_line, = ax_pwm.plot([], [], color='tomato', linewidth=1.8, label='PWM command')
ax_pwm.legend(fontsize=8)

# ── Panel 4: Heading ──────────────────────────────────────────────────────────
ax_head = fig.add_subplot(2, 2, 4)
ax_head.set_title('Heading vs Time')
ax_head.set_xlabel('Time (s)')
ax_head.set_ylabel('Degrees (°)')
ax_head.set_ylim(-10, 370)
ax_head.set_yticks([0, 90, 180, 270, 360])
ax_head.grid(True, alpha=0.3)

head_line, = ax_head.plot([], [], color='mediumorchid', linewidth=1.8, label='Heading')
ax_head.legend(fontsize=8)

plt.tight_layout()

# ─── Update callback ──────────────────────────────────────────────────────────
ARROW_LEN = 0.4

def update():
    with _lock:
        snap = list(_data)

    if not snap:
        return

    xs      = [d['x']       for d in snap]
    ys      = [d['y']       for d in snap]
    ts      = [d['t']       for d in snap]
    vel     = [d['vel']     for d in snap]
    dpos    = [d['dpos']    for d in snap]
    velcmd  = [d['velcmd']  for d in snap]
    pwmcmd  = [d['pwmcmd']  for d in snap]
    heading = [d['heading'] for d in snap]

    cx, cy   = xs[-1], ys[-1]
    t_end    = ts[-1]
    h_rad    = np.radians(heading[-1])

    # ── Map ──────────────────────────────────────────────────────────────────
    trail_line.set_data(xs, ys)
    pos_dot.set_data([cx], [cy])

    ti = active_waypoint(cx, cy, dpos[-1])
    tx, ty = WP_X[ti], WP_Y[ti]
    target_dot.set_data([tx], [ty])
    target_line.set_data([cx, tx], [cy, ty])

    # Heading arrow: BNO055 0°=North → x=sin(h), y=cos(h)
    ax_tip = cx + ARROW_LEN * np.sin(h_rad)
    ay_tip = cy + ARROW_LEN * np.cos(h_rad)
    heading_arrow.set_position((cx, cy))
    heading_arrow.xy = (ax_tip, ay_tip)

    pad = 1.2
    ax_map.set_xlim(min(xs + WP_X) - pad, max(xs + WP_X) + pad)
    ax_map.set_ylim(min(ys + WP_Y) - pad, max(ys + WP_Y) + pad)

    # ── Velocity ──────────────────────────────────────────────────────────────
    vel_line.set_data(ts, vel)
    velcmd_line.set_data(ts, velcmd)
    dpos_line.set_data(ts, dpos)
    ax_vel.set_xlim(max(0.0, t_end - WINDOW_SECS), t_end + 0.5)
    all_vel = vel + velcmd + dpos
    if all_vel:
        lo, hi = min(all_vel), max(all_vel)
        ax_vel.set_ylim(lo - 0.1, hi + 0.1)

    # ── PWM ───────────────────────────────────────────────────────────────────
    pwmcmd_line.set_data(ts, pwmcmd)
    ax_pwm.set_xlim(max(0.0, t_end - WINDOW_SECS), t_end + 0.5)
    if pwmcmd:
        lo, hi = min(pwmcmd), max(pwmcmd)
        ax_pwm.set_ylim(lo - 1, hi + 1)

    # ── Heading ───────────────────────────────────────────────────────────────
    head_line.set_data(ts, heading)
    ax_head.set_xlim(max(0.0, t_end - WINDOW_SECS), t_end + 0.5)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# ─── Clean shutdown ───────────────────────────────────────────────────────────
def _shutdown(sig=None, frame=None):
    print("\nStopped.")
    _csv_file.close()
    plt.close('all')
    sys.exit(0)

signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

# ─── Main loop ────────────────────────────────────────────────────────────────
plt.ion()
plt.show()

while True:
    update()
    plt.pause(0.1)
