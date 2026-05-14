"""
Telemetry plotter for Autonomous Vehicle Cascade Controller.
Reads from path_reciever.ino over serial and plots live.

    pip install pyserial matplotlib

Change PORT to match your receiver Arduino (Mac: ls /dev/tty.usb*)
"""

import time
import threading
import serial
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ─── Config ───────────────────────────────────────────────────────────────────
PORT        = '/dev/cu.usbmodem1051DB35C79C2'   # <-- change this
BAUD        = 115200
WINDOW_SECS = 30      # seconds of history shown on charts
ARROW_LEN   = 0.4     # heading arrow length (metres)

# Keep in sync with pathX/pathY in follower.ino (include start position)
WP_X = [0.0, 2.0, 4.0, 6.0, 8.0]
WP_Y = [0.0, 2.0, 0.0, 2.0, 0.0]

# ─── CSV parser (matches path_reciever.ino output: cpos,dpos,x,y,des_h,cur_h) ─
def parse_line(line):
    parts = line.strip().split(',')
    if len(parts) != 6:
        return None
    try:
        return {
            'cpos':  float(parts[0]),
            'dpos':  float(parts[1]),
            'x':     float(parts[2]),
            'y':     float(parts[3]),
            'des_h': float(parts[4]),
            'cur_h': float(parts[5]),
        }
    except ValueError:
        return None

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
        with _lock:
            _data.append(pkt)

threading.Thread(target=_reader, daemon=True).start()

# ─── Identify active waypoint from dpos ───────────────────────────────────────
def active_waypoint(x, y, dpos):
    """Return the index of the waypoint whose distance from (x,y) best matches dpos."""
    best_i, best_diff = 0, float('inf')
    for i, (wx, wy) in enumerate(zip(WP_X, WP_Y)):
        diff = abs(np.sqrt((wx - x)**2 + (wy - y)**2) - dpos)
        if diff < best_diff:
            best_diff, best_i = diff, i
    return best_i

# ─── Figure layout (1 row × 3 columns) ───────────────────────────────────────
fig = plt.figure(figsize=(18, 6))
fig.suptitle('Autonomous Vehicle Telemetry', fontweight='bold', fontsize=13)

ax_map = fig.add_subplot(1, 3, 1)
ax_map.set_title('Position Map')
ax_map.set_xlabel('X (m)')
ax_map.set_ylabel('Y (m)')
ax_map.set_aspect('equal', adjustable='datalim')
ax_map.grid(True, alpha=0.3)

# Waypoint path line and labels
ax_map.plot(WP_X, WP_Y, color='salmon', linestyle='--', linewidth=1, zorder=2)
ax_map.plot(WP_X, WP_Y, 'ro', markersize=9, zorder=3)
for i, (wx, wy) in enumerate(zip(WP_X, WP_Y)):
    ax_map.annotate(f'P{i}', (wx, wy), xytext=(5, 5),
                    textcoords='offset points', fontsize=8, color='darkred')

# Live map artists
trail_line,   = ax_map.plot([], [], color='steelblue', linewidth=1.5, zorder=4)
pos_dot,      = ax_map.plot([], [], 'o', color='steelblue', markersize=9, zorder=5)
target_dot,   = ax_map.plot([], [], '*', color='gold', markersize=16,
                             markeredgecolor='darkorange', markeredgewidth=1.2,
                             zorder=6, label='Active target')
target_line,  = ax_map.plot([], [], color='gold', linestyle=':', linewidth=1.5,
                             zorder=4)

legend_patches = [
    mpatches.Patch(color='limegreen',  label='Current heading'),
    mpatches.Patch(color='orange',     label='Desired heading'),
    mpatches.Patch(color='steelblue',  label='Path taken'),
    plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='gold',
               markersize=12, markeredgecolor='darkorange', label='Active target'),
    mpatches.Patch(color='salmon',     label='Waypoints'),
]
ax_map.legend(handles=legend_patches, fontsize=7, loc='upper left')

# ─── Heading chart ────────────────────────────────────────────────────────────
ax_hdg = fig.add_subplot(1, 3, 2)
ax_hdg.set_title('Heading vs Time')
ax_hdg.set_xlabel('Time (s)')
ax_hdg.set_ylabel('Degrees')
ax_hdg.set_ylim(-185, 370)
ax_hdg.set_yticks(range(0, 361, 45))
ax_hdg.axhline(0,   color='grey', linewidth=0.5, linestyle=':')
ax_hdg.axhline(360, color='grey', linewidth=0.5, linestyle=':')
ax_hdg.grid(True, alpha=0.3)

cur_line, = ax_hdg.plot([], [], color='limegreen', linewidth=1.8, label='Current')
des_line, = ax_hdg.plot([], [], color='orange',    linewidth=1.8, linestyle='--',
                        label='Desired')
err_line, = ax_hdg.plot([], [], color='red',       linewidth=1.2, linestyle=':',
                        label='Error')
ax_hdg.legend(fontsize=8)

# ─── Position chart ───────────────────────────────────────────────────────────
ax_pos = fig.add_subplot(1, 3, 3)
ax_pos.set_title('Distance to Target vs Time')
ax_pos.set_xlabel('Time (s)')
ax_pos.set_ylabel('Metres')
ax_pos.grid(True, alpha=0.3)

dpos_line, = ax_pos.plot([], [], color='orange',   linewidth=1.8, linestyle='--',
                         label='Desired dist (to target)')
cpos_line, = ax_pos.plot([], [], color='steelblue', linewidth=1.8,
                         label='Travelled dist (segment)')
ax_pos.legend(fontsize=8)

plt.tight_layout()

# ─── Helpers ─────────────────────────────────────────────────────────────────
_arrows = []

def _hdg_to_dxdy(deg):
    r = np.radians(deg)
    return ARROW_LEN * np.cos(r), ARROW_LEN * np.sin(r)

def _wrap180(vals):
    return ((np.array(vals) + 180) % 360) - 180

# ─── Update callback ──────────────────────────────────────────────────────────
def update():
    global _arrows

    with _lock:
        snap = list(_data)

    if not snap:
        return

    xs   = [d['x']     for d in snap]
    ys   = [d['y']     for d in snap]
    ts   = [d['t']     for d in snap]
    ch   = [d['cur_h'] for d in snap]
    dh   = [d['des_h'] for d in snap]
    dpos = [d['dpos']  for d in snap]
    cpos = [d['cpos']  for d in snap]

    cx, cy = xs[-1], ys[-1]

    # ── Map ──────────────────────────────────────────────────────────────────
    trail_line.set_data(xs, ys)
    pos_dot.set_data([cx], [cy])

    # Active target waypoint
    ti = active_waypoint(cx, cy, dpos[-1])
    tx, ty = WP_X[ti], WP_Y[ti]
    target_dot.set_data([tx], [ty])
    target_line.set_data([cx, tx], [cy, ty])

    # Heading arrows — rebuild each frame
    for a in _arrows:
        a.remove()
    _arrows = []
    kw = dict(xycoords='data', textcoords='data',
              annotation_clip=False, fontsize=0)
    cu, cv = _hdg_to_dxdy(ch[-1])
    du, dv = _hdg_to_dxdy(dh[-1])
    _arrows.append(ax_map.annotate(
        '', xy=(cx + cu, cy + cv), xytext=(cx, cy),
        arrowprops=dict(arrowstyle='->', color='limegreen', lw=2.5), **kw))
    _arrows.append(ax_map.annotate(
        '', xy=(cx + du, cy + dv), xytext=(cx, cy),
        arrowprops=dict(arrowstyle='->', color='orange', lw=2.5), **kw))

    pad = 1.2
    ax_map.set_xlim(min(xs + WP_X) - pad, max(xs + WP_X) + pad)
    ax_map.set_ylim(min(ys + WP_Y) - pad, max(ys + WP_Y) + pad)

    # ── Heading chart ─────────────────────────────────────────────────────────
    errs = list(_wrap180(np.array(dh) - np.array(ch)))
    cur_line.set_data(ts, ch)
    des_line.set_data(ts, dh)
    err_line.set_data(ts, errs)
    t_end = ts[-1]
    ax_hdg.set_xlim(max(0.0, t_end - WINDOW_SECS), t_end + 0.5)

    # ── Position chart ────────────────────────────────────────────────────────
    dpos_line.set_data(ts, dpos)
    cpos_line.set_data(ts, cpos)
    ax_pos.set_xlim(max(0.0, t_end - WINDOW_SECS), t_end + 0.5)
    all_pos = dpos + cpos
    if all_pos:
        ax_pos.set_ylim(-0.1, max(all_pos) * 1.1 + 0.1)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# ─── Main loop ────────────────────────────────────────────────────────────────
plt.ion()
plt.show()

try:
    while True:
        update()
        plt.pause(0.1)
except KeyboardInterrupt:
    print("\nStopped.")
    plt.ioff()
    plt.show()
