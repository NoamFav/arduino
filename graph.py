# live_pid.py
import re, time, argparse
from collections import deque
import matplotlib.pyplot as plt

pat = re.compile(r"^\d+,\d+(\.\d+)?,\d+(\.\d+)?,-?\d+(\.\d+)?$")
parser = argparse.ArgumentParser()
parser.add_argument("--file", default="data.csv")
parser.add_argument("--window", type=float, default=10.0, help="seconds to show when following")
parser.add_argument("--fps", type=float, default=20.0)
args = parser.parse_args()

# State
t, target, pos, out = deque(), deque(), deque(), deque()
follow = True
win = args.window

def parse_line(s):
    if not s or s.startswith("#") or s.startswith("ms,target,pos,out"):
        return None
    if not pat.match(s):  # skip junk/partial lines
        return None
    ms, a, b, c = s.split(",")
    return (int(ms) / 1000.0, float(a), float(b), float(c))

# Matplotlib setup (single plot, toolbar gives zoom/pan)
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
(line_target,) = ax.plot([], [], label="Target")
(line_pos,)    = ax.plot([], [], label="Position")
(line_out,)    = ax.plot([], [], label="Output")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_title("PID Live")
ax.grid(True)
ax.legend()

def on_key(ev):
    global follow, win
    if ev.key == "f":         # toggle follow mode
        follow = not follow
        print(f"follow={follow}")
    elif ev.key == "+":       # increase window
        win *= 1.5
        print(f"window={win:.2f}s")
    elif ev.key == "-":       # decrease window
        win = max(1.0, win / 1.5)
        print(f"window={win:.2f}s")
    elif ev.key == "r":       # reset view to data bounds
        if t:
            ax.set_xlim(min(t), max(t))
            ax.relim(); ax.autoscale_view(scaley=True)
            fig.canvas.draw_idle()
fig.canvas.mpl_connect("key_press_event", on_key)

# Tail the file
with open(args.file, "r", encoding="utf-8", errors="ignore") as f:
    # fast-forward to end but keep existing data if you want:
    f.seek(0, 0)  # change to f.seek(0, 2) to ignore historical data
    for line in f:
        rec = parse_line(line.strip())
        if rec:
            ts, a, b, c = rec
            t.append(ts); target.append(a); pos.append(b); out.append(c)

    # main loop
    delay = 1.0 / args.fps
    while True:
        where = f.tell()
        line = f.readline()
        if not line:
            time.sleep(delay)
            f.seek(where)
        else:
            rec = parse_line(line.strip())
            if not rec:
                continue
            ts, a, b, c = rec
            t.append(ts); target.append(a); pos.append(b); out.append(c)

        if not t:
            continue

        # Update plot data
        line_target.set_data(t, target)
        line_pos.set_data(t, pos)
        line_out.set_data(t, out)

        # X-limits: follow latest or leave as user-zoomed
        if follow:
            tmax = t[-1]
            ax.set_xlim(max(0, tmax - win), tmax)
            # Autoscale Y to visible window
            # (simple: autoscale whole set; fast: compute window slice)
            ax.relim(); ax.autoscale_view(scaley=True)
        # draw
        plt.pause(0.001)