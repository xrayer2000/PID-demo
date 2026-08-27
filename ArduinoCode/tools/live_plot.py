#!/usr/bin/env python3
"""
Lightweight live serial telemetry plotter.

A direct, low-overhead alternative to the Teleplot VS Code extension, for
when you need a redraw rate it won't give you. Reads the exact same wire
format your firmware already emits for Teleplot (`>name:value\\n`), so no
firmware changes are needed.

Usage:
    # single-column mode (original behavior) — one row per --group
    python live_plot.py --port /dev/ttyACM0 --baud 115200 \\
        --group "angle_AXIS 2,setPoint_AXIS 2" --group "rpm_AXIS 2"

    # grid mode — rows are BASE names (no "_AXIS n" suffix), columns are axes.
    # --axes "1,2" puts AXIS 1 on the left, AXIS 2 on the right.
    python live_plot.py --port /dev/ttyACM0 --baud 115200 \\
        --axes "1,2" --group "angle" --group "setPoint" --group "velocity"
    # -> 3 rows x 2 cols = 6 subplots, left col = "_AXIS 1", right col = "_AXIS 2"

If you don't pass --group (and don't pass --axes), every variable gets its
own subplot. Close any other program holding the serial port (Teleplot,
PlatformIO monitor, etc.) before running this — only one process can own it
at a time.
"""

#launch with: python3 live_plot.py --port /dev/ttyACM0 --baud 115200 --axes "1,2" --group "angle" --group "setPoint" --group "velocity"


import argparse
import re
import threading
import time
from collections import deque, defaultdict

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

LINE_RE = re.compile(r'^>([^:]+):([-+]?(?:[0-9]*\.?[0-9]+|nan))')


def serial_reader(ser, data, lock, window_seconds, t0):
    while True:
        try:
            raw = ser.readline().decode(errors='ignore').strip()
        except Exception:
            continue
        m = LINE_RE.match(raw)
        if not m:
            continue
        name, value = m.group(1), float(m.group(2))
        t = time.time() - t0
        with lock:
            dq = data[name]
            dq.append((t, value))
            cutoff = t - window_seconds
            while dq and dq[0][0] < cutoff:
                dq.popleft()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True, help='e.g. /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--window', type=float, default=15.0, help='seconds of history shown')
    ap.add_argument('--fps', type=float, default=60.0, help='plot redraw rate')
    ap.add_argument('--group', action='append', default=[],
                     help='comma-separated variable names sharing one subplot row '
                          '(repeat --group for multiple rows). In grid mode (--axes set), '
                          'these are BASE names without the "_AXIS n" suffix.')
    ap.add_argument('--axes', default=None,
                     help='comma-separated axis numbers, left-to-right column order, '
                          'e.g. "1,2". When set, enables grid mode: each --group becomes '
                          'a row, each axis becomes a column, and "_AXIS <n>" is appended '
                          'to every base name to find the matching telemetry variable.')
    ap.add_argument('--overlay', action='store_true', help='overlay all selected axes on the same subplot')
    args = ap.parse_args()

    if not args.group:
        args.group = ['pos,CmdPos', 'vel,CmdVel', 'acc']

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    data = defaultdict(deque)
    lock = threading.Lock()
    t0 = time.time()

    reader = threading.Thread(target=serial_reader, args=(ser, data, lock, args.window, t0),
                               daemon=True)
    reader.start()

    print("Waiting for first telemetry line...")
    while not data:
        time.sleep(0.05)

    with lock:
        names = sorted(data.keys())

    axis_list = [a.strip() for a in args.axes.split(',')] if args.axes else None

    lines = {}
    ax_to_names = {}

    if axis_list and args.overlay:
        row_groups = [g.split(',') for g in args.group]
        nrows = len(row_groups)

        fig, axes_grid = plt.subplots(
            nrows, 1, sharex=True,
            figsize=(10, 2.5 * nrows),
            squeeze=False
        )

        for r, bases in enumerate(row_groups):
            ax = axes_grid[r][0]

            row_names = []

            for axis_num in axis_list:
                for base in bases:
                    full_name = f'{base}_AXIS {axis_num}'

                    (line,) = ax.plot([], [], label=f'{base} AXIS {axis_num}')

                    lines[full_name] = (ax, line)
                    row_names.append(full_name)

            ax_to_names[ax] = row_names
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)

            if r == 0:
                ax.set_title('AXIS 1 vs AXIS 2')

        axes_grid[-1][0].set_xlabel('time (s)')
        axes = [axes_grid[r][0] for r in range(nrows)]

    elif axis_list:
        if not args.group:
            raise SystemExit('--axes requires at least one --group of base names')
        row_groups = [g.split(',') for g in args.group]
        nrows, ncols = len(row_groups), len(axis_list)

        fig, axes_grid = plt.subplots(nrows, ncols, sharex=True,
                                       figsize=(5 * ncols, 2.5 * nrows), squeeze=False)

        for r, bases in enumerate(row_groups):
            for c, axis_num in enumerate(axis_list):
                ax = axes_grid[r][c]
                row_names = []
                for base in bases:
                    full_name = f'{base}_AXIS {axis_num}'
                    (line,) = ax.plot([], [], label=base)
                    lines[full_name] = (ax, line)
                    row_names.append(full_name)
                ax_to_names[ax] = row_names
                ax.legend(loc='upper right', fontsize=8)
                ax.grid(True, alpha=0.3)
                if r == 0:
                    ax.set_title(f'AXIS {axis_num}')
        for c in range(ncols):
            axes_grid[-1][c].set_xlabel('time (s)')
        axes = [ax for row in axes_grid for ax in row]
    else:
        groups = [g.split(',') for g in args.group] if args.group else [[n] for n in names]

        fig, axes_col = plt.subplots(len(groups), 1, sharex=True, figsize=(10, 3 * len(groups)))
        if len(groups) == 1:
            axes_col = [axes_col]

        for ax, group in zip(axes_col, groups):
            for name in group:
                (line,) = ax.plot([], [], label=name)
                lines[name] = (ax, line)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes_col[-1].set_xlabel('time (s)')
        axes = list(axes_col)
        ax_to_names = {ax: [n for n, (a, _) in lines.items() if a is ax] for ax in axes}

    plt.tight_layout()
    plt.show(block=False)

    for ax in axes:
        ax.set_xlim(-args.window, 0)

    all_lines = [line for _, line in lines.values()]

    def capture_backgrounds():
        for line in all_lines:
            line.set_visible(False)
        fig.canvas.draw()
        bg = {ax: fig.canvas.copy_from_bbox(ax.bbox) for ax in axes}
        for line in all_lines:
            line.set_visible(True)
        return bg

    backgrounds = capture_backgrounds()

    margin_frac = 0.15      # rescale when data gets within 15% of current edge
    min_rescale_gap = 0.2   # don't rescale more than 5x/sec even during fast transients
    last_rescale = 0.0

    def axis_needs_rescale(ax, names, snapshot):
        ymin, ymax = float('inf'), float('-inf')
        for name in names:
            pts = snapshot.get(name)
            if not pts:
                continue
            ys = [p[1] for p in pts]
            ymin, ymax = min(ymin, min(ys)), max(ymax, max(ys))
        if ymin == float('inf'):
            return False
        lo, hi = ax.get_ylim()
        span = hi - lo
        pad = span * margin_frac
        return ymin < lo + pad or ymax > hi - pad

    frame_interval = 1.0 / args.fps

    try:
        while plt.fignum_exists(fig.number):
            loop_start = time.time()
            with lock:
                snapshot = {k: list(v) for k, v in data.items() if k in lines}
            now = time.time() - t0

            for name, (ax, line) in lines.items():
                pts = snapshot.get(name)
                if not pts:
                    continue
                xs, ys = zip(*pts)
                xs = [x - now for x in xs]
                line.set_data(xs, ys)

            do_rescale = (loop_start - last_rescale > min_rescale_gap) and any(
                axis_needs_rescale(ax, ax_to_names[ax], snapshot) for ax in axes
            )

            if do_rescale:
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
                backgrounds = capture_backgrounds()
                last_rescale = loop_start

            for ax in axes:
                fig.canvas.restore_region(backgrounds[ax])
            for name, (ax, line) in lines.items():
                if snapshot.get(name):
                    ax.draw_artist(line)
            for ax in axes:
                fig.canvas.blit(ax.bbox)

            fig.canvas.flush_events()

            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()