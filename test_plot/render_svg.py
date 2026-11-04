#!/usr/bin/env python3
"""把 dump_waveform 的 CSV 画成纯 SVG 图；不依赖 matplotlib，可在无 ROS 环境运行。"""

import bisect
import csv
import html
import math
import sys
from pathlib import Path

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
MOTOR_LABELS = [
    "fin_0 (x=0.00, z=+0.20)",
    "fin_1 (x=0.12, z=-0.20)",
    "fin_2 (x=0.24, z=+0.25)",
    "fin_3 (x=0.36, z=-0.25)",
]
WIDTH = 1320
LEFT = 96
RIGHT_MARGIN = 300
TOP = 96
PANEL_HEIGHT = 250
GAP = 62
BOTTOM = 74


def esc(text):
    return html.escape(str(text), quote=True)


def read_waveform(path):
    rows = []
    meta = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("#"):
                for token in line[1:].strip().split():
                    if "=" in token:
                        key, value = token.split("=", 1)
                        meta[key] = value
                continue
            header = line.strip().split(",")
            break
        reader = csv.DictReader(stream, fieldnames=header)
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
    return meta, rows


def read_summary(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def nice_ticks(low, high, count=5):
    if high <= low:
        high = low + 1.0
    raw_step = (high - low) / count
    magnitude = 10 ** math.floor(math.log10(abs(raw_step))) if raw_step > 0 else 1.0
    step = magnitude
    for candidate in (1.0, 2.0, 2.5, 5.0, 10.0):
        step = candidate * magnitude
        if raw_step <= step:
            break
    ticks = []
    value = math.ceil(low / step - 1e-9) * step
    while value <= high + 1e-9:
        ticks.append(round(value, 12))
        value += step
    if not ticks:
        ticks = [low, high]
    return ticks


def format_tick(value):
    if abs(value) < 1e-12:
        return "0"
    if abs(value) >= 1000 or abs(value) < 1e-3:
        return "{0:.2e}".format(value)
    text = ("{0:.4f}".format(value)).rstrip("0").rstrip(".")
    return text if text else "0"


class Panel:
    def __init__(self, x0, y0, width, height, x_min, x_max, y_min, y_max):
        self.x0 = x0
        self.y0 = y0
        self.width = width
        self.height = height
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def px(self, x):
        return self.x0 + (x - self.x_min) / (self.x_max - self.x_min) * self.width

    def py(self, y):
        return self.y0 + self.height - (y - self.y_min) / (self.y_max - self.y_min) * self.height


def draw_panel(parts, panel, y_label, x_label, series, extra=""):
    parts.append(
        '<rect x="{0:.1f}" y="{1:.1f}" width="{2:.1f}" height="{3:.1f}" '
        'fill="#ffffff" stroke="#d0d7de"/>'.format(
            panel.x0, panel.y0, panel.width, panel.height
        )
    )
    for tick in nice_ticks(panel.x_min, panel.x_max):
        x = panel.px(tick)
        parts.append(
            '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
            'stroke="#eef1f4"/>'.format(x, panel.y0, panel.y0 + panel.height)
        )
        parts.append(
            '<text x="{0:.1f}" y="{1:.1f}" font-size="12" fill="#57606a" '
            'text-anchor="middle">{2}</text>'.format(
                x, panel.y0 + panel.height + 18, esc(format_tick(tick))
            )
        )
    for tick in nice_ticks(panel.y_min, panel.y_max):
        y = panel.py(tick)
        parts.append(
            '<line x1="{0:.1f}" y1="{1:.1f}" x2="{2:.1f}" y2="{1:.1f}" '
            'stroke="#eef1f4"/>'.format(panel.x0, y, panel.x0 + panel.width)
        )
        parts.append(
            '<text x="{0:.1f}" y="{1:.1f}" font-size="12" fill="#57606a" '
            'text-anchor="end">{2}</text>'.format(
                panel.x0 - 10, y + 4, esc(format_tick(tick))
            )
        )
    parts.append(
        '<text x="{0:.1f}" y="{1:.1f}" font-size="13" fill="#24292f" '
        'text-anchor="middle" transform="rotate(-90 {0:.1f} {1:.1f})">{2}</text>'.format(
            panel.x0 - 58, panel.y0 + panel.height / 2, esc(y_label), panel.x0 - 58,
            panel.y0 + panel.height / 2
        )
    )
    parts.append(
        '<text x="{0:.1f}" y="{1:.1f}" font-size="13" fill="#24292f" '
        'text-anchor="middle">{2}</text>'.format(
            panel.x0 + panel.width / 2, panel.y0 + panel.height + 42, esc(x_label)
        )
    )
    for name, color, points, width, dash in series:
        if not points:
            continue
        if width <= 0.0:
            for x, y in points[::6]:
                parts.append(
                    '<circle cx="{0:.2f}" cy="{1:.2f}" r="2.2" fill="{2}" '
                    'fill-opacity="0.55"/>'.format(panel.px(x), panel.py(y), color)
                )
            continue
        path = "M " + " L ".join(
            "{0:.2f},{1:.2f}".format(panel.px(x), panel.py(y)) for x, y in points
        )
        dash_attr = ' stroke-dasharray="{0}"'.format(dash) if dash else ""
        parts.append(
            '<path d="{0}" fill="none" stroke="{1}" stroke-width="{2}" '
            'stroke-linejoin="round"{3}/>'.format(path, color, width, dash_attr)
        )
    parts.append(extra)


def build_figure(tag, meta, rows, summary_row, fluent_phase=None):
    period = float(summary_row["period_s"])
    t1 = float(summary_row["t1_s"])
    t2 = float(summary_row["t2_s"])
    fast_rate = float(summary_row["fast_rate_rad_s"])
    slow_rate = float(summary_row["slow_rate_rad_s"])
    x_max = period * 2.0

    phase = [(row["time_s"], row["phase_rad"]) for row in rows]
    rate = [(row["time_s"], row["phase_rate_rad_s"]) for row in rows]
    displacement = [
        [(row["time_s"], row["displacement_{0}".format(i)]) for row in rows]
        for i in range(4)
    ]

    disp_min = min(point[1] for series in displacement for point in series)
    disp_max = max(point[1] for series in displacement for point in series)
    padding = max(1e-6, (disp_max - disp_min) * 0.12)
    rate_max = max(fast_rate, slow_rate) * 1.12

    height = TOP + PANEL_HEIGHT * 3 + GAP * 2 + BOTTOM
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" '
        'viewBox="0 0 {0} {1}" font-family="DejaVu Sans, Arial, sans-serif">'.format(
            WIDTH, height
        ),
        '<rect width="100%" height="100%" fill="#f6f8fa"/>',
        '<text x="{0}" y="38" font-size="23" font-weight="600" fill="#111827">'
        "Fluent asymmetric flapping — {1}</text>".format(LEFT, esc(tag)),
        '<text x="{0}" y="64" font-size="14" fill="#374151">'
        "A={1} model-unit · F={2} Hz · fi(body_coeff)={3} · duty(tf_coeff)={4} · "
        "mode={5} · source: {6}</text>".format(
            LEFT,
            esc(meta.get("amplitude_model", summary_row["amplitude_model"])),
            esc(meta.get("frequency_hz", summary_row["frequency_hz"])),
            esc(meta.get("body_coeff", summary_row["body_coeff"])),
            esc(meta.get("tf_coeff", summary_row["tf_coeff"])),
            esc(meta.get("waveform", summary_row["waveform"])),
            esc(summary_row["source"]),
        ),
    ]

    width = WIDTH - LEFT - RIGHT_MARGIN
    panels = [
        Panel(LEFT, TOP, width, PANEL_HEIGHT, 0.0, x_max, 0.0, 2.0 * math.pi),
        Panel(LEFT, TOP + PANEL_HEIGHT + GAP, width, PANEL_HEIGHT, 0.0, x_max,
              -rate_max, rate_max),
        Panel(LEFT, TOP + (PANEL_HEIGHT + GAP) * 2, width, PANEL_HEIGHT, 0.0, x_max,
              disp_min - padding, disp_max + padding),
    ]

    annotations = []
    for index, panel in enumerate(panels[:2]):
        for cycle in range(2):
            for boundary, color in ((t1 + cycle * period, "#9a6700"),
                                    (t2 + cycle * period, "#8250df")):
                x = panel.px(boundary)
                annotations.append(
                    '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
                    'stroke="{3}" stroke-width="1" stroke-dasharray="4 4"/>'.format(
                        x, panel.y0, panel.y0 + panel.height, color
                    )
                )

    phase_series = [("phase", "#0b3d91", phase, 2.2, None)]
    if fluent_phase:
        phase_series.append(("fluent", "#e6550d", fluent_phase, 0.0, None))
    draw_panel(parts, panels[0], "phase φ (rad)", "time (s)", phase_series,
               "".join(annotations))
    for value, label in ((math.pi / 2.0, "π/2"), (3.0 * math.pi / 2.0, "3π/2")):
        y = panels[0].py(value)
        parts.append(
            '<line x1="{0:.1f}" y1="{1:.1f}" x2="{2:.1f}" y2="{1:.1f}" '
            'stroke="#8c959f" stroke-width="1" stroke-dasharray="2 3"/>'.format(
                panels[0].x0, y, panels[0].x0 + panels[0].width
            )
        )
        parts.append(
            '<text x="{0:.1f}" y="{1:.1f}" font-size="11" fill="#8c959f">{2}</text>'.format(
                panels[0].x0 + panels[0].width + 6, y + 4, esc(label)
            )
        )
    parts.append(
        '<text x="{0:.1f}" y="{1:.1f}" font-size="11" fill="#9a6700">'
        "t1 = duty/2·T</text>".format(
            panels[0].px(t1) + 6, panels[0].y0 + 16
        )
    )
    parts.append(
        '<text x="{0:.1f}" y="{1:.1f}" font-size="11" fill="#8250df">'
        "t2 = (1−duty/2)·T</text>".format(
            panels[0].px(t2) + 6, panels[0].y0 + 32
        )
    )

    rate_series = [("rate", "#0b3d91", rate, 2.0, None)]
    extras = []
    for value, color, label in ((fast_rate, "#bc4c00", "π·F/duty"),
                                (-fast_rate, "#bc4c00", None),
                                (slow_rate, "#1a7f37", "π·F/(1−duty)"),
                                (-slow_rate, "#1a7f37", None)):
        y = panels[1].py(value)
        extras.append(
            '<line x1="{0:.1f}" y1="{1:.1f}" x2="{2:.1f}" y2="{1:.1f}" '
            'stroke="{3}" stroke-width="1" stroke-dasharray="5 4"/>'.format(
                panels[1].x0, y, panels[1].x0 + panels[1].width, color
            )
        )
        if label:
            label_y = y - 9 if y - 9 > panels[1].y0 + 14 else y + 18
            extras.append(
                '<text x="{0:.1f}" y="{1:.1f}" font-size="11" fill="{2}">{3}</text>'.format(
                    panels[1].x0 + 8, label_y, color, esc(label)
                )
            )
    draw_panel(parts, panels[1], "dφ/dt (rad/s)", "time (s)", rate_series,
               "".join(extras))

    disp_series = [
        ("fin{0}".format(i), COLORS[i], displacement[i], 1.9, None) for i in range(4)
    ]
    draw_panel(parts, panels[2], "model displacement (model unit)", "time (s)",
               disp_series)
    zero_y = panels[2].py(0.0)
    parts.append(
        '<line x1="{0:.1f}" y1="{1:.1f}" x2="{2:.1f}" y2="{1:.1f}" '
        'stroke="#8c959f" stroke-width="1"/>'.format(
            panels[2].x0, zero_y, panels[2].x0 + panels[2].width
        )
    )

    # 延迟模式：标出每个电机自己的换向时刻，应正好落在它的波峰/波谷。
    if summary_row.get("waveform") == "fluent_asymmetric_delayed":
        dt = rows[1]["time_s"] - rows[0]["time_s"]
        for index in range(4):
            values = [row["displacement_{0}".format(index)] for row in rows]
            delay = float(summary_row["delay_{0}".format(index)])
            for base in (t1, t2):
                for cycle in (0, 1):
                    moment = base + delay + cycle * period
                    if moment < 0.0 or moment > x_max:
                        continue
                    sample_index = min(
                        len(values) - 1, max(0, int(round(moment / dt)))
                    )
                    x = panels[2].px(moment)
                    parts.append(
                        '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
                        'stroke="{3}" stroke-width="1" stroke-opacity="0.35" '
                        'stroke-dasharray="2 3"/>'.format(
                            x, panels[2].y0, panels[2].y0 + panels[2].height,
                            COLORS[index]
                        )
                    )
                    parts.append(
                        '<circle cx="{0:.2f}" cy="{1:.2f}" r="3" fill="{2}"/>'.format(
                            x, panels[2].py(values[sample_index]), COLORS[index]
                        )
                    )

    legend_x = LEFT + width + 24
    legend_y = panels[2].y0 + 8
    parts.append(
        '<text x="{0}" y="{1}" font-size="13" font-weight="600" fill="#24292f">'
        "motors / mock mounts</text>".format(legend_x, legend_y)
    )
    for index, label in enumerate(MOTOR_LABELS):
        y = legend_y + 26 + index * 24
        parts.append(
            '<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" stroke="{3}" '
            'stroke-width="3"/>'.format(legend_x, y - 4, legend_x + 26, COLORS[index])
        )
        parts.append(
            '<text x="{0}" y="{1}" font-size="12" fill="#374151">{2}</text>'.format(
                legend_x + 34, y, esc(label)
            )
        )
    note_y = legend_y + 26 + 4 * 24 + 16
    for line in (
        "fi = body_coeff: ψᵢ = xᵢ·fi·2π/0.55",
        "wave = sin(ψᵢ − φ)",
        "duty = tf_coeff: 快段占周期比例",
        "mock_linear: q = offset + gain·d",
    ):
        parts.append(
            '<text x="{0}" y="{1}" font-size="12" fill="#57606a">{2}</text>'.format(
                legend_x, note_y, esc(line)
            )
        )
        note_y += 20
    if fluent_phase:
        parts.append(
            '<text x="{0}" y="{1}" font-size="12" fill="#e6550d">'
            "橙点: Fluent 记录 phase</text>".format(legend_x, note_y + 8)
        )
    if summary_row.get("waveform") == "fluent_asymmetric_delayed":
        parts.append(
            '<text x="{0}" y="{1}" font-size="12" fill="#57606a">'
            "虚线+圆点=该电机换向时刻</text>".format(legend_x, note_y + 28)
        )

    parts.append(
        '<text x="{0}" y="{1}" font-size="12" fill="#6b7280">'
        "生成自 fish_control/fluent_waveform.cpp（C++） · 双周期 · 1601 采样点</text>".format(
            LEFT, height - 24
        )
    )
    parts.append("</svg>")
    return "\n".join(parts)


def build_comparison(summary, waveforms):
    period = max(float(row["period_s"]) for row in summary)
    x_max = period * 2.0
    rows_all = []
    for row, (meta, rows) in zip(summary, waveforms):
        if row.get("waveform") != "fluent_asymmetric":
            continue
        series = [(r["time_s"], r["displacement_0"]) for r in rows]
        rows_all.append((row["tag"], series, row))

    all_values = [value for _, series, _ in rows_all for _, value in series]
    low, high = min(all_values), max(all_values)
    padding = max(1e-6, (high - low) * 0.12)
    height = TOP + PANEL_HEIGHT * 2 + GAP + BOTTOM
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" '
        'viewBox="0 0 {0} {1}" font-family="DejaVu Sans, Arial, sans-serif">'.format(
            WIDTH, height
        ),
        '<rect width="100%" height="100%" fill="#f6f8fa"/>',
        '<text x="{0}" y="38" font-size="23" font-weight="600" fill="#111827">'
        "fin_0 位移对比 · {1} 组 Fluent 参数</text>".format(LEFT, len(rows_all)),
        '<text x="{0}" y="64" font-size="14" fill="#374151">'
        "同一位移公式，仅改变 A / F / fi / duty；两周期</text>".format(LEFT),
    ]
    width = WIDTH - LEFT - RIGHT_MARGIN
    panel = Panel(LEFT, TOP, width, PANEL_HEIGHT * 2, 0.0, x_max, low - padding, high + padding)
    series = []
    palette = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#e377c2", "#8c564b"]
    for index, (tag, points, _row) in enumerate(rows_all):
        series.append((tag, palette[index % len(palette)], points, 1.8, None))
    draw_panel(parts, panel, "fin_0 model displacement (model unit)", "time (s)", series)
    zero_y = panel.py(0.0)
    parts.append(
        '<line x1="{0:.1f}" y1="{1:.1f}" x2="{2:.1f}" y2="{1:.1f}" '
        'stroke="#8c959f" stroke-width="1"/>'.format(
            panel.x0, zero_y, panel.x0 + panel.width
        )
    )
    legend_x = LEFT + width + 24
    legend_y = TOP + 8
    parts.append(
        '<text x="{0}" y="{1}" font-size="13" font-weight="600" fill="#24292f">'
        "参数组</text>".format(legend_x, legend_y)
    )
    for index, (tag, points, row) in enumerate(rows_all):
        y = legend_y + 26 + index * 46
        parts.append(
            '<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" stroke="{3}" '
            'stroke-width="3"/>'.format(
                legend_x, y - 4, legend_x + 26, palette[index % len(palette)]
            )
        )
        parts.append(
            '<text x="{0}" y="{1}" font-size="12" fill="#24292f">{2}</text>'.format(
                legend_x + 34, y, esc(tag)
            )
        )
        parts.append(
            '<text x="{0}" y="{1}" font-size="11" fill="#57606a">'
            "A={2} · F={3} · fi={4} · duty={5}</text>".format(
                legend_x + 34, y + 16,
                esc(row["amplitude_model"]), esc(row["frequency_hz"]),
                esc(row["body_coeff"]), esc(row["tf_coeff"]),
            )
        )
    parts.append("</svg>")
    return "\n".join(parts)


def load_fluent_phase(csv_path, limit):
    points = []
    with Path(csv_path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            time_s = float(row["time"])
            if time_s > limit:
                break
            points.append((time_s, float(row["phase"])))
    return points


def phase_deviation(rows, fluent_points, breakpoints):
    """把 Fluent 记录的 phase 与本次 C++ 输出线性插值比较，跳变点附近不参与统计。"""
    if not fluent_points:
        return None, 0
    times = [row["time_s"] for row in rows]
    phases = [row["phase_rad"] for row in rows]
    worst = 0.0
    compared = 0
    for time_s, recorded in fluent_points:
        if any(abs(time_s - point) < 0.01 for point in breakpoints):
            continue
        index = bisect.bisect_left(times, time_s)
        if index <= 0:
            value = phases[0]
        elif index >= len(times):
            value = phases[-1]
        else:
            left, right = times[index - 1], times[index]
            ratio = (time_s - left) / (right - left)
            value = phases[index - 1] + ratio * (phases[index] - phases[index - 1])
        worst = max(worst, abs(value - recorded))
        compared += 1
    return worst, compared


def main(argv):
    output = Path(argv[1]).resolve() if len(argv) > 1 else Path(__file__).resolve().parent / "output"
    summary = read_summary(output / "summary.csv")
    waveforms = [read_waveform(output / (row["tag"] + ".csv")) for row in summary]

    fluent_source = output.parent.parent.parent.parent / "01_sim/data/last_all_ultra_data/30_yuanshi_data/data1_soft.csv"
    baseline = "data1_baseline"
    for row, (meta, rows) in zip(summary, waveforms):
        fluent = None
        if row["tag"] == baseline and fluent_source.is_file():
            try:
                fluent = load_fluent_phase(fluent_source, float(row["period_s"]) * 2.0)
            except (KeyError, ValueError, OSError):
                fluent = None
        figure = build_figure(row["tag"], meta, rows, row, fluent)
        target = output / (row["tag"] + ".svg")
        target.write_text(figure, encoding="utf-8")
        print("[PASS] wrote", target.name, "with" if fluent else "without", "Fluent overlay")
        if fluent and row["tag"] == baseline:
            period = float(row["period_s"])
            breakpoints = []
            for cycle in range(2):
                breakpoints.extend([cycle * period,
                                    cycle * period + float(row["t1_s"]),
                                    cycle * period + float(row["t2_s"])])
            worst, compared = phase_deviation(rows, fluent, breakpoints)
            status = "[PASS]" if worst is not None and worst < 2e-6 else "[WARN]"
            print("{0} baseline phase vs Fluent recorded: max|delta|={1:.2e} rad over {2} samples"
                  .format(status, worst if worst is not None else float("nan"), compared))

    comparison = build_comparison(summary, waveforms)
    (output / "compare_fin0.svg").write_text(comparison, encoding="utf-8")
    print("[PASS] wrote compare_fin0.svg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
