import math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle, FancyArrow, Polygon
from matplotlib.collections import LineCollection
from matplotlib.transforms import Affine2D
for f in ["Inter-Regular", "Inter-Medium", "Inter-SemiBold", "Inter-Bold"]:
    font_manager.fontManager.addfont(f"/usr/share/fonts/opentype/inter/{f}.otf")
plt.rcParams.update({"font.family": "Inter", "text.color": "#e5e7eb", "axes.labelcolor": "#cbd5e1",
                     "xtick.color": "#94a3b8", "ytick.color": "#94a3b8", "axes.edgecolor": "#334155"})
REPO = "/workspace/ros2-amr-slam-nav2"
sys.path.insert(0, f"{REPO}/amr_sim")
import world
BG = "#0b1117"; PANEL = "#111a23"; ACC = "#38bdf8"; TXT = "#e5e7eb"; MUTE = "#94a3b8"; DIM = "#64748b"
C_LIDAR = "#f87171"; C_PATH = "#fbbf24"; C_PART = "#4ade80"; C_GOAL = "#e879f9"; C_TRAIL = "#38bdf8"
C_DEF = "#94a3b8"; C_TUN = ACC
W, H, FPS = 1920, 1080, 30
HONEST = "Simple 2D simulator (my own)  ·  real slam_toolbox / AMCL / Nav2 on ROS 2 Humble  ·  data from the logged runs"
OX, OY = 1.0, 1.0
GRID = world.build()
ANG = np.linspace(-math.pi, math.pi, 360, endpoint=False)
R_STEPS = np.arange(0.12, 8.0, world.RES / 2)


def raycast(x, y, th):
    """Same lidar model as amr_sim/sim_node.py (360 beams, 8 m), without the 1 cm noise."""
    a = th + ANG
    xs = x + np.outer(np.cos(a), R_STEPS); ys = y + np.outer(np.sin(a), R_STEPS)
    c, rr = (xs / world.RES).astype(int), (ys / world.RES).astype(int)
    inside = (c >= 0) & (rr >= 0) & (c < GRID.shape[1]) & (rr < GRID.shape[0])
    hit = np.ones_like(inside); hit[inside] = GRID[rr[inside], c[inside]]
    first = np.where(hit.any(1), hit.argmax(1), -1)
    return np.where(first >= 0, R_STEPS[np.clip(first, 0, None)], np.inf)


def fig_new():
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=BG); return fig


def map_rgb(m):
    """occupancy grid -> dark RViz-like colours"""
    out = np.zeros(m.shape + (3,))
    unk = np.array([0x0b, 0x11, 0x17]) / 255; free = np.array([0x1e, 0x29, 0x3b]) / 255; occ = np.array([0xe2, 0xe8, 0xf0]) / 255
    out[:] = unk; out[(m >= 0) & (m < 50)] = free; out[m >= 50] = occ
    return out


def draw_map(ax, m, meta):
    res, ox, oy = meta
    ext = [ox + OX, ox + OX + m.shape[1] * res, oy + OY, oy + OY + m.shape[0] * res]
    ax.imshow(map_rgb(m), origin="lower", extent=ext, interpolation="nearest", zorder=0)


def style_ax(ax, xl, yl):
    ax.set_facecolor(BG); ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values(): s.set_color("#1f2a36")


def draw_lidar(ax, x, y, th, ranges, angles=ANG, every=2, z=3):
    a = th + angles
    ok = np.isfinite(ranges) & (ranges < 7.99)
    hx, hy = x + ranges * np.cos(a), y + ranges * np.sin(a)
    idx = np.where(ok)[0][::every]
    segs = [((x, y), (hx[i], hy[i])) for i in idx]
    ax.add_collection(LineCollection(segs, colors=C_LIDAR, linewidths=0.8, alpha=0.32, zorder=z))
    ax.scatter(hx[ok], hy[ok], s=16, color=C_LIDAR, zorder=z + 1, linewidths=0)


def draw_robot(ax, x, y, th, z=6, scale=1.0):
    """Differential-drive AMR at true scale: 0.36 m body (sim footprint radius 0.18 m), two drive wheels, heading arrow."""
    tr = Affine2D().rotate(th).translate(x, y) + ax.transData
    body = Polygon([(-0.17, -0.15), (0.13, -0.15), (0.19, -0.07), (0.19, 0.07), (0.13, 0.15), (-0.17, 0.15)],
                   closed=True, fc="#f1f5f9", ec=ACC, lw=1.6, zorder=z, transform=tr)
    ax.add_patch(body)
    for s in (-1, 1):
        ax.add_patch(Rectangle((-0.07, s * 0.15 - 0.035 + (0 if s > 0 else 0)), 0.14, 0.07 * 1.0, fc="#0f172a", ec="#0f172a",
                               zorder=z + 1, transform=tr))
    ax.add_patch(Circle((0.10, 0), 0.035, fc=C_LIDAR, ec="#7f1d1d", lw=0.8, zorder=z + 2, transform=tr))  # lidar puck
    ax.add_patch(FancyArrow(-0.08, 0, 0.36, 0, width=0.035, head_width=0.11, head_length=0.1, fc=ACC, ec=ACC,
                            zorder=z + 3, transform=tr, length_includes_head=True))


def draw_goal(ax, gx, gy, gyaw, z=5, r=0.05):
    ax.add_patch(Circle((gx, gy), 0.05, fc="none", ec=C_GOAL, lw=2, zorder=z))
    ax.plot([gx - 0.12, gx + 0.12], [gy, gy], color=C_GOAL, lw=1.2, zorder=z); ax.plot([gx, gx], [gy - 0.12, gy + 0.12], color=C_GOAL, lw=1.2, zorder=z)
    ax.add_patch(FancyArrow(gx, gy, 0.3 * math.cos(gyaw), 0.3 * math.sin(gyaw), width=0.015, head_width=0.07, head_length=0.07,
                            fc=C_GOAL, ec=C_GOAL, zorder=z, length_includes_head=True))


def header(fig, step, title, sub=None):
    fig.text(0.035, 0.925, step, fontsize=17, color=ACC, weight="semibold")
    fig.text(0.035, 0.875, title, fontsize=34, color=TXT, weight="bold")
    if sub: fig.text(0.035, 0.835, sub, fontsize=17, color=MUTE)


def footer(fig):
    fig.text(0.035, 0.03, HONEST, fontsize=13, color=DIM)
    fig.text(0.965, 0.03, "Shanmukh Narayana Chaganti", fontsize=13, color=DIM, ha="right")


def badge(fig, x, y, text):
    fig.text(x, y, text, fontsize=15, color=BG, weight="semibold", ha="right",
             bbox=dict(boxstyle="round,pad=0.45", fc=ACC, ec="none"))


def legend(fig, x, y, items, dy=0.045, fs=16):
    """items: list of (kind, label)"""
    ax = fig.add_axes([x, y - dy * len(items), 0.03, dy * len(items)]); ax.axis("off")
    ax.set_xlim(0, 1); ax.set_ylim(0, len(items))
    for i, (kind, lab) in enumerate(items):
        yy = len(items) - i - 0.5
        if kind == "robot":
            ax.add_patch(Rectangle((0.15, yy - 0.22), 0.7, 0.44, fc="#f1f5f9", ec=ACC, lw=1.5))
        elif kind == "lidar":
            ax.plot([0.05, 0.75], [yy - 0.2, yy + 0.15], color=C_LIDAR, alpha=0.4, lw=1); ax.plot([0.8], [yy + 0.17], "o", color=C_LIDAR, ms=4)
        elif kind == "path":
            ax.plot([0.05, 0.95], [yy, yy], color=C_PATH, lw=2.5)
        elif kind == "trail":
            ax.plot([0.05, 0.95], [yy, yy], color=C_TRAIL, lw=2.5, alpha=0.6)
        elif kind == "particles":
            ax.scatter([0.2, 0.45, 0.7, 0.35, 0.6], [yy, yy + 0.15, yy - 0.1, yy - 0.18, yy + 0.2], s=8, color=C_PART)
        elif kind == "goal":
            ax.plot([0.5], [yy], "o", mfc="none", mec=C_GOAL, ms=10, mew=2); ax.plot([0.2, 0.8], [yy, yy], color=C_GOAL, lw=1)
        elif kind == "wall":
            ax.add_patch(Rectangle((0.15, yy - 0.22), 0.7, 0.44, fc="#e2e8f0"))
        elif kind == "free":
            ax.add_patch(Rectangle((0.15, yy - 0.22), 0.7, 0.44, fc="#1e293b", ec="#334155"))
        fig.text(x + 0.04, y - dy * (i + 0.5), lab, fontsize=fs, color=TXT, va="center")
