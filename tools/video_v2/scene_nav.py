"""Scene 2: AMCL + Nav2, goal 1 (8.0, 1.0), default vs tuned params, replayed at 1x (real time) from the
visualisation re-runs (runs/<name>/viz.pkl: /ground_truth, /scan, /particle_cloud, /plan)."""
import os, sys, json, pickle, numpy as np
from multiprocessing import Pool
from theme import *
GOAL = (8.0, 1.0, 0.0); GI = 0
zn = np.load(f"{REPO}/results/nav_tuned.npz"); MAP, META = zn["map0"], zn["meta"][0]
XL0, YL0 = (-0.15, 10.25), (-0.15, 2.85)


def load(name):
    d = pickle.load(open(f"runs/{name}/viz.pkl", "rb")); goals = json.load(open(f"runs/{name}/goals.json"))
    gt = np.array(d["gt"]); scan_t = np.array([s[0] for s in d["scan"]])
    mv = np.argmax(np.hypot(gt[:, 1] - gt[0, 1], gt[:, 2] - gt[0, 2]) > 0.01); t0 = gt[mv, 0] - 1.0
    # end of goal 1 = first global plan published for goal 2
    g2 = np.array([goals_xy[1][0] - OX, goals_xy[1][1] - OY])
    t_end = min(p[0] for p in d["plan"] if np.hypot(*(p[1][-1] - g2)) < 0.3)
    stop_i = np.searchsorted(gt[:, 0], t_end) - 1
    return dict(gt=gt, scan=d["scan"], scan_t=scan_t, pc=d["pc"], pc_t=np.array([p[0] for p in d["pc"]]),
                plan=d["plan"], plan_t=np.array([p[0] for p in d["plan"]]), t0=t0, t_end=t_end,
                err=goals[GI]["final_err_m"], res=goals[GI]["result"], stop=gt[stop_i])


goals_xy = [(8.0, 1.0), (9.35, 3.3)]
RUNS = {"default": load("default"), "tuned": load(os.environ.get("TUNED", "tuned"))}
DUR = max(r["t_end"] - r["t0"] for r in RUNS.values()) + 1.0
SPEED = 1.0
NPLAY = int(DUR * FPS / SPEED); NZOOM = 75; NHOLD = 75
NTOT = NPLAY + NZOOM + NHOLD
OUT = "frames/nav"; os.makedirs(OUT, exist_ok=True)
ease = lambda u: 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0), 1))


def panel(fig, rect, r, label, col, t, zoom):
    ax = fig.add_axes(rect)
    w0, h0 = XL0[1] - XL0[0], YL0[1] - YL0[0]; cx0, cy0 = np.mean(XL0), np.mean(YL0)
    w = w0 + (2.6 - w0) * zoom; h = w * h0 / w0
    cx = cx0 + (GOAL[0] + 0.05 - cx0) * zoom; cy = cy0 + (GOAL[1] - cy0) * zoom
    style_ax(ax, (cx - w / 2, cx + w / 2), (cy - h / 2, cy + h / 2))
    draw_map(ax, MAP, META)
    gt = r["gt"]; tt = min(t, r["t_end"]); i = max(0, min(np.searchsorted(gt[:, 0], tt) - 1, len(gt) - 1))
    x, y = np.interp(tt, gt[:, 0], gt[:, 1]), np.interp(tt, gt[:, 0], gt[:, 2])
    th = np.interp(tt, gt[:, 0], np.unwrap(gt[:, 3]))  # smooth pose between logged samples
    j = np.searchsorted(r["plan_t"], tt) - 1
    if j >= 0 and t < r["t_end"] + 0.5:
        p = r["plan"][j][1]; ax.plot(p[:, 0] + OX, p[:, 1] + OY, color=C_PATH, lw=2.2, zorder=2.5, alpha=0.9)
    m = (gt[:, 0] >= r["t0"]) & (gt[:, 0] <= tt)
    ax.plot(gt[m, 1], gt[m, 2], color=C_TRAIL, lw=1.6, alpha=0.5, zorder=2)
    k = np.searchsorted(r["pc_t"], tt) - 1
    if k >= 0:
        pc = r["pc"][k][2]; ax.scatter(pc[:, 0] + OX, pc[:, 1] + OY, s=5 + 25 * zoom, color=C_PART, alpha=0.55, linewidths=0, zorder=4.5)
    s = np.searchsorted(r["scan_t"], tt) - 1
    if s >= 0: draw_lidar(ax, x, y, th, r["scan"][s][1])
    draw_robot(ax, x, y, th)
    draw_goal(ax, *GOAL, z=11)
    done = t >= r["t_end"]
    ax.text(0.012, 0.93, label, transform=ax.transAxes, fontsize=17, weight="semibold", color=col, va="top",
            bbox=dict(boxstyle="round,pad=0.35", fc=BG, ec=col, lw=1.2), zorder=20)
    dgoal = math.hypot(x - GOAL[0], y - GOAL[1])
    if done:
        msg = f"Nav2: {r['res']}  ·  stopped {r['err'] * 100:.1f} cm from the goal"
    else:
        msg = f"distance to goal {dgoal:.2f} m"
    ax.text(0.988, 0.93, msg, transform=ax.transAxes, fontsize=16, color=TXT if not done else BG, ha="right", va="top",
            weight="semibold" if done else "normal",
            bbox=dict(boxstyle="round,pad=0.35", fc=BG if not done else col, ec=col, lw=1.2), zorder=20)
    if zoom > 0.6:
        a = (zoom - 0.6) / 0.4
        ax.plot([GOAL[0], x], [GOAL[1], y], color="#facc15", lw=2.5, alpha=a, zorder=12)
        ax.plot([x], [y], "o", color="#facc15", ms=7, alpha=a, zorder=13); ax.plot([GOAL[0]], [GOAL[1]], "o", color=C_GOAL, ms=7, alpha=a, zorder=13)
        ax.text(0.5, 0.08, f"robot centre → goal: {r['err'] * 100:.1f} cm", transform=ax.transAxes, color="#facc15", fontsize=19, weight="bold",
                ha="center", alpha=a, zorder=21, bbox=dict(boxstyle="round,pad=0.3", fc=BG, ec="none", alpha=0.85 * a))
    return k


def render(f):
    if f < NPLAY: tl, zoom = f / FPS * SPEED, 0.0
    else: tl, zoom = DUR, ease((f - NPLAY) / NZOOM)
    fig = fig_new()
    header(fig, "STEP 2 OF 2  ·  NAVIGATION", "Same goal, same map: default vs tuned Nav2",
           "AMCL locates the robot on the saved map, Nav2 plans a path and drives it")
    for n, (name, lab, col, rect) in enumerate([("default", "DEFAULT PARAMS", C_DEF, [0.035, 0.465, 0.66, 0.325]),
                                                 ("tuned", "TUNED PARAMS", C_TUN, [0.035, 0.105, 0.66, 0.325])]):
        r = RUNS[name]; panel(fig, rect, r, lab, col, r["t0"] + tl, zoom)
    badge(fig, 0.695, 0.815, "1× real time" if zoom == 0 else "zoom on goal")
    legend(fig, 0.73, 0.79, [("robot", "Robot (to scale)"), ("lidar", "Live lidar scan"), ("particles", "AMCL particles (pose guesses)"),
                             ("path", "Nav2 global path"), ("trail", "Where it really drove"), ("goal", "Pick-spot goal")])
    fig.text(0.73, 0.45, "Goal 1 of 20 shown", fontsize=15, color=MUTE)
    fig.text(0.73, 0.41, "Default: goal tolerance 25 cm,\nAMCL updates every 25 cm, 60 beams", fontsize=14, color=C_DEF, va="top", linespacing=1.5)
    fig.text(0.73, 0.31, "Tuned: goal tolerance 5 cm,\nAMCL updates every 5 cm, 180 beams", fontsize=14, color=C_TUN, va="top", linespacing=1.5)
    fig.text(0.73, 0.20, "This clip is a re-run of both configs\nrecorded for the visuals; the 20-goal\nresults follow next.", fontsize=13, color=DIM, va="top", linespacing=1.5)
    footer(fig); fig.savefig(f"{OUT}/f{f:04d}.png", facecolor=BG); plt.close(fig)


if __name__ == "__main__":
    fr = range(NTOT) if len(sys.argv) < 2 else [int(a) for a in sys.argv[1:]]
    print({k: (round(v["t_end"] - v["t0"], 1), v["err"], v["res"]) for k, v in RUNS.items()}, NTOT)
    with Pool(6) as p: p.map(render, fr, chunksize=8)
