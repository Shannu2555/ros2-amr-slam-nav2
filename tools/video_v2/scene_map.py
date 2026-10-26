"""Scene 1: slam_toolbox mapping, from results/mapping_run.npz (every /map update + ground-truth pose).
Lidar rays are re-cast from the logged pose with the simulator's own lidar model."""
import os, sys, numpy as np
from multiprocessing import Pool
from theme import *
z = np.load(f"{REPO}/results/mapping_run.npz"); mt = z["mt"]; gt = z["gt"]; n = len(mt)
maps = [z[f"map{k}"] for k in range(n)]; meta = z["meta"]
SPEED = 6
T = np.arange(gt[0, 0], gt[-1, 0], SPEED / FPS)
T = np.concatenate([T, [gt[-1, 0]] * 45])   # hold final map 1.5 s
dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))])
TH_U = np.unwrap(gt[:, 3])


def pose_at(g, t):  # interpolate between logged poses so the motion is smooth
    return np.interp(t, g[:, 0], g[:, 1]), np.interp(t, g[:, 0], g[:, 2]), np.interp(t, g[:, 0], TH_U)


OUT = "frames/map"; os.makedirs(OUT, exist_ok=True)


def render(f):
    t = T[f]; k = max(0, np.searchsorted(mt, t, "right") - 1); i = min(np.searchsorted(gt[:, 0], t), len(gt) - 1)
    x, y, th = pose_at(gt, t)
    fig = fig_new()
    header(fig, "STEP 1 OF 2  ·  MAPPING", "The robot builds the warehouse map as it drives",
           "slam_toolbox fuses each lidar scan with wheel odometry")
    ax = fig.add_axes([0.035, 0.09, 0.66, 0.71]); style_ax(ax, (-0.3, 12.3), (-0.3, 8.3))
    if t >= mt[0]: draw_map(ax, maps[k], meta[k])
    ax.plot(gt[:i + 1, 1], gt[:i + 1, 2], color=C_TRAIL, lw=2, alpha=0.55, zorder=2)
    draw_lidar(ax, x, y, th, raycast(x, y, th))
    draw_robot(ax, x, y, th)
    badge(fig, 0.69, 0.765, f"{SPEED}× speed")
    legend(fig, 0.73, 0.78, [("robot", "Robot (to scale, 0.36 m)"), ("lidar", "Lidar rays + wall hits"),
                             ("trail", "Path driven"), ("wall", "Mapped wall"), ("free", "Mapped free space")])
    st = [("Map updates", f"{k + 1 if t >= mt[0] else 0} / {n}"), ("Distance driven", f"{dist[i]:.1f} m"),
          ("Time", f"{t - gt[0, 0]:.0f} s")]
    for j, (a, b) in enumerate(st):
        fig.text(0.73, 0.43 - j * 0.075, a, fontsize=15, color=MUTE); fig.text(0.73, 0.40 - j * 0.075, b, fontsize=24, color=TXT, weight="semibold")
    fig.text(0.73, 0.12, "Lidar: 360 beams, 8 m range", fontsize=14, color=DIM)
    footer(fig); fig.savefig(f"{OUT}/f{f:04d}.png", facecolor=BG); plt.close(fig)


if __name__ == "__main__":
    fr = range(len(T)) if len(sys.argv) < 2 else [int(a) for a in sys.argv[1:]]
    with Pool(6) as p: p.map(render, fr, chunksize=8)
    print(len(T), "frames")
