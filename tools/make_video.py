"""Builds docs/media/amr_demo.mp4 from the real logs: mapping timelapse (every /map
update slam_toolbox published), the tuned Nav2 run replayed from ground truth, then result slides."""
import json, os, shutil, subprocess, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "amr_sim"))


ROOT = os.path.join(os.path.dirname(__file__), ".."); R = f"{ROOT}/results"; M = f"{ROOT}/docs/media"
OX, OY = 1.0, 1.0; fps = 30
plt.rcParams.update({"font.size": 13})


def frames_dir(name):
    d = f"/tmp/amr_{name}"; shutil.rmtree(d, ignore_errors=True); os.makedirs(d); return d


def encode(d, out):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", f"{d}/f%04d.png",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-vf", "scale=1080:1080", out], check=True)


def draw_map(ax, m, meta):
    res, ox, oy = meta
    ext = [ox + OX, ox + OX + m.shape[1] * res, oy + OY, oy + OY + m.shape[0] * res]
    ax.imshow(np.where(m < 0, 0.3, m / 100.), origin="lower", cmap="gray_r", extent=ext, vmin=0, vmax=1)
    ax.set_xlim(-0.3, 12.3); ax.set_ylim(-0.3, 8.3); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])


# A: mapping timelapse
z = np.load(f"{R}/mapping_run.npz"); mt = z["mt"]; gt = z["gt"]; n = len(mt)
d = frames_dir("map"); fig = plt.figure(figsize=(10.8, 10.8), dpi=100)
T = np.linspace(gt[0, 0], gt[-1, 0], fps * 16)
for f, t in enumerate(T):
    k = max(0, np.searchsorted(mt, t) - 1); i = np.searchsorted(gt[:, 0], t)
    fig.clf(); ax = fig.add_axes([0.03, 0.18, 0.94, 0.64])
    draw_map(ax, z[f"map{k}"], z["meta"][k])
    ax.plot(gt[:i, 1], gt[:i, 2], "b", lw=1.5); ax.add_patch(Circle((gt[i - 1, 1], gt[i - 1, 2]), 0.18, color="tab:orange"))
    fig.text(0.5, 0.93, "Step 1: mapping the warehouse with slam_toolbox", ha="center", fontsize=22, weight="bold")
    fig.text(0.5, 0.875, f"ROS 2 Humble, 2D lidar 360 beams, map update {k + 1}/{n}, sim time {t - gt[0, 0]:5.0f} s", ha="center", fontsize=15)
    fig.text(0.5, 0.08, "white = free, black = wall, grey = not seen yet", ha="center", fontsize=15, color="#444")
    fig.savefig(f"{d}/f{f:04d}.png")
for j in range(1, fps):
    shutil.copy(f"{d}/f{f:04d}.png", f"{d}/f{f + j:04d}.png")
encode(d, "/tmp/amr_A.mp4"); plt.close(fig)

# B: tuned Nav2 run replay
zn = np.load(f"{R}/nav_tuned.npz"); g = zn["gt"]; goals = json.load(open(f"{R}/goals_tuned.json"))
k = len(zn["mt"]) - 1; m, meta = zn[f"map{k}"], zn["meta"][k]
cum = np.cumsum([x["time_s"] for x in goals]); t0 = g[0, 0]; tdone = g[-1, 0]
# goal index by time: first goal starts when the robot first moves
mv = np.argmax(np.hypot(g[:, 1] - g[0, 1], g[:, 2] - g[0, 2]) > 0.02); tstart = g[mv, 0] - 0.5
d = frames_dir("nav"); fig = plt.figure(figsize=(10.8, 10.8), dpi=100)
T = np.linspace(tstart, min(tdone, tstart + cum[-1] + 1), fps * 20)
for f, t in enumerate(T):
    i = np.searchsorted(g[:, 0], t); gi = min(int(np.searchsorted(cum, t - tstart)), len(goals) - 1)
    fig.clf(); ax = fig.add_axes([0.03, 0.18, 0.94, 0.64]); draw_map(ax, m, meta)
    for j, x in enumerate(goals):
        c = "tab:green" if j < gi else ("tab:red" if j == gi else "#bbbbbb")
        ax.plot(x["x"], x["y"], "s", ms=9, color=c); ax.text(x["x"] + 0.12, x["y"] + 0.12, str(j + 1), fontsize=9)
    s = max(0, i - 600)
    ax.plot(g[s:i, 1], g[s:i, 2], "b", lw=1.5); ax.add_patch(Circle((g[i - 1, 1], g[i - 1, 2]), 0.18, color="tab:orange"))
    done = sum(1 for x in goals[:gi])
    fig.text(0.5, 0.93, "Step 2: AMCL + Nav2, 20 pick-station goals", ha="center", fontsize=22, weight="bold")
    fig.text(0.5, 0.875, f"goal {gi + 1}/20   (replayed from the ground-truth log, sim time {t - tstart:4.0f} s)", ha="center", fontsize=15)
    fig.text(0.5, 0.08, f"reached: {done}   last stop error: " + (f"{goals[gi - 1]['final_err_m'] * 100:.1f} cm" if gi else "-"),
             ha="center", fontsize=17, weight="bold", color="tab:blue")
    fig.savefig(f"{d}/f{f:04d}.png")
for j in range(1, fps):
    shutil.copy(f"{d}/f{f:04d}.png", f"{d}/f{f + j:04d}.png")
encode(d, "/tmp/amr_B.mp4"); plt.close(fig)

# C: slides
met = json.load(open(f"{R}/metrics.json")); a, b, mp = met["nav_default"], met["nav_tuned"], met["mapping"]
fig = plt.figure(figsize=(10.8, 10.8), dpi=100)
fig.text(0.5, 0.88, "Results: 20 goals, same map, same goals", ha="center", fontsize=25, weight="bold")
rows = [("", "default params", "tuned params"),
        ("Nav2 said SUCCEEDED", f"{a['succeeded']}/20", f"{b['succeeded']}/20"),
        ("mean stop error (truth)", f"{a['final_err_mean_cm']} cm", f"{b['final_err_mean_cm']} cm"),
        ("worst stop error", f"{a['final_err_max_cm']} cm", f"{b['final_err_max_cm']} cm"),
        ("stops within 5 cm", f"{a['within_5cm']}/20", f"{b['within_5cm']}/20"),
        ("wall contacts", f"{a['contacts']}", f"{b['contacts']}"),
        ("total time for 20 goals", f"{a['total_time_s'] / 60:.1f} min", f"{b['total_time_s'] / 60:.1f} min")]
for k2, (x, y, w) in enumerate(rows):
    yy = 0.76 - k2 * 0.075; wt = "bold" if k2 == 0 else "normal"
    fig.text(0.05, yy, x, fontsize=19, weight=wt); fig.text(0.55, yy, y, fontsize=19, weight=wt); fig.text(0.78, yy, w, fontsize=19, weight=wt)
fig.text(0.05, 0.18, f"Map: median wall error {mp['wall_err_median_cm']} cm, {mp['walls_within_5cm_pct']}% of wall cells within 5 cm", fontsize=16)
fig.text(0.05, 0.14, f"Wheel odometry alone drifted up to {mp['odom_max_drift_m']} m over the {mp['drive_length_m']} m mapping drive", fontsize=16)
fig.text(0.5, 0.06, "All numbers from the logs in the repo (results/)", ha="center", fontsize=13, color="gray")
fig.savefig("/tmp/amr_slide_table.png"); plt.close(fig)
parts = ["/tmp/amr_A.mp4", "/tmp/amr_B.mp4"]
for k2, s in enumerate(["/tmp/amr_slide_table.png", f"{M}/goal_error.png", f"{M}/map_vs_truth.png"]):
    out = f"/tmp/amr_s{k2}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-t", "4.5", "-i", s, "-r", str(fps),
                    "-vf", "scale=1080:1080:force_original_aspect_ratio=decrease,pad=1080:1080:(ow-iw)/2:(oh-ih)/2:white",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", out], check=True)
    parts.append(out)
open("/tmp/amr_parts.txt", "w").write("".join(f"file '{p}'\n" for p in parts))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "/tmp/amr_parts.txt",
                "-c", "copy", f"{M}/amr_demo.mp4"], check=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", "/tmp/amr_A.mp4", "-vf", "fps=8,scale=480:-1:flags=lanczos",
                f"{M}/mapping.gif"], check=True)
shutil.copy("/tmp/amr_slide_table.png", f"{M}/results_table.png")
print("ok")
