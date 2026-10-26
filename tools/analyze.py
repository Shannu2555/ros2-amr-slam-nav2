"""Computes map accuracy (slam_toolbox map vs ground-truth grid) and Nav2 goal stats
from the logs in results/, writes results/metrics.json and plots in docs/media/."""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import ndimage
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "amr_sim"))
import world

ROOT = os.path.join(os.path.dirname(__file__), ".."); R = f"{ROOT}/results"; M = f"{ROOT}/docs/media"
os.makedirs(M, exist_ok=True)
OX, OY = 1.0, 1.0
met = {}
# --- map accuracy
z = np.load(f"{R}/mapping_run.npz"); n = len(z["mt"]); m = z[f"map{n-1}"]; res, ox, oy = z["meta"][n - 1]
truth = world.build()
dist_true = ndimage.distance_transform_edt(~truth) * world.RES  # distance to nearest true obstacle
occ = np.argwhere(m >= 65)
wx = ox + OX + (occ[:, 1] + 0.5) * res; wy = oy + OY + (occ[:, 0] + 0.5) * res
c = np.clip((wx / world.RES).astype(int), 0, truth.shape[1] - 1); r = np.clip((wy / world.RES).astype(int), 0, truth.shape[0] - 1)
e = dist_true[r, c]
gt, od = z["gt"], z["od"]
odx, ody = od[:, 1] + OX, od[:, 2] + OY
met["mapping"] = dict(
    map_cells=int(m.size), occupied_cells=int(len(occ)), map_updates=int(n),
    wall_err_median_cm=round(float(np.median(e)) * 100, 1), wall_err_p95_cm=round(float(np.percentile(e, 95)) * 100, 1),
    walls_within_5cm_pct=round(float((e <= 0.05).mean()) * 100, 1),
    drive_length_m=round(float(np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))), 1),
    drive_time_s=round(float(gt[-1, 0] - gt[0, 0]), 1),
    odom_end_drift_m=round(float(np.hypot(odx[-1] - gt[-1, 1], ody[-1] - gt[-1, 2])), 2),
    odom_max_drift_m=round(float(np.max(np.hypot(np.interp(gt[:, 0], od[:, 0], odx) - gt[:, 1], np.interp(gt[:, 0], od[:, 0], ody) - gt[:, 2]))), 2))
# map figure
fig, ax = plt.subplots(figsize=(11, 7.5))
ext = [ox + OX, ox + OX + m.shape[1] * res, oy + OY, oy + OY + m.shape[0] * res]
ax.imshow(np.where(m < 0, 0.35, m / 100.), origin="lower", cmap="gray_r", extent=ext, vmin=0, vmax=1)
ty, tx = np.nonzero(truth ^ ndimage.binary_erosion(truth))
ax.scatter((tx + .5) * world.RES, (ty + .5) * world.RES, s=0.4, c="tab:green", label="true walls (world file)")
ax.plot(gt[:, 1], gt[:, 2], "b", lw=1.2, label="real path (ground truth)")
ax.plot(odx, ody, "r", lw=1.2, label="wheel odometry alone")
ax.set_title(f"slam_toolbox map vs real walls: median wall error {met['mapping']['wall_err_median_cm']} cm, "
             f"{met['mapping']['walls_within_5cm_pct']}% within 5 cm", fontsize=12)
ax.legend(loc="upper right", fontsize=9); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_xlim(-0.2, 12.2); ax.set_ylim(-0.2, 8.2)
fig.tight_layout(); fig.savefig(f"{M}/map_vs_truth.png", dpi=110); plt.close(fig)
# --- goals
runs = {}
for name in ("default", "tuned"):
    p = f"{R}/goals_{name}.json"
    if not os.path.exists(p):
        continue
    g = json.load(open(p)); runs[name] = g
    ok = [x for x in g if x["result"] == "SUCCEEDED"]
    fe = np.array([x["final_err_m"] for x in g])
    contacts = sum(1 for l in open(f"{R}/logs/{name}/sim.log") if "CONTACT" in l)
    met[f"nav_{name}"] = dict(goals=len(g), succeeded=len(ok), contacts=contacts,
        final_err_mean_cm=round(fe.mean() * 100, 1), final_err_max_cm=round(fe.max() * 100, 1),
        within_5cm=int((fe <= 0.05).sum()), within_10cm=int((fe <= 0.10).sum()),
        total_time_s=round(sum(x["time_s"] for x in g), 1), total_path_m=round(sum(x["path_m"] for x in g), 1),
        mean_time_s=round(np.mean([x["time_s"] for x in g]), 1))
json.dump(met, open(f"{R}/metrics.json", "w"), indent=1)
print(json.dumps(met, indent=1))
if len(runs) == 2:
    fig, ax = plt.subplots(figsize=(11, 5.5)); k = np.arange(1, 21)
    for off, (name, col) in zip((-0.2, 0.2), (("default", "tab:red"), ("tuned", "tab:blue"))):
        ax.bar(k + off, [x["final_err_m"] * 100 for x in runs[name]], 0.4, color=col,
               label=f"{name} params: mean {met['nav_'+name]['final_err_mean_cm']} cm")
    ax.axhline(5, color="k", ls="--", lw=1); ax.text(20.6, 5.5, "5 cm", ha="right")
    ax.set_xticks(k); ax.set_xlabel("goal #"); ax.set_ylabel("distance from pick spot when Nav2 said done [cm]")
    ax.set_title("Where the robot actually stopped (ground truth), 20 goals per run"); ax.legend(); ax.grid(axis="y", alpha=.3)
    fig.tight_layout(); fig.savefig(f"{M}/goal_error.png", dpi=110); plt.close(fig)
