"""Title card, results table, bar chart, per-goal chart, closing card. Numbers from results/metrics.json
and results/goals_*.json (the logged 20-goal runs)."""
import json, numpy as np
from theme import *
met = json.load(open(f"{REPO}/results/metrics.json")); A, B, MP = met["nav_default"], met["nav_tuned"], met["mapping"]
GD = json.load(open(f"{REPO}/results/goals_default.json")); GT = json.load(open(f"{REPO}/results/goals_tuned.json"))
os.makedirs("cards", exist_ok=True)


def title():
    fig = fig_new()
    fig.text(0.08, 0.72, "ROS 2 HUMBLE  ·  WAREHOUSE AMR", fontsize=20, color=ACC, weight="semibold")
    fig.text(0.08, 0.53, "Mapping a warehouse with slam_toolbox,\nthen driving 20 pick goals with AMCL + Nav2", fontsize=46,
             color=TXT, weight="bold", linespacing=1.25, va="bottom")
    fig.add_artist(plt.Line2D([0.08, 0.20], [0.47, 0.47], color=ACC, lw=4))
    fig.text(0.08, 0.40, "Shanmukh Narayana Chaganti", fontsize=28, color=TXT, weight="semibold")
    fig.text(0.08, 0.35, "B.Tech Robotics & AI  ·  Amrita School of Engineering, Bengaluru", fontsize=18, color=MUTE)
    fig.text(0.08, 0.10, HONEST, fontsize=14, color=DIM)
    fig.savefig("cards/title.png", facecolor=BG); plt.close(fig)


def table():
    fig = fig_new()
    header(fig, "RESULTS  ·  ALL 20 GOALS", "Nav2 said SUCCEEDED 20/20 both times. The truth didn't agree.",
           "Ground-truth stop error from the logged runs, same map and same 20 goals")
    rows = [("Nav2 result SUCCEEDED", f"{A['succeeded']}/20", f"{B['succeeded']}/20"),
            ("Mean stop error", f"{A['final_err_mean_cm']} cm", f"{B['final_err_mean_cm']} cm"),
            ("Worst stop error", f"{A['final_err_max_cm']:.1f} cm", f"{B['final_err_max_cm']} cm"),
            ("Stops within 5 cm", f"{A['within_5cm']}/20", f"{B['within_5cm']}/20"),
            ("Wall contacts", f"{A['contacts']}", f"{B['contacts']}"),
            ("Total time, 20 goals", f"{A['total_time_s'] / 60:.1f} min", f"{B['total_time_s'] / 60:.1f} min  (+{(B['total_time_s'] - A['total_time_s']) / 60:.1f} min)")]
    x0, x1, x2, x3 = 0.12, 0.50, 0.68, 0.88; y = 0.70
    fig.text(x1, y, "Default", fontsize=20, color=C_DEF, weight="semibold"); fig.text(x2, y, "Tuned", fontsize=20, color=C_TUN, weight="semibold")
    fig.add_artist(plt.Line2D([x0, x3], [y - 0.025, y - 0.025], color="#334155", lw=1.5))
    for i, (a, b, c) in enumerate(rows):
        yy = y - 0.085 * (i + 1)
        if i in (1, 2, 3):
            fig.add_artist(Rectangle((x0 - 0.01, yy - 0.03), x3 - x0 + 0.02, 0.07, color="#111c27", zorder=-1))
        fig.text(x0, yy, a, fontsize=22, color=TXT, va="center"); fig.text(x1, yy, b, fontsize=24, color=C_DEF, va="center")
        fig.text(x2, yy, c, fontsize=24, color=C_TUN, weight="bold", va="center")
    fig.text(x0, 0.10, f"Mapping: {MP['drive_length_m']} m drive, {MP['map_updates']} map updates, {MP['walls_within_5cm_pct']}% of mapped wall cells within 5 cm of the real walls",
             fontsize=16, color=MUTE)
    footer(fig); fig.savefig("cards/table.png", facecolor=BG); plt.close(fig)


def axstyle(ax):
    ax.set_facecolor(BG)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.tick_params(labelsize=15); ax.grid(axis="y", color="#1f2a36", lw=1); ax.set_axisbelow(True)


def bars():
    fig = fig_new()
    header(fig, "RESULTS  ·  STOP ERROR", "Mean stop error 21.0 cm → 4.9 cm", "Distance between the goal and where the robot really stopped (ground truth), 20 goals")
    ax = fig.add_axes([0.10, 0.13, 0.80, 0.62]); axstyle(ax)
    cats = ["Mean", "Worst"]; a = [A["final_err_mean_cm"], A["final_err_max_cm"]]; b = [B["final_err_mean_cm"], B["final_err_max_cm"]]
    xx = np.arange(2); w = 0.34
    r1 = ax.bar(xx - w / 2 - 0.01, a, w, color=C_DEF, label="Default params"); r2 = ax.bar(xx + w / 2 + 0.01, b, w, color=C_TUN, label="Tuned params")
    for rr, v in list(zip(r1, a)) + list(zip(r2, b)):
        ax.text(rr.get_x() + rr.get_width() / 2, v + 0.6, f"{v:.1f} cm", ha="center", fontsize=20, weight="semibold", color=TXT)
    ax.axhline(25, color=C_GOAL, ls="--", lw=1.5); ax.text(1.45, 25.6, "default goal tolerance 25 cm", color=C_GOAL, fontsize=15, ha="left")
    ax.axhline(5, color=C_GOAL, ls=":", lw=1.5); ax.text(1.45, 5.6, "tuned goal tolerance 5 cm", color=C_GOAL, fontsize=15, ha="left")
    ax.set_xticks(xx, cats, fontsize=18); ax.set_ylabel("Stop error (cm)", fontsize=17); ax.set_ylim(0, 36); ax.set_xlim(-0.6, 2.25)
    ax.legend(frameon=False, fontsize=16, loc="upper right", labelcolor=TXT)
    footer(fig); fig.savefig("cards/bars.png", facecolor=BG); plt.close(fig)


def pergoal():
    fig = fig_new()
    header(fig, "RESULTS  ·  EVERY GOAL", f"Within 5 cm: {A['within_5cm']}/20 → {B['within_5cm']}/20", "Stop error for each of the 20 pick goals, ground truth")
    ax = fig.add_axes([0.10, 0.13, 0.80, 0.62]); axstyle(ax)
    g = np.arange(1, 21); a = [x["final_err_m"] * 100 for x in GD]; b = [x["final_err_m"] * 100 for x in GT]
    ax.plot(g, a, "-o", color=C_DEF, lw=2.5, ms=8, label=f"Default params (mean {A['final_err_mean_cm']} cm)")
    ax.plot(g, b, "-o", color=C_TUN, lw=2.5, ms=8, label=f"Tuned params (mean {B['final_err_mean_cm']} cm)")
    ax.axhspan(0, 5, color=C_TUN, alpha=0.08); ax.text(20.4, 2.5, "≤ 5 cm", color=C_TUN, fontsize=14, va="center")
    ax.set_xticks(g); ax.set_xlabel("Goal number", fontsize=17); ax.set_ylabel("Stop error (cm)", fontsize=17)
    ax.set_xlim(0.5, 21.2); ax.set_ylim(0, 36); ax.legend(frameon=False, fontsize=16, loc="upper right", labelcolor=TXT, ncol=2)
    footer(fig); fig.savefig("cards/pergoal.png", facecolor=BG); plt.close(fig)


def outro():
    fig = fig_new()
    fig.text(0.08, 0.70, "What I took from it", fontsize=20, color=ACC, weight="semibold")
    fig.text(0.08, 0.65, "Don't trust the status message.\nMeasure against ground truth.", fontsize=46, color=TXT, weight="bold", linespacing=1.25, va="top")
    fig.text(0.08, 0.36, "Code, configs, saved map and raw logs:  github.com/Shannu2555/ros2-amr-slam-nav2", fontsize=20, color=MUTE)
    fig.text(0.08, 0.30, "Shanmukh Narayana Chaganti", fontsize=24, color=TXT, weight="semibold")
    fig.text(0.08, 0.10, HONEST, fontsize=14, color=DIM)
    fig.savefig("cards/outro.png", facecolor=BG); plt.close(fig)


for fn in (title, table, bars, pergoal, outro): fn()
print([ (x['final_err_m']) for x in GD][:3])
