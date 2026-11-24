"""Run folder for the insertion servo, same layout as Media/RULES.md:
config.json, trajectory.csv, metrics.json, README.md, screenshots/, videos/servo.mp4, plots/ (controller.log is written by run_insert.sh).
  python3 ros2_iface/run_record.py RUN_FOLDER        redraw the plots of a finished run"""
import csv, json, pathlib, subprocess, sys, time
import cv2, numpy as np

COLUMNS = ["t_s", "stage", "side_px", "cx_px", "cy_px", "goal_cx_px", "goal_cy_px", "goal_side_px",
           "off_x_mm", "off_y_mm", "off_z_mm", "to_go_mm", "cmd_x_mm", "cmd_y_mm", "cmd_z_mm",
           "tool_x_mm", "tool_y_mm", "tool_z_mm"] + ["q%d_deg" % i for i in range(1, 7)]


def git_state(root):
    try:
        commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=root, text=True).strip()
        dirty = subprocess.check_output(["git", "status", "--porcelain", "--", "."], cwd=root, text=True).strip()
        return dict(commit=commit, uncommitted_files_in_IBVS=dirty.splitlines())
    except Exception:
        return dict(commit="unknown")


class Recorder:
    def __init__(self, folder, config):
        self.dir = pathlib.Path(folder)
        for sub in ("plots", "screenshots", "videos"):
            (self.dir / sub).mkdir(parents=True, exist_ok=True)
        (self.dir / "config.json").write_text(json.dumps(config, indent=2, default=str) + "\n")
        self.config, self.t0, self.rows, self.video, self.first = config, time.time(), 0, None, True
        self.file = open(self.dir / "trajectory.csv", "w", newline="")
        self.writer = csv.writer(self.file); self.writer.writerow(COLUMNS)
        self.last_to_go = None

    def step(self, stage, color, c, goal, d, side, cmd, tool_mm, q_deg):
        """One observation: marker corners c, goal {c, side}, camera offset d (m), commanded base-frame step cmd (m) or None."""
        n = float(np.linalg.norm(d))
        cmd = [float("nan")] * 3 if cmd is None else list(np.asarray(cmd) * 1e3)
        self.writer.writerow([round(time.time() - self.t0, 3), stage, round(side, 2), *np.round(c.mean(0), 1), *np.round(goal["c"], 1), goal["side"],
                              *np.round(np.asarray(d) * 1e3, 2), round(n * 1e3, 2), *np.round(cmd, 2), *np.round(tool_mm, 1), *np.round(q_deg, 2)])
        self.file.flush()
        self.rows += 1; self.last_to_go = n * 1e3
        img = color.copy()
        cv2.polylines(img, [c.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        gx, gy, h = goal["c"][0], goal["c"][1], goal["side"] / 2
        cv2.rectangle(img, (int(gx - h), int(gy - h)), (int(gx + h), int(gy + h)), (0, 0, 255), 2)
        cv2.putText(img, "%s  %.0f px  %.1f mm to go" % (stage, side, n * 1e3), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.video is None:
            self.video = cv2.VideoWriter(str(self.dir / "videos" / "servo.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (img.shape[1], img.shape[0]))
        self.video.write(img)
        if self.first:
            cv2.imwrite(str(self.dir / "screenshots" / "first.png"), img); self.first = False
        self.last_img = img

    def close(self, outcome, message=""):
        if self.video is not None:
            self.video.release()
        if self.rows:
            cv2.imwrite(str(self.dir / "screenshots" / "last.png"), self.last_img)
        self.file.close()
        metrics = dict(outcome=outcome, message=message, duration_s=round(time.time() - self.t0, 1), observations=self.rows,
                       final_distance_to_go_mm=None if self.last_to_go is None else round(self.last_to_go, 1))
        (self.dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
        c = self.config
        (self.dir / "README.md").write_text(
            "# Insertion servo run: %s\n\nOutcome: **%s** %s\n\n- Command: `%s`\n- Wire: %s, mode: %s\n- Code: %s\n- Not a verified insertion: the servo only brings the plug to the socket entrance; "
            "no seating sensor was used.\n- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), "
            "screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.\n" % (
                c["started"], outcome, message, " ".join(c["command"]), c["wire"], "continuous" if c["args"]["continuous"] else "stop-and-observe", c["git"]))
        try:
            make_plots(self.dir)
        except Exception as e:
            print("plots failed:", e, flush=True)


def load(folder):
    with open(pathlib.Path(folder) / "trajectory.csv") as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    return rows, col


def make_plots(folder):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    folder = pathlib.Path(folder)
    rows, col = load(folder)
    if not rows:
        return
    t, stages = col("t_s"), [r["stage"] for r in rows]
    switch = [t[i] for i in range(1, len(t)) if stages[i] != stages[i - 1]]
    plt.rcParams.update({"font.size": 12, "axes.grid": True, "grid.alpha": 0.3})

    def save(fig, name):
        fig.tight_layout(); fig.savefig(folder / "plots" / name, dpi=130); plt.close(fig)

    def marks(ax):
        for s in switch:
            ax.axvline(s, color="gray", ls="--", lw=1)

    fig, ax = plt.subplots(figsize=(9, 5))
    for k, lab in (("off_x_mm", "x (image right)"), ("off_y_mm", "y (image down)"), ("off_z_mm", "z (toward wall)")):
        ax.plot(t, col(k), marker=".", label=lab)
    marks(ax); ax.set(title="Camera offset to the goal, camera frame (dashed: stage change)", xlabel="time (s)", ylabel="offset (mm)"); ax.legend()
    save(fig, "offset_to_goal.png")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, col("to_go_mm"), marker=".", color="k"); marks(ax)
    ax.set(title="Distance still to go to the goal (dashed: stage change)", xlabel="time (s)", ylabel="distance (mm)")
    save(fig, "distance_to_go.png")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, col("side_px"), marker=".", label="marker side now")
    ax.plot(t, col("goal_side_px"), ls="--", color="r", label="goal side")
    marks(ax); ax.set(title="Marker size in the image (grows as the camera closes in)", xlabel="time (s)", ylabel="side length (px)"); ax.legend()
    save(fig, "marker_size.png")

    fig, ax = plt.subplots(figsize=(9, 5.4))
    ax.plot(col("cx_px"), col("cy_px"), marker=".", label="marker centre")
    ax.scatter(col("cx_px")[0], col("cy_px")[0], c="g", s=80, label="start", zorder=3)
    ax.scatter(col("goal_cx_px"), col("goal_cy_px"), c="r", marker="x", s=80, label="goal centre", zorder=3)
    xs, ys = np.r_[col("cx_px"), col("goal_cx_px")], np.r_[col("cy_px"), col("goal_cy_px")]
    pad = max(xs.max() - xs.min(), ys.max() - ys.min(), 40) * 0.2
    ax.set(xlim=(xs.min() - pad, xs.max() + pad), ylim=(ys.max() + pad, ys.min() - pad), title="Marker centre path in the image (zoomed; image is 848 x 480 px)", xlabel="x (px)", ylabel="y (px, down)")
    ax.set_aspect("equal"); ax.legend()
    save(fig, "marker_image_path.png")

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for ax, k, lab in zip(axes, ("tool_x_mm", "tool_y_mm", "tool_z_mm"), ("x (toward wall)", "y (sideways)", "z (height)")):
        ax.plot(t, col(k), marker=".", color="C0"); marks(ax); ax.set_ylabel(lab + " (mm)")
    axes[0].set_title("Tool position in the robot base frame"); axes[-1].set_xlabel("time (s)")
    save(fig, "tool_position.png")

    fig, ax = plt.subplots(figsize=(9, 5))
    for i in range(1, 7):
        ax.plot(t, col("q%d_deg" % i), label="joint %d" % i)
    marks(ax); ax.set(title="Joint angles", xlabel="time (s)", ylabel="angle (deg)"); ax.legend(ncol=3)
    save(fig, "joints.png")

    cmd = np.stack([col("cmd_x_mm"), col("cmd_y_mm"), col("cmd_z_mm")])
    if not np.isnan(cmd).all():
        fig, ax = plt.subplots(figsize=(9, 5))
        for row, lab in zip(cmd, ("x (toward wall)", "y (sideways)", "z (height)")):
            ax.plot(t, row, marker=".", label=lab)
        marks(ax); ax.set(title="Commanded move per step, robot base frame", xlabel="time (s)", ylabel="step (mm)"); ax.legend()
        save(fig, "commanded_step.png")


if __name__ == "__main__":
    make_plots(sys.argv[1])
