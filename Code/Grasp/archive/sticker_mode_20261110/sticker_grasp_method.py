"""ARCHIVED 2026-11-10: the sticker mode as it was in grasp_node.py (class GraspNode method). Not importable on its own: it needs the node's self._close_frame, self._color_bgr, self.cfg, self.last, self._publish, WORK, housing_footprint (still in housing_grasp/color_marker.py, used by the cloud mode), marker_points and report_sticker_grasp (below).
Restore: paste it back into the class, import report_sticker_grasp, register ("sticker_grasp", self.sticker_grasp) in the services and add sticker=self.sticker_grasp to the mode dict of /grasp/pick and run_pipeline.py."""

    def sticker_grasp(self):
        """Grasp straight from the marker sticker (user decision 2026-11-07): no depth-measured housing, no collision / optimizer checks.
        Jaw axis across the housing footprint seen in the colour image, TCP `grasp_drop_m` below the sticker top, approach straight down."""
        depth, K, _, T_BC = self._close_frame()
        color, g, gc = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"]
        gp = marker_points(depth, color, K, T_BC, g, self.target)   # only the marker blob nearest to the scan target
        if len(gp) < g["min_marker_points"]:
            raise RuntimeError("only %d marker points in the close view: camera over the sticker? (/grasp/view)" % len(gp))
        c = np.median(gp, 0)  # median: depth streaks on the glossy head would pull a mean
        fp = housing_footprint(color, depth, K, T_BC, c, g)
        jaw = np.r_[fp["jaw_axis"], 0.0]
        jaw /= np.linalg.norm(jaw)
        z_G = np.array([0.0, 0.0, -1.0])
        open_w = min(fp["width_m"] + 2 * gc["open_margin_m"], self.cfg["gripper"]["max_open_m"])
        p = np.array([c[0], c[1], c[2] - g["grasp_drop_m"]])
        grasps = []
        for flip in (1, -1):
            y = flip * jaw
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = np.stack([np.cross(y, z_G), y, z_G], 1), p
            Tp, Tl = T.copy(), T.copy()
            Tp[:3, 3] -= z_G * gc["standoff_m"]
            Tl[:3, 3] -= z_G * gc["lift_m"]
            grasps.append(dict(T_BG=T, T_pre=Tp, T_lift=Tl, open_width_m=open_w, expected_closed_width_m=fp["width_m"], zone_offset_m=0.0, flip=flip, cost=0.0))
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=gp, cluster=gp)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="sticker", sticker_centroid_median_m=c, n_marker_points=len(gp), footprint=fp, jaw_axis=jaw, open_width_m=open_w,
            tcp_m=p, grasp_drop_m=g["grasp_drop_m"], grasps=[dict(flip=q["flip"], T_BG=q["T_BG"]) for q in grasps]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_sticker_grasp(run / "plots", gp, fp, c, p, jaw, open_w)
        self._publish(self.last)
        return "sticker median (%.1f, %.1f, %.1f) mm, %d pts; housing footprint %.1f x %.1f mm; jaw axis (%.2f, %.2f); opening %.1f mm; TCP (%.1f, %.1f, %.1f) mm (run %s)" % (
            *(c * 1e3), len(gp), fp["length_m"] * 1e3, fp["width_m"] * 1e3, jaw[0], jaw[1], open_w * 1e3, *(p * 1e3), run.name)
