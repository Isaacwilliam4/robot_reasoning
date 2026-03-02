import cv2
import numpy as np
import matplotlib.pyplot as plt


def make_camera_matrix(fx, fy, cx, cy):
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return K


def draw_trajectory(traj_img, t, scale=40.0):
    """
    Draw x-z trajectory on an image canvas.
    We use x (right) and z (forward) from accumulated translation vector t.
    Monocular VO has unknown metric scale, so this is a relative trajectory.
    """
    h, w = traj_img.shape[:2]
    x = int(w / 2 + scale * t[0, 0])
    z = int(h / 2 + scale * t[2, 0])
    cv2.circle(traj_img, (x, z), 2, (255, 255, 255), -1)
    return traj_img


def main():
    # -------------------------
    # Webcam setup
    # -------------------------
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam. Try a different camera index in VideoCapture().")

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Could not read the first frame from webcam.")

    h, w = frame.shape[:2]

    # -------------------------
    # Camera intrinsics (approx)
    # -------------------------
    # These intrinsics are approximate. For better results, calibrate the webcam and replace them.
    # A reasonable default is fx ~ fy ~ 0.9 * width (in pixels), principal point at image center.
    fx = 0.9 * w
    fy = 0.9 * w
    cx = w / 2.0
    cy = h / 2.0
    K = make_camera_matrix(fx, fy, cx, cy)

    # -------------------------
    # Feature + matcher
    # -------------------------
    orb = cv2.ORB_create(nfeatures=2500, fastThreshold=10)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    prev_kp, prev_des = orb.detectAndCompute(prev_gray, None)

    # -------------------------
    # Pose accumulation
    # -------------------------
    # World-to-camera pose (R, t). We accumulate camera motion in a global frame.
    R_global = np.eye(3, dtype=np.float64)
    t_global = np.zeros((3, 1), dtype=np.float64)

    # Trajectory canvas
    traj = np.zeros((600, 600, 3), dtype=np.uint8)

    # Matplotlib live plot (optional). We keep it simple and use the OpenCV canvas as primary.
    plt.ion()
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.set_title("Monocular VO trajectory (relative scale)")
    ax.set_xlabel("x (arbitrary units)")
    ax.set_ylabel("z (arbitrary units)")
    xs, zs = [], []
    line, = ax.plot(xs, zs)

    print("Controls:")
    print("  q  : quit")
    print("  r  : reset trajectory")
    print("Notes:")
    print("  This is monocular VO, so translation scale is unknown.")
    print("  Expect drift. Texture and lighting matter.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)

        if prev_des is None or des is None or len(prev_kp) < 8 or len(kp) < 8:
            cv2.imshow("Matches", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            prev_gray, prev_kp, prev_des = gray, kp, des
            continue

        matches = bf.match(prev_des, des)
        matches = sorted(matches, key=lambda m: m.distance)

        # Keep best matches
        keep = min(350, len(matches))
        matches = matches[:keep]

        pts_prev = np.float32([prev_kp[m.queryIdx].pt for m in matches])
        pts_curr = np.float32([kp[m.trainIdx].pt for m in matches])

        # Robust essential matrix with RANSAC
        E, mask = cv2.findEssentialMat(
            pts_curr, pts_prev, K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:
            prev_gray, prev_kp, prev_des = gray, kp, des
            cv2.imshow("Matches", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        inliers = mask.ravel().astype(bool)
        pts_prev_in = pts_prev[inliers]
        pts_curr_in = pts_curr[inliers]

        if len(pts_prev_in) < 8:
            prev_gray, prev_kp, prev_des = gray, kp, des
            cv2.imshow("Matches", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # Recover relative pose: R_rel, t_rel (unit-norm translation direction)
        _, R_rel, t_rel, _ = cv2.recoverPose(E, pts_curr_in, pts_prev_in, K)

        # Accumulate pose.
        # Important: monocular translation magnitude is unknown; t_rel is a direction.
        # We apply a fixed step length to make the trajectory visible.
        step = 0.05  # arbitrary units per frame; tune for visualization
        t_global = t_global + (R_global @ (t_rel * step))
        R_global = R_rel @ R_global

        # Draw matches
        match_vis = cv2.drawMatches(
            prev_gray, prev_kp, gray, kp, matches, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        cv2.putText(match_vis, f"Matches: {len(matches)}  Inliers: {int(inliers.sum())}",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # Draw trajectory (x-z)
        traj = draw_trajectory(traj, t_global, scale=120.0)
        traj_disp = traj.copy()
        cv2.putText(traj_disp, "Trajectory (x-z, relative scale)", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        cv2.imshow("Matches", match_vis)
        cv2.imshow("Trajectory", traj_disp)

        # Matplotlib trajectory update
        xs.append(float(t_global[0, 0]))
        zs.append(float(t_global[2, 0]))
        line.set_xdata(xs)
        line.set_ydata(zs)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.001)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            R_global = np.eye(3, dtype=np.float64)
            t_global = np.zeros((3, 1), dtype=np.float64)
            traj[:] = 0
            xs, zs = [], []
            line.set_xdata(xs)
            line.set_ydata(zs)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.001)

        prev_gray, prev_kp, prev_des = gray, kp, des

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
