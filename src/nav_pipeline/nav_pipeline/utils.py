import numpy as np


def resample_by_arclength(points, ds=0.05):
    """
    Resample a 2D polyline (list of [x, y]) by equal arc-length spacing.

    Parameters
    ----------
    points : (N,2) array-like
        Input coordinates (x, y)
    ds : float
        Desired spacing between successive points

    Returns
    -------
    new_pts : (M,2) ndarray
        Resampled coordinates at uniform arc length
    """
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 2:
        return pts

    # --- Remove NaNs and duplicates first ---
    diffs = np.diff(pts, axis=0)
    seglen = np.linalg.norm(diffs, axis=1)
    keep = np.concatenate([[True], seglen > 1e-8])
    pts = pts[keep]

    if pts.shape[0] < 2:
        return pts

    # --- Compute cumulative arc length ---
    diffs = np.diff(pts, axis=0)
    seglen = np.linalg.norm(diffs, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]

    if total < 1e-9:
        return pts

    # --- Create uniform arclength samples ---
    s_new = np.arange(0.0, total + 1e-9, ds)
    x_new = np.interp(s_new, s, pts[:, 0])
    y_new = np.interp(s_new, s, pts[:, 1])
    new_pts = np.stack([x_new, y_new], axis=1)

    # --- Remove any residual near-zero segments ---
    diffs2 = np.linalg.norm(np.diff(new_pts, axis=0), axis=1)
    keep2 = np.concatenate([[True], diffs2 > 1e-8])
    new_pts = new_pts[keep2]

    return new_pts


# Optional helper for computing total path length (used for debugging)
def path_length(points):
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    seglen = np.linalg.norm(diffs, axis=1)
    return float(np.sum(seglen))
