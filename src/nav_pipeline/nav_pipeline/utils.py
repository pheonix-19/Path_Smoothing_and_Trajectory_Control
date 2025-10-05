import numpy as np

def dist(a, b):
    a = np.asarray(a); b = np.asarray(b)
    return np.linalg.norm(a - b)

def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(sum(dist(points[i], points[i+1]) for i in range(len(points)-1)))

def cumulative_lengths(points):
    L = [0.0]
    for i in range(1, len(points)):
        L.append(L[-1] + dist(points[i-1], points[i]))
    return np.array(L, dtype=float)

def resample_by_arclength(points, ds):
    """Return points resampled by arclength spacing ds."""
    if len(points) < 2:
        return np.array(points, dtype=float)
    pts = np.array(points, dtype=float)
    s = cumulative_lengths(pts)
    total = s[-1]
    if total == 0.0:
        return pts
    s_new = np.arange(0.0, total + 1e-9, ds)
    x = np.interp(s_new, s, pts[:,0])
    y = np.interp(s_new, s, pts[:,1])
    return np.vstack([x,y]).T
