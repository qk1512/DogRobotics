import numpy as np
from collections import deque

def median_filter_1d(x, k=5):
    k = max(1, k|1)  # k lẻ
    pad = k//2
    xp = np.pad(x, (pad, pad), mode='edge')
    return np.array([np.median(xp[i:i+k]) for i in range(len(x))])

class LidarSmoother:
    def __init__(self, n_angles=360, alpha=0.1, med_win=5):
        self.alpha = alpha
        self.med_win = med_win
        self.state = None
        self.n = n_angles

    def __call__(self, distances_mm):
        d = np.asarray(distances_mm, dtype=float)
        # clamp/outlier
        d[(d < 100) | (d > 12000) | ~np.isfinite(d)] = np.nan
        # điền nan bằng lân cận
        isn = np.isnan(d)
        if isn.any():
            d[isn] = np.interp(np.flatnonzero(isn), np.flatnonzero(~isn), d[~isn])
        # median theo góc
        d = median_filter_1d(d, k=self.med_win)
        # EMA theo thời gian
        if self.state is None:
            self.state = d
        else:
            self.state = self.alpha * d + (1 - self.alpha) * self.state
        return self.state  # mm


class EMAArray:
    def __init__(self, alpha=0.3, size=360):
        self.alpha = alpha
        self.values = np.full(size, np.nan)  # lưu giá trị mượt hóa

    def update(self, arr):
        if np.isnan(self.values).all():
            self.values = np.array(arr, dtype=float)
        else:
            self.values = self.alpha * np.array(arr, dtype=float) + (1 - self.alpha) * self.values
        return self.values
