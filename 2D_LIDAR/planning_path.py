import math
from collections import deque

# Actions
STOP = "STOP"
FORWARD = "FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"

def _norm360(a):
    return a % 360

def _angle_in_sector(angle, start, end):
    # all in [0,360)
    angle = _norm360(angle)
    start = _norm360(start)
    end = _norm360(end)
    if start <= end:
        return start <= angle <= end
    else:
        # wrap
        return angle >= start or angle <= end

class SimpleReactivePlanner:
    def __init__(self,
                 safe_dist=0.5,           # meters; any return < safe_dist marks blocked
                 fov_min=-90, fov_max=90, # field of view relative to lidar 0° (deg)
                 resolution_deg=1,        # angular resolution in degrees
                 min_gap_deg=10,          # minimum gap width to consider passable
                 forward_threshold_deg=8, # if target within this, go forward
                 hysteresis_len=3):
        self.safe_dist = safe_dist
        # store FOV in 0..360 format (start,end)
        self.fov_start = _norm360(fov_min)
        self.fov_end   = _norm360(fov_max)
        self.res = resolution_deg
        self.min_gap = max(1, int(min_gap_deg // resolution_deg))
        self.forward_thresh = forward_threshold_deg
        self.history = deque(maxlen=hysteresis_len)
        self.last_action = STOP

    def build_occupancy_by_angle(self, scan_points):
        """Return list blocked[i] for angles 0..360-1 with resolution self.res:
           blocked[j] True means obstacle within safe_dist at that angle bin.
           Angles are lidar frame 0..360 (0 = forward), increase clockwise.
        """
        n_bins = int(360 // self.res)
        blocked = [False] * n_bins

        for ang, dist_mm in scan_points:
            if dist_mm <= 0:
                continue
            d = dist_mm / 1000.0
            if d <= self.safe_dist:
                # map to bin
                a = _norm360(ang)
                idx = int(a // self.res) % n_bins
                blocked[idx] = True

        return blocked

    def extract_fov_bins(self):
        """Return list of bins indices that lie inside the configured FOV (in increasing angle order CW)."""
        res = self.res
        n_bins = int(360 // res)
        bins = []
        # iterate full circle but pick those inside FOV
        for i in range(n_bins):
            ang = i * res
            if _angle_in_sector(ang, self.fov_start, self.fov_end):
                bins.append(i)
        return bins

    def find_largest_gap(self, blocked, fov_bins):
        """Return (start_bin, end_bin, width_bins). bins are indices in blocked."""
        if not fov_bins:
            return None
        # build list of 0/1 for fov sequence
        seq = [0 if not blocked[i] else 1 for i in fov_bins]
        # find longest run of zeros
        best_len = 0; best_range = None
        cur_len = 0; cur_start = 0
        for i, val in enumerate(seq):
            if val == 0:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
            else:
                if cur_len > best_len:
                    best_len = cur_len
                    best_range = (cur_start, i-1)
                cur_len = 0
        # check tail
        if cur_len > best_len:
            best_len = cur_len
            best_range = (cur_start, len(seq)-1)
        # If no gap found
        if best_range is None:
            return None
        # map back to bin indices
        s_idx = fov_bins[best_range[0]]
        e_idx = fov_bins[best_range[1]]
        return (s_idx, e_idx, best_len)

    def bin_to_angle(self, bin_idx):
        # center angle of bin
        return (bin_idx * self.res + self.res/2.0) % 360

    def choose_action(self, scan_points):
        blocked = self.build_occupancy_by_angle(scan_points)
        fov_bins = self.extract_fov_bins()
        gap = self.find_largest_gap(blocked, fov_bins)

        if gap is None:
            action = STOP
            target_angle = None
        else:
            s_idx, e_idx, width = gap
            # require min_gap
            if width < self.min_gap:
                action = STOP
                target_angle = None
            else:
                # choose center bin
                # handle wrap: compute average in circular sense
                # simple: convert s,e to angles and take middle along FOV sequence
                mid_bin = (s_idx + e_idx) // 2
                target_angle = self.bin_to_angle(mid_bin)
                # normalize to -180..180 relative to forward 0
                a = ((target_angle + 180) % 360) - 180

                # decide
                if abs(a) <= self.forward_thresh:
                    action = FORWARD
                elif a > 0:
                    # positive means clockwise to right side
                    # choose TURN_RIGHT
                    action = TURN_RIGHT
                else:
                    action = TURN_LEFT

        # hysteresis: prefer last action unless new action persists
        self.history.append(action)
        if len(self.history) == self.history.maxlen:
            # stable if majority in history equals action
            from collections import Counter
            cnt = Counter(self.history)
            most, num = cnt.most_common(1)[0]
            stable_action = most
        else:
            stable_action = self.last_action if self.last_action else action

        self.last_action = stable_action

        # produce motion command (v, omega)
        v = 0.0; omega = 0.0
        if stable_action == FORWARD:
            v = 0.15  # m/s
            omega = 0.0
        elif stable_action == TURN_LEFT:
            v = 0.0
            omega = 0.8  # rad/s (positive CCW)
        elif stable_action == TURN_RIGHT:
            v = 0.0
            omega = -0.8
        else:
            v = 0.0; omega = 0.0

        return {
            "action": stable_action,
            "target_angle": None if gap is None else self.bin_to_angle((gap[0]+gap[1])//2),
            "gap_width_bins": (None if gap is None else gap[2]),
            "v": v, "omega": omega,
            "blocked_map": blocked
        }
