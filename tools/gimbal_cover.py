"""Độ phủ mặt cầu từ một lần dò tầm gimbal (map_*.csv): vùng tâm ảnh với tới được + khung hình -> mặt cầu."""
import sys, csv, numpy as np

def load(path):
    rows = list(csv.DictReader(open(path)))
    ph = np.array([r['phase'] for r in rows])
    R = np.array([[float(r['r%d' % i]) for i in range(9)] for r in rows]).reshape(-1, 3, 3)
    cam = -R[:, :, 2]
    yaw = np.degrees(np.arctan2(cam[:, 0], cam[:, 1]))
    pit = np.degrees(np.arcsin(np.clip(cam[:, 2], -1, 1)))
    return ph, yaw, pit

def reach(ph, yaw, pit):
    """Tâm ảnh với tới được: mọi hướng máy đã đi qua khi quét tilt ở các trạm, nội suy giữa các trạm."""
    st = []
    for k in range(50):
        s = np.isin(ph, [f'st{k}_tilt+', f'st{k}_tilt-'])
        if s.any():
            st.append((np.degrees(np.angle(np.mean(np.exp(1j * np.radians(yaw[s]))))), pit[s].min(), pit[s].max()))
    m = ph == 'pan+'
    yu = np.degrees(np.unwrap(np.radians(yaw[m])))
    # Vị trí trạm dọc cung pan (độ tính từ đầu pan+), sắp theo cung.
    def arcpos(y):
        d = (y - yu[0] + 540) % 360 - 180
        sgn = np.sign(yu[-1] - yu[0])
        return (d * sgn) % 360
    arcLen = abs(yu[-1] - yu[0])
    pts = sorted((min(arcpos(y), arcLen), lo, hi) for y, lo, hi in st)
    centers = []
    for s in np.arange(0, arcLen + 0.01, 1.0):
        xs = [p[0] for p in pts]
        lo = np.interp(s, xs, [p[1] for p in pts]); hi = np.interp(s, xs, [p[2] for p in pts])
        y = yu[0] + np.sign(yu[-1] - yu[0]) * s
        for p in np.arange(lo, hi + 0.01, 2.0):
            centers.append((y, p))
    return np.array(centers), arcLen, st

def cover(centers, hfov, vfov, res=1.0):
    lat = np.radians(np.arange(-90 + res / 2, 90, res)); lon = np.radians(np.arange(-180 + res / 2, 180, res))
    LA, LO = np.meshgrid(lat, lon, indexing='ij')
    D = np.stack([np.cos(LA) * np.sin(LO), np.cos(LA) * np.cos(LO), np.sin(LA)], -1)
    cov = np.zeros(LA.shape, bool)
    th, tv = np.tan(np.radians(hfov / 2)), np.tan(np.radians(vfov / 2))
    for y, p in centers[::2]:
        y, p = np.radians(y), np.radians(p)
        f = np.array([np.cos(p) * np.sin(y), np.cos(p) * np.cos(y), np.sin(p)])
        r = np.array([np.cos(y), -np.sin(y), 0.0])
        u = np.cross(r, f)
        z = D @ f
        ok = (z > 0.05) & (np.abs(D @ r) <= th * z) & (np.abs(D @ u) <= tv * z)
        cov |= ok
    w = np.cos(LA)
    frac = (cov * w).sum() / w.sum()
    rows = cov.any(1)
    top = 90 - np.degrees(lat[rows].max()) - res / 2
    bot = 90 + np.degrees(lat[rows].min()) - res / 2
    # Chỗ hở theo vĩ độ: mỗi hàng vĩ độ còn hở bao nhiêu độ kinh.
    return frac, cov, lat, lon

if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    ph, yaw, pit = load(sys.argv[1])
    centers, arcLen, st = reach(ph, yaw, pit)
    print("cung pan %.1f°, %d tâm ảnh" % (arcLen, len(centers)))
    for hf, vf, name in [(59.6, 74.8, 'dọc'), (74.8, 59.6, 'ngang')]:
        frac, cov, lat, lon = cover(centers, hf, vf)
        print(f"máy {name} ({hf}x{vf}): phủ {frac * 100:.1f}% mặt cầu")
        for band in [(75, 90), (60, 75), (45, 60), (-45, 45), (-60, -45), (-75, -60), (-90, -75)]:
            m = (np.degrees(lat) >= band[0]) & (np.degrees(lat) < band[1])
            w = np.cos(lat[m])[:, None]
            print(f"   vĩ độ {band[0]:4d}..{band[1]:3d}: {(cov[m] * w).sum() / (w.sum() * cov.shape[1]) * 100:5.1f}%")
