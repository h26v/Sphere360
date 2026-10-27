"""Mô hình tầm với OM7 (đo từ map_*.csv): ở hướng yaw psi, máy ngẩng/cúi được trong
[L0 + b*cos(psi - phi), U0 + b*cos(psi - phi)] (b = độ nghiêng tay cầm, phi = hướng ngả), pan đi được ARC độ,
chỗ hở pan đặt lệch delta độ so với hướng ngả. Quét b và delta, tính độ phủ mặt cầu (một lượt, không chỉnh tay)."""
import sys
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__file__))
from gimbal_cover import cover

L0, U0, ARC = -37.0, 23.0, 295.0

def centers(b, delta, step=2.0):
    out = []
    gap_mid = delta  # hướng ngả phi = 0
    start = gap_mid + (360 - ARC) / 2
    for s in np.arange(0, ARC + 0.01, step):
        y = start + s
        c = b * np.cos(np.radians(y))
        for p in np.arange(L0 + c, U0 + c + 0.01, 3.0):
            if -89 < p < 89:
                out.append(((y + 180) % 360 - 180, p))
    return np.array(out)

if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    hf, vf = 59.6, 74.8
    print("nghiêng|lệch " + " ".join(f"{d:6d}" for d in range(0, 181, 45)))
    for b in [0, 10, 17, 25, 30, 35, 40, 45, 55]:
        row = []
        for d in range(0, 181, 45):
            frac, cov, lat, lon = cover(centers(b, d), hf, vf, res=2.0)
            row.append(frac * 100)
        print(f"{b:6d}° " + " ".join(f"{v:6.1f}" for v in row))
