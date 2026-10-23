"""Dựng ảnh phối cảnh từ ảnh cầu equirect (dùng chung cho grid_views.py và detail_views.py)."""
import cv2
import numpy as np


def view(eq, yaw_deg, pitch_deg, fov_deg=100, size=900):
    h, w = eq.shape[:2]
    f = size / 2 / np.tan(np.radians(fov_deg) / 2)
    xs, ys = np.meshgrid(np.arange(size) - size / 2, np.arange(size) - size / 2)
    d = np.stack([xs / f, ys / f, np.ones_like(xs)], -1)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    p, y = np.radians(pitch_deg), np.radians(yaw_deg)
    # Hệ toàn cảnh: x phải, y xuống, z hướng nhìn ban đầu.
    rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    d = d @ (ry @ rx).T
    u = (np.arctan2(d[..., 0], d[..., 2]) / (2 * np.pi) + 0.5) * w
    v = (1 - np.arccos(np.clip(d[..., 1], -1, 1)) / np.pi) * (h - 1)
    u = np.mod(u, w)
    return cv2.remap(eq, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE)

