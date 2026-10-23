"""Đo tiêu cự thật của camera từ các cặp ảnh kề nhau trong một lượt chụp Sphere360.

Với mỗi hệ số tiêu cự, đếm số điểm khớp giải thích được bằng một phép xoay thuần túy.
Hệ số đúng cho nhiều điểm đạt nhất. So sánh thêm với homography (mô hình tự do hơn).
"""
import json
import sys

import cv2
import numpy as np

sess = sys.argv[1]
pairs = [(0, 1), (1, 2), (2, 3), (0, 9), (1, 10)]
meta = json.load(open(f"{sess}/session.json"))
f0 = meta["focalPx"]
scale = 0.28

sift = cv2.SIFT_create(3000)
cache = {}


def feats(i):
    if i not in cache:
        img = cv2.imread(f"{sess}/{meta['frames'][i]['file']}", cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        kp, des = sift.detectAndCompute(img, None)
        cache[i] = (img.shape, np.float64([k.pt for k in kp]), des)
    return cache[i]


def matches(i, j):
    si, pi, di = feats(i)
    sj, pj, dj = feats(j)
    bf = cv2.BFMatcher()
    good = [m for m, n in bf.knnMatch(di, dj, k=2) if m.distance < 0.7 * n.distance]
    return si, pi[[m.queryIdx for m in good]], pj[[m.trainIdx for m in good]]


def rays(pts, f, shape):
    h, w = shape
    v = np.c_[(pts[:, 0] - w / 2) / f, (pts[:, 1] - h / 2) / f, np.ones(len(pts))]
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def rot_inliers(a, b, thr, iters=600, rng=np.random.default_rng(0)):
    best = 0
    n = len(a)
    for _ in range(iters):
        p, q = rng.choice(n, 2, replace=False)
        a1, a2, b1, b2 = a[p], a[q], b[p], b[q]
        ac, bc = np.cross(a1, a2), np.cross(b1, b2)
        if np.linalg.norm(ac) < 1e-3:
            continue
        ac /= np.linalg.norm(ac)
        bc /= np.linalg.norm(bc)
        Ma = np.c_[a1, ac, np.cross(a1, ac)]
        Mb = np.c_[b1, bc, np.cross(b1, bc)]
        A = Ma @ Mb.T
        err = np.arccos(np.clip(np.sum(a * (b @ A.T), axis=1), -1, 1))
        best = max(best, int((err < thr).sum()))
    return best


for i, j in pairs:
    shape, pa, pb = matches(i, j)
    if len(pa) < 20:
        print(i, j, "ít khớp", len(pa))
        continue
    H, mask = cv2.findHomography(pa, pb, cv2.RANSAC, 1.5)
    hom = int(mask.sum()) if mask is not None else 0
    line = []
    for k in np.arange(0.70, 1.31, 0.05):
        f = f0 * scale * k
        a, b = rays(pa, f, shape), rays(pb, f, shape)
        line.append((round(k, 2), rot_inliers(a, b, 1.5 / f)))
    best = max(line, key=lambda t: t[1])
    print(f"cặp {i}-{j}: {len(pa)} khớp, homography {hom}, xoay tốt nhất k={best[0]} -> {best[1]}  {line}")
