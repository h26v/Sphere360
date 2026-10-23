"""Ảnh tổng quan equirect và lưới góc nhìn phối cảnh: grid_views.py pano.jpg out_prefix"""
import sys
import importlib.util

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location("pv", __file__.replace("grid_views.py", "pole_view_lib.py"))
pv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pv)

eq = cv2.imread(sys.argv[1])
pre = sys.argv[2]
cv2.imwrite(pre + "_eq.jpg", cv2.resize(eq, (2048, 1024), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 85])
rows = []
for pitch in (45, -10, -60):
    row = []
    for yaw in (0, 90, 180, 270):
        row += [pv.view(eq, yaw, pitch, 90, 640), np.full((640, 6, 3), 255, np.uint8)]
    rows.append(np.hstack(row[:-1]))
cv2.imwrite(pre + "_grid.jpg", np.vstack([np.vstack([r, np.full((6, r.shape[1], 3), 255, np.uint8)]) for r in rows]),
            [cv2.IMWRITE_JPEG_QUALITY, 85])
poles = np.hstack([pv.view(eq, 0, 90, 110, 800), np.full((800, 6, 3), 255, np.uint8), pv.view(eq, 0, -90, 110, 800)])
cv2.imwrite(pre + "_poles.jpg", poles, [cv2.IMWRITE_JPEG_QUALITY, 85])
