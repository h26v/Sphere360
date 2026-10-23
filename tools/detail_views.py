"""Cắt vài góc chi tiết: detail_views.py pano.jpg out.jpg yaw,pitch,fov ..."""
import sys
import importlib.util

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location("pv", __file__.replace("detail_views.py", "pole_view_lib.py"))
pv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pv)

eq = cv2.imread(sys.argv[1])
tiles = []
for arg in sys.argv[3:]:
    y, p, f = map(float, arg.split(","))
    tiles += [pv.view(eq, y, p, f, 900), np.full((900, 8, 3), 255, np.uint8)]
cv2.imwrite(sys.argv[2], cv2.resize(np.hstack(tiles[:-1]), None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA),
            [cv2.IMWRITE_JPEG_QUALITY, 88])
