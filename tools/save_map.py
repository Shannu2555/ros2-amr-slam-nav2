"""Writes the last slam_toolbox /map from the recorded npz as a Nav2 map (pgm + yaml),
same trinary format as nav2_map_server's map_saver (free 254, occupied 0, unknown 205)."""
import sys
import numpy as np
z = np.load(sys.argv[1])
n = len(z["mt"]); m = z[f"map{n-1}"]; res, ox, oy = z["meta"][n - 1]
img = np.full(m.shape, 205, np.uint8); img[m == 0] = 254; img[m >= 65] = 0
img = np.flipud(img)
out = sys.argv[2]
with open(out + ".pgm", "wb") as f:
    f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode()); f.write(img.tobytes())
open(out + ".yaml", "w").write(f"image: {out.split('/')[-1]}.pgm\nmode: trinary\nresolution: {res}\n"
    f"origin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
print(n, m.shape, res, ox, oy)
