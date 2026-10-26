"""Ground-truth warehouse map (5 cm grid). 12 m x 8 m, outer walls, shelf rows,
one narrow 0.70 m aisle. Origin of the grid = (0, 0) at the bottom-left corner."""
import numpy as np

RES = 0.05
W, H = 12.0, 8.0


def build():
    g = np.zeros((int(H / RES), int(W / RES)), dtype=bool)  # [row=y, col=x]

    def box(x0, y0, x1, y1):
        g[int(round(y0 / RES)):int(round(y1 / RES)), int(round(x0 / RES)):int(round(x1 / RES))] = True

    t = 0.1
    box(0, 0, W, t); box(0, H - t, W, H); box(0, 0, t, H); box(W - t, 0, W, H)
    # three long shelf rows (0.6 m deep) with gaps at the ends -> normal aisles 1.4 m
    for y in (2.0, 4.0):
        box(2.0, y, 9.0, y + 0.6)
    # a short shelf block next to the lower row leaving a 0.70 m slot (x 9.0..9.7) between the lower corridor and the middle aisle
    box(9.7, 0.1, 10.6, 3.2)
    # a few pallets / boxes
    box(3.0, 6.2, 3.8, 6.8); box(6.0, 5.6, 6.6, 6.2); box(10.4, 5.8, 11.2, 6.6)
    box(0.8, 3.0, 1.2, 3.6)
    return g


if __name__ == "__main__":
    g = build()
    print(g.shape, g.mean())
