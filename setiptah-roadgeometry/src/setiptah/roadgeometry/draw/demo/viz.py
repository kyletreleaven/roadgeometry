import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches

from setiptah.roadgeometry.draw.visualization import fitArc2Segment


def main() :
    pt1 = np.random.rand(2)
    pt2 = np.random.rand(2)
    dEuc = np.linalg.norm(pt2 - pt1)
    d = ( 1. + np.random.rand() ) * dEuc
    center, R, theta1, theta2 = fitArc2Segment( pt1, pt2, d )

    plt.figure()
    xdata = [ pt1[0], pt2[0] ]
    ydata = [ pt1[1], pt2[1] ]
    plt.scatter(xdata ,ydata)
    ax = plt.gca()
    circ = mpl.patches.Circle(center, R, linestyle='--', fill=False)
    ax.add_patch(circ)
    arc = mpl.patches.Arc(center, R, R, angle=0., theta1=theta2, theta2=theta1)
    ax.add_patch(arc)
    ax.set_aspect('equal')


if __name__ == "__main__":
    main()
    plt.show()
