import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from IPython.display import display, clear_output

def draw_cart_pendulum(y, m, M, L):
    x = y[0]
    th = y[2]

    # dimensions
    W = 1 * np.sqrt(M / 5)  # cart width
    H = 0.5 * np.sqrt(M / 5)  # cart height
    wr = 0.2  # wheel radius
    mr = 0.3 * np.sqrt(m)  # mass radius

    # positions
    cart_y = wr / 2 + H / 2  # cart vertical position
    w1x = x - 0.9 * W / 2
    w1y = 0
    w2x = x + 0.9 * W / 2 - wr
    w2y = 0
    px = x + L * np.sin(th)
    py = cart_y - L * np.cos(th)

    fig, ax = plt.subplots()
    ax.plot([-10, 10], [0, 0], 'w', linewidth=2)
    ax.add_patch(patches.FancyBboxPatch((x - W / 2, cart_y - H / 2), W, H,
                                        boxstyle="round,pad=0.1", facecolor=[1, 0.1, 0.1], edgecolor=[1, 1, 1]))
    ax.add_patch(patches.Circle((w1x + wr / 2, w1y + wr / 2), wr / 2,
                                facecolor=[1, 1, 1], edgecolor=[1, 1, 1]))
    ax.add_patch(patches.Circle((w2x + wr / 2, w2y + wr / 2), wr / 2,
                                facecolor=[1, 1, 1], edgecolor=[1, 1, 1]))

    ax.plot([x, px], [cart_y, py], 'w', linewidth=2)
    ax.add_patch(patches.Circle((px, py), mr / 2,
                                facecolor=[0.3, 0.3, 1], edgecolor=[1, 1, 1]))

    ax.set_xlim([-5, 5])
    ax.set_ylim([-2, 2.5])
    ax.set_facecolor('k')
    ax.tick_params(axis='x', colors='w')
    ax.tick_params(axis='y', colors='w')
    fig.set_size_inches(8, 4)
    fig.patch.set_facecolor('k')
    fig.patch.set_alpha(1.0)

    display(fig)
    
    clear_output(wait = True)
    plt.pause(0.005)

