"""
Live plotting from file.
"""


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


if __name__ == "__main__":
    fig = plt.figure()
    fig.tight_layout()
    ax1 = fig.add_subplot(3,1,1)
    ax2 = fig.add_subplot(3,1,2)
    ax3 = fig.add_subplot(3,1,3)

    def animate(i):
        graph_data = open('pi_control/logs/data.txt', 'r').read()
        lines = graph_data.split('\n')

        xs = list()
        in_lefts, in_rights = list(), list()
        w_lefts, w_rights = list(), list()
        o_lefts, o_rights = list(), list()
        
        for l in lines:
            if len(l) > 1:
                x, in_left, in_right, w_left, w_right, o_left, o_right = l.split(',')
                xs.append(float(x))

                in_lefts.append(float(in_left))
                in_rights.append(float(in_right))
                w_lefts.append(float(w_left))
                w_rights.append(float(w_right))
                o_lefts.append(float(o_left))
                o_rights.append(float(o_right))
        
        # Input
        ax1.clear()
        ax1.plot(xs, in_lefts, label='left')
        ax1.plot(xs, in_rights, label='left')
        ax1.set_title('Input')
        ax1.set_xticks([])
        ax1.legend()

        # Weight
        ax2.clear()
        ax2.plot(xs, w_lefts, label='left')
        ax2.plot(xs, w_rights, label='right')
        ax2.set_title('Weight')
        ax2.set_xticks([])
        ax2.legend()

        # Output
        ax3.clear()
        ax3.plot(xs, o_lefts, label='left')
        ax3.plot(xs, o_rights, label='right')
        ax3.set_title('Output')
        ax3.set_xticks([])
        ax3.legend()

    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
