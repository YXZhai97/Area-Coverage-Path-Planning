"""
steps to create an animation
1. have a clear picture of the animation
2. import standard Modules and FuncAnimation
3. set up Figure, Axes, and Line object
4. initialize data
5. define the animation function
6. pass everything to FuncAnimation
7. display or save animation

"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# set up empty figure, axes and line object
fig, ax = plt.subplots()
ax.set(xlim=(-0.1, 2 * np.pi + 0.1), ylim=(-1.1, 1.1))
line, = ax.plot([], [],'ro')

x = np.linspace(0, 2 * np.pi, num=50)
y = np.sin(x)


# i is the frame
def animate(i):
    line.set_data(x[:i], y[:i])
    return line,


def init():
    line.set_data([],[])
    return line,


anim = FuncAnimation(fig, animate, frames=1000, init_func=init, interval=40, blit=False)

plt.show()
anim.save('../image/animation.gif', writer='imagemagick', fps=60)