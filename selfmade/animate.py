from MBS_design import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#MBS.integrate(5.0, 0.001)
MBS.read_q()

def animate(i):
    print(i)
    MBS.set_q(MBS.q[i*100][1:])
    plt.cla()
    MBS.plot()
    plt.xlim(-0.2,0.2)
    plt.ylim(8.5,8.7)

ani = FuncAnimation(plt.gcf(), animate, frames=len(MBS.q)//100, interval=1)

plt.axis('equal')

# Enregistrement de l'animation au format MP4
ani.save('animation.mp4', writer='ffmpeg', fps=10)

plt.show()

