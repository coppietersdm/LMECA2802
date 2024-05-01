from MBS_design import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


MBS.read_q()

positions = []
left_vert_force = []
right_vert_force = []
left_hor_force = []
right_hor_force = []
t = []

for i in range(0,len(MBS.q),100):
    MBS.set_q(MBS.q[i][1:])
    MBS.forward_kinematics()
    MBS.compute_q_dependent_variables()
    MBS.compute_external_forces(MBS.q[i][0])
    position_sensors = np.array(list(map(lambda pos_sens: pos_sens[0].anchor_points_inertial_frame[pos_sens[1]],MBS.position_sensors)))[5]
    positions.append(position_sensors)
    t.append(MBS.q[i][0])
    
    left_vert_force.append((MBS.position_sensors[4][0].R0i@MBS.position_sensors[4][0].Fext)[2])
    right_vert_force.append((MBS.position_sensors[7][0].R0i@MBS.position_sensors[7][0].Fext)[2])
    
    left_hor_force.append((MBS.position_sensors[4][0].R0i@MBS.position_sensors[4][0].Fext)[0])
    right_hor_force.append((MBS.position_sensors[7][0].R0i@MBS.position_sensors[7][0].Fext)[0])
    
    
positions = np.array(positions)

plt.plot(t, positions.T[0])
print(positions.T[0][-1])
plt.title("horizontal position of the top")
plt.show()

plt.plot(t, left_vert_force) 
plt.plot(t, right_vert_force)
plt.show()

plt.plot(t, left_hor_force)
plt.plot(t, right_hor_force)
plt.show()