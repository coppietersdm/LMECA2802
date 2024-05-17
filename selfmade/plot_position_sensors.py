from MBS_design import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


MBS.read_q()

positions4 = []
speeds4 = []
positions7 = []
left_vert_force = []
right_vert_force = []
left_hor_force = []
right_hor_force = []
t = []

for i in range(0,len(MBS.q),100):
    MBS.set_q(MBS.q[i][1:])
    MBS.set_qd(MBS.qd[i])
    MBS.compute_q_dependent_variables()
    MBS.forward_kinematics()
    MBS.compute_q_and_qd_dependent_variables()
    MBS.backward_dynamics()
    
    position_sensors4 = np.array(list(map(lambda pos_sens: pos_sens[0].anchor_points_inertial_frame[pos_sens[1]],MBS.position_sensors)))[4]
    positions4.append(position_sensors4)
    speed_sensors4 = np.array(list(map(lambda pos_sens: pos_sens[0].anchor_points_speed_inertial_frame[pos_sens[1]],MBS.position_sensors)))[4]
    speeds4.append(speed_sensors4)
    
    position_sensors7 = np.array(list(map(lambda pos_sens: pos_sens[0].anchor_points_inertial_frame[pos_sens[1]],MBS.position_sensors)))[7]
    positions7.append(position_sensors7)
    t.append(MBS.q[i][0])
    
    left_vert_force.append(MBS.spring(0,6)[2])
    right_vert_force.append(MBS.spring(1,5)[2])
    
    left_hor_force.append(MBS.spring(0,6)[1])
    right_hor_force.append(MBS.spring(1,5)[1])
    
positions4 = np.array(positions4)
speeds4 = np.array(speeds4)
positions7 = np.array(positions7)

plt.plot(t, positions4.T[1], label = 'left')

#plt.plot(t, speeds4.T[1])
plt.plot(t, positions7.T[1], label = 'right')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('position [m]')
plt.title("horizontal position of the top")
plt.savefig('posTop.pdf')
plt.show()
plt.clf()

plt.plot(t, np.array(left_vert_force)/1e3, label = 'left') 
plt.plot(t, np.array(right_vert_force)/1e3, label = 'right')
plt.title('Vertical forces of the walls')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('force [kN]')
plt.savefig('vertForce.pdf')
plt.show()

plt.clf()


plt.plot(t, np.array(left_hor_force)/1e3, label = 'left')
plt.plot(t, np.array(right_hor_force)/1e3, label = 'right')
plt.title('Horizontal forces of the walls')
plt.legend()
plt.xlabel('time [s]')
plt.ylabel('force [kN]')
plt.savefig("horForce.pdf")
plt.show()

forces = [MBS.spring(0,6),MBS.spring(1,5),MBS.spring(2,6),MBS.spring(3,5),MBS.spring(4,7)]

for force in forces:
    print(force)

print(forces[2] + forces[3])

#plt.show()