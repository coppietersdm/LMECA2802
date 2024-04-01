from math import sin, cos, pi
import numpy as np
from scipy.integrate import solve_ivp
from derivative import compute_derivatives
from matplotlib import pyplot as plt

class MBSData:
    """ Class to define the parameters of the the double mass-spring model
    and of the problem to solve.
     
    It contains the following fields: 
    
    general parameters:
    -------------------
    g:     Gravity
    
    masses:
    -------
    m1:    Unsprung mass
    m2:    Sprung mass
    
    parameters of the tyre:
    -----------------------
    k01:   Stiffness
    d1:    Damping
    z01:   Neutral length
    
    parameters of the suspension:
    -----------------------------
    k02:   Stiffness
    d2:    Damping
    z02:   Neutral length   
    
    parameter of the external force:
    --------------------------------
    Fmax:  Semi-amplitude of the displacement
    f0:    Frequency at start
    t0:    Time for specifying frequency at start
    f1:    Frequency at end
    t1:    Time for specifying frequency at end
    
    equilibrium positions:
    ----------------------
    q1:    Equilibrium position coordinate of m1
    q2:    Equilibrium position coordinate of m2
    """
    
    def __init__(self):
        self.g = 9.81 
    
        self.m1 = 25
        self.m2 = 315
        
        self.d1 = 107
        self.d2 = 4000
        self.z01 = 0.375
        self.z02 = 0.8
        self.k01 = 190e3
        self.k02 = 37e3

        self.Fmax = 10000
        self.f0 = 1
        self.f1 = 10
        self.t0 = 0
        self.t1 = 10
        
        self.q1 = 0.357445263
        self.q2 = 0.716482432
        

def compute_dynamic_response(data):
    ### Data ###
    tspan=(data.t0, data.t1)
    n_state = 2 #number of states  of q and qd
    
    #### Matrices initialization ####
    q = np.zeros(n_state)
    qd = np.zeros(n_state)

    ### Initial values ###
    #q
    q0 = np.array([data.q1, data.q2])
    q[:] = q0[:]
    
    #y
    y0 = np.zeros(4)
    y0[:n_state] = q[:]
    y0[n_state:2*n_state] = qd[:]

    ### Runge Kutta ###   
    sol = solve_ivp(fun=lambda t,y: compute_derivatives(t, y, data), t_span=tspan, y0=y0)
    
    ### Display balance values ###
    print('q1 = ',sol.y[0][-1],'q2 = ',sol.y[1][-1])

    ### Calculate qdd ###
    ydmatrix = np.zeros([2*n_state, sol.t.__len__()])
    for i in range(sol.t.__len__()):
        np.array([sol.y[0][i],sol.y[1][i],sol.y[2][i],sol.y[3][i]])
        a = compute_derivatives(sol.t[i], np.array([sol.y[0][i],sol.y[1][i],sol.y[2][i],sol.y[3][i]]),data)
        ydmatrix[0][i] = a[0]
        ydmatrix[1][i] = a[1] 
        ydmatrix[2][i] = a[2] 
        ydmatrix[3][i] = a[3]

    ### Files creation and results record ###
    np.savetxt('dirdyn_q.res', np.transpose(np.array([sol.t,sol.y[0],sol.y[1]])))
    np.savetxt('dirdyn_qd.res', np.transpose(np.array([sol.t,sol.y[2],sol.y[3]])))
    np.savetxt('dirdyn_qdd.res', np.transpose(np.array([sol.t,ydmatrix[2],ydmatrix[3]])))
    plt.plot(sol.t, sol.y[0], label='q1')
    plt.plot(sol.t, sol.y[1], label='q2')
    plt.show()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# Main function

if __name__ == '__main__':
    mbs_data = MBSData()
    
    compute_dynamic_response(mbs_data)  