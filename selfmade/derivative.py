import numpy as np
from math import sin, cos, pi


def sweep(t, t0, f0, t1, f1, Fmax):
    """ Compute the value of a force sweep function at the given time.
    The sweep function has asinusoidal shape with constant amplitude 
    and a varying frequency. This function enables to consider linear
    variation of the frequency between f0 and f1 defined at time t0 and t1.

	:param t: the time instant when to compute the function.
	:param t0: the time instant when to specify f0.
	:param f0: the frequency at time t0.
	:param t1: the time instant when to specify f1.
	:param f1: the frequency at time t1.
	:param Fmax: the semi-amplitude of the function.
		
	:return Fext: the value of the sweep function.
    """
    Fext = Fmax*sin(2*pi*(f0+(f1-f0)/(t1-t0)*t/2)*t)   
    
    return Fext

def compute_derivatives(t, y, data):
    #### Initialization of matrices and vectors ####
    M = np.zeros((2,2))
    c = np.zeros((2,1))
    Q = np.zeros((2,1))  
    yd = np.zeros(4)
    F = np.zeros((2,1)) 
    
    ### Values of matrices et vecteurs ###
    #M
    M[0][0] = data.m1 + data.m2
    M[0][1] = data.m2
    M[1][0] = data.m2
    M[1][1] = data.m2
    
    #Minv
    invM = np.zeros((2,2))
    detM = M[0][0]*M[1][1] - M[0][1]*M[1][0]
    invM[0][0] = 1/detM*M[1][1]
    invM[0][1] = -1/detM*M[0][1]
    invM[1][0] = -1/detM*M[1][0] 
    invM[1][1] = 1/detM*M[0][0]   
    
    #Fext
    Fext = sweep(t, data.t0, data.f0, data.t1, data.f1, data.Fmax)
    
    #c
    c[0][0] = (data.m1+data.m2)*data.g + Fext
    c[1][0] = data.m2*data.g + Fext
    
    ### Calculate yd ###
    yd[0] = y[2]
    yd[1] = y[3]
    
    Q[0][0] = -data.k01*((y[0])-data.z01)-data.d1*(y[2])
    Q[1][0] = -data.k02*((y[1])-data.z02)-data.d2*(y[3])
    
    F[0][0] = Q[0][0] - c[0][0]
    F[1][0] = Q[1][0] - c[1][0]
    
    yd[2] = invM[0][0]*F[0][0] + invM[0][1]*F[1][0]
    yd[3] = invM[1][0]*F[0][0] + invM[1][1]*F[1][0]
    
    return yd