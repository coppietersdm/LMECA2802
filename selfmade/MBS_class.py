from math import sin, cos, pi
import numpy as np
from scipy.integrate import solve_ivp
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.animation import FuncAnimation

e1 = np.array([1,0,0])
e2 = np.array([0,1,0])
e3 = np.array([0,0,1])
zero = np.zeros(3)
I2 = np.array([[0,0,0],[0,1,0],[0,0,0]])

g = np.array([0,0,-9.81])


def Rmat(phi, theta):
    return Rotation.from_rotvec(phi * theta).as_matrix()

def tilde(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

class mbs():
    def __init__(self):
        self.bodies = []
        self.base = body(None, "T1")
        self.q = None
        self.t = None
    
    def read_q(self):
        self.q = np.loadtxt('dirdyn_q.res')
    
    def add_body(self, inbody, joint, m=0, I=0, dii=np.zeros(3), dhi = np.zeros(3), q0=np.zeros(3), Q=lambda t,q,qd: 0):
        self.bodies.append(body(inbody, joint, m=m, I=I, dii=dii, dhi=dhi, q0=q0, Q=Q))
    
    def compute_q_dependent_variables(self):
        for body in self.bodies:
            body.compute_q_dependent_variables()
            
    def plot(self):
        self.compute_q_dependent_variables()
        for body in self.bodies:
            body.plot()
    
    def set_q(self, q):
        for i in range(len(self.bodies)):
            self.bodies[i].q[0] = q[i]
    
    def set_qd(self,qd):
        for i in range(len(self.bodies)):
            self.bodies[i].q[1] = qd[i]
    
    def get_q(self):
        return np.array([body.q[0] for body in self.bodies])
    
    def get_qd(self):
        return np.array([body.q[1] for body in self.bodies])

    def get_qdd(self):
        return np.array([body.q[2] for body in self.bodies])

    def compute_barycentric_quantities(self):
        for body in reversed(self.bodies):
            body.m_bar = body.m + sum(map(lambda x:x.m_bar, body.children))
            body.b = (body.m*body.dii + sum(map(lambda x:x.m_bar*x.dhi, body.children)))/body.m_bar
            body.K = body.I - sum(map(lambda x:x.m_bar*tilde(x.dhi)@tilde(x.dhi),body.children))
    
    def forward_kinematics(self):
        # initialisation
        for i in ([self.base] + self.bodies):
            i.Om = np.zeros((len(self.bodies),3))
            i.Am = np.zeros((len(self.bodies),3))
        
        for i in self.bodies:
            R = i.Rhi.T 
            h = i.inbody
            i.omega    = R @ h.omega    + i.phi*i.q[1]
            i.omegad_c = R @ h.omegad_c + tilde(i.omega)@i.phi*i.q[1]
            i.beta_c   = tilde(i.omegad_c)  + tilde(i.omega)@tilde(i.omega)
            i.alpha_c  = R @ (h.alpha_c + h.beta_c@i.dzhi) + 2*tilde(i.omega)@i.psi*i.q[1]
            for k in range(i.id):
                i.Om[k] = R @  h.Om[k]  + ((k+1) == i.id)*i.phi
                i.Am[k] = R @ (h.Am[k]  + tilde(h.Om[k])@i.dzhi) + ((k+1) == i.id)*i.psi
          
    def backward_dynamics(self):
        # initialisation
        for i in (self.bodies):
            i.Wm = np.zeros((len(self.bodies),3))
            i.Fm = np.zeros((len(self.bodies),3))
            i.Lm = np.zeros((len(self.bodies),3))
            
        for i in reversed(self.bodies):
            i.Wc = i.m*(i.alpha_c + i.beta_c@i.dzii) - i.Fext
            i.Fc = sum(map(lambda j:j.Rhi @ j.Fc, i.children)) + i.Wc
            i.Lc = sum(map(lambda j:j.Rhi @ j.Lc + tilde(j.dzhi)@j.Rhi@j.Fc, i.children)) + tilde(i.dzii)@i.Wc - i.Lext + i.I@i.omegad_c + tilde(i.omega)@i.I@i.omega
            for k in range(i.id):
                i.Wm[k] = i.m*(i.Am[k]+tilde(i.Om[k])@i.dzii)
                i.Fm[k] = sum(map(lambda j:j.Rhi @ j.Fm[k], i.children)) + i.Wm[k]
                i.Lm[k] = sum(map(lambda j:j.Rhi @ j.Lm[k] + tilde(j.dzhi)@j.Rhi @ j.Fm[k], i.children)) + tilde(i.dzii)@i.Wm[k] + i.I@i.Om[k]
    
    def second_derivative(self,t):
        self.compute_q_dependent_variables()
        self.forward_kinematics()
        self.backward_dynamics()

        M = np.zeros((len(self.bodies),len(self.bodies)))
        c = np.zeros(len(self.bodies))
        for i in self.bodies:
            c[i.id-1] = i.psi @ i.Fc + i.phi @ i.Lc
            for j in self.bodies:
                M[i.id-1][j.id-1] = i.psi @ i.Fm[j.id-1] + i.phi @ i.Lm[j.id-1]
                M[j.id-1][i.id-1] = M[i.id-1][j.id-1]
        Q = np.array([body.Q(t,body.q[0],body.q[1]) for body in self.bodies])
        qdd = np.linalg.solve(M,-c+Q)
        for body in self.bodies:
            body.q[2] = qdd[body.id-1]
            
    def integrate(self, t, dt):
        y0 = np.array(list(self.get_q()) + list(self.get_qd()))
        def derivative(t,y):
            self.set_q(y[:len(self.bodies)])
            self.set_qd(y[len(self.bodies):2*len(self.bodies)])
            self.second_derivative(t)
            return np.array(list(self.get_qd()) + list(self.get_qdd()))
        self.t = np.linspace(0.0,t,int(t*1000+1))
        sol = solve_ivp(fun=lambda t,y: derivative(t,y), t_span=(0,t), y0=y0, t_eval=self.t, method = 'RK45')
        self.q = np.zeros((len(self.t), len(self.bodies)+1))
        self.q.T[1:] = sol.y[:len(self.bodies)]
        self.q.T[0] = self.t
        np.savetxt('dirdyn_q.res', self.q)
        
                
class body():
    N = 0
    def __init__(self, inbody, joint, m=0.0, I=np.zeros((3,3), dtype=float), dii=np.zeros(3,dtype=float), dhi = np.zeros(3,dtype=float), q0=np.zeros(3,dtype=float), Q=lambda t,q,qd:0.0):
        # ---------------------------------------------------------------------------------------------
        # body constants
        # ---------------------------------------------------------------------------------------------
        self.m = m; self.I = I
        self.dii = dii; self.dhi = dhi

        self.Q = Q
        
        self.inbody = inbody
        self.id = body.N; body.N += 1
        
        dico = {"T1":(e1,zero), "T2":(e2,zero), "T3":(e3,zero), "R1":(zero,e1), "R2":(zero,e2), "R3":(zero,e3)}
        self.psi, self.phi = dico[joint]

        self.children = []
        if(self.inbody != None):
            self.inbody.children.append(self)
        
        # ---------------------------------------------------------------------------------------------
        # q dependent variables
        # ---------------------------------------------------------------------------------------------
        self.q = q0

        self.Oi, self.Oi_prime, self.Gi = (np.zeros(3),np.zeros(3),np.zeros(3))
        self.R0i = np.eye(3, dtype=float)
        self.z, self.dzhi, self.dzii = (np.zeros(3),np.zeros(3),np.zeros(3))
        self.Rhi = np.eye(3, dtype=float)
        
        # ---------------------------------------------------------------------------------------------
        # kinematics
        # ---------------------------------------------------------------------------------------------
        self.omega = np.zeros(3, dtype=float)
        self.omegad_c = np.zeros(3, dtype=float)
        self.alpha_c = -g
        self.beta_c = np.zeros((3,3), dtype=float)
        
        self.Om = None
        self.Am = None
        
        # ---------------------------------------------------------------------------------------------
        # dynamics
        # ---------------------------------------------------------------------------------------------        
        self.Wc = np.zeros(3, dtype=float)
        self.Fc = np.zeros(3, dtype=float)
        self.Lc = np.zeros(3, dtype=float)
        
        self.Wm = None
        self.Fm = None
        self.Lm = None
         
        #external forces and torques
        self.Fext = np.zeros(3, dtype=float)
        self.Lext = np.zeros(3, dtype=float)
        
    def compute_q_dependent_variables(self):        
        self.z          = self.q[0]*self.psi
        self.dzhi       = self.inbody.z + self.dhi
        self.dzii       = self.z + self.dii
        self.Rhi        = Rmat(self.phi, self.q[0])
        
        self.R0i        = self.inbody.R0i @ self.Rhi
        self.Oi         = self.inbody.Oi + self.inbody.R0i @ self.dzhi
        self.Oi_prime   = self.Oi + self.inbody.R0i @ self.z
        self.Gi         = self.Oi + self.R0i @ self.dzii
    
    def plot(self):
        plt.plot(self.Oi[0], self.Oi[2], 'bo')
        plt.plot(self.Oi_prime[0], self.Oi_prime[2], 'go')
        plt.plot(self.Gi[0], self.Gi[2], 'ro')
        plt.plot([self.Oi[0], self.Oi_prime[0]],[self.Oi[2], self.Oi_prime[2]], 'g')
        plt.plot([self.Oi_prime[0], self.Gi[0]], [self.Oi_prime[2], self.Gi[2]], 'k')
        for child in self.children:
            plt.plot([self.Oi_prime[0],child.Oi[0]],[self.Oi_prime[2],child.Oi[2]], 'k--')
        