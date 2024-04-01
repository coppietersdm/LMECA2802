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
    
    def add_body(self, inbody, joint, m=0, I=0, dii=np.zeros(3), dhi = np.zeros(3), q0=np.zeros(3)):
        self.bodies.append(body(inbody, joint, m=m, I=I, dii=dii, dhi=dhi, q0=q0))
    
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
    
    def second_derivative(self):
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
        Q = np.zeros(len(self.bodies))
        Q[1] = - 10*self.bodies[1].q[0] #-1*self.bodies[1].q[1]
        qdd = np.linalg.solve(M,-c+Q)
        for body in self.bodies:
            body.q[2] = qdd[body.id-1]
            
    def update(self, dt):
        q = self.get_q()
        qd = self.get_qd()
        
        self.second_derivative()
        k1 = self.get_qd()
        k1d = self.get_qdd()
        self.set_q(q + k1*dt/2)
        self.set_qd(qd + k1d*dt/2)
        
        self.second_derivative()
        k2 = self.get_qd()
        k2d = self.get_qdd()
        self.set_q(q + k2*dt/2)
        self.set_qd(qd + k2d*dt/2)
        
        self.second_derivative()
        k3 = self.get_qd()
        k3d = self.get_qdd()
        self.set_q(q + k3*dt)
        self.set_qd(qd + k3d*dt)
        
        self.second_derivative()
        k4 = self.get_qd()
        k4d = self.get_qdd()
        
        self.set_q(q + (k1+2*k2+2*k3+k4)*dt/6)
        self.set_qd(qd + (k1d+2*k2d+2*k3d+k4d)*dt/6)
        
      
class body():
    N = 0
    def __init__(self, inbody, joint, m=0, I=np.zeros((3,3)), dii=np.zeros(3), dhi = np.zeros(3), q0=np.zeros(3)):
        # ---------------------------------------------------------------------------------------------
        # body constants
        # ---------------------------------------------------------------------------------------------
        self.m = m; self.I = I
        self.dii = dii; self.dhi = dhi

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
        self.R0i = np.eye(3)
        self.z, self.dzhi, self.dzii = (np.zeros(3),np.zeros(3),np.zeros(3))
        self.Rhi = np.eye(3)
        
        # ---------------------------------------------------------------------------------------------
        # kinematics
        # ---------------------------------------------------------------------------------------------
        self.omega = np.zeros(3)
        self.omegad_c = np.zeros(3)
        self.alpha_c = -g
        self.beta_c = np.zeros((3,3))
        
        self.Om = None
        self.Am = None
        
        # ---------------------------------------------------------------------------------------------
        # dynamics
        # ---------------------------------------------------------------------------------------------        
        self.Wc = np.zeros(3)
        self.Fc = np.zeros(3)
        self.Lc = np.zeros(3)
        
        self.Wm = None
        self.Fm = None
        self.Lm = None
         
        #external forces and torques
        self.Fext = np.zeros(3)
        self.Lext = np.zeros(3)
        
    
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
        

MBS = mbs()

MBS.add_body(MBS.base,"R2", m=1, I=I2, dii=np.array([0,0,-1]), q0 = np.array([pi/2,0,0]))
MBS.add_body(MBS.bodies[-1],"T3", m=1, I=I2, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]), q0=np.array([0.1,0,0]))
MBS.add_body(MBS.bodies[-1],"R2", m=1, I=I2, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]))
MBS.add_body(MBS.bodies[-1],"R2", m=1, I=I2, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]), q0=np.array([-0.5,0,0]))
# MBS.add_body(MBS.bodies[-1],"R2", m=1, I=I2, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]))
# MBS.add_body(MBS.bodies[-1],"T1", m=1, I=I2, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]))



def animate(i):
    # MBS.set_q([1*sin(i/10),3*cos(i/10)]) #,0.5*sin(i/5),-1-0.6*cos(i/5), 0.5*sin(i/10), 0.5*cos(i/10)])
    for i in range(10):
        MBS.update(0.001)
    plt.cla()
    MBS.plot()
    plt.xlim(-10,10)
    plt.ylim(-10,10)

ani = FuncAnimation(plt.gcf(), animate, interval=10)
plt.axis('equal')
plt.show()

print([x for x in map(lambda x:x.id, MBS.bodies)])
