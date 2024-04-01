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

e1 = np.array([1,0,0])
e2 = np.array([0,1,0])
e3 = np.array([0,0,1])
zero = np.zeros(3)

def Rmat(phi, theta):
    return Rotation.from_rotvec(phi * theta).as_matrix()

def tilde(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

class mbs():
    def __init__(self):
        self.bodies = []   
    
    def add_body(self, inbody, joint, m=0, I=0, dii=np.zeros(3), dhi = np.zeros(3), q0=np.zeros(3)):
        self.bodies.append(body(inbody, joint, m=m, I=I, dii=dii, dhi=dhi, q0=q0))
        
    def get_body(self, id):
        return self.bodies[id]
    
    def R_and_O_of_all_bodies(self):
        for body in self.bodies:
            #orientation of the inbody
            R0h = np.eye(3)
            if(body.inbody != None):
                R0h = body.inbody.R0i
            
            #position of the inbody
            Oh = np.zeros(3)
            if(body.inbody != None):
                Oh = body.inbody.Oi
            
            #orientation of the body in the inbody frame
            body.Rhi = Rmat(body.phi, body.q[0])
            
            #orientation of the body in the absolute frame
            body.R0i = R0h @ body.Rhi
            
            #position of the body in the absolute frame
            body.Oi = Oh + R0h @ (body.dhi)
            body.Oi_prime = body.Oi + R0h@body.psi*body.q[0]
            body.Gi = body.Oi_prime + R0h @ body.Rhi @ (body.dii)
            
    def plot(self):
        self.R_and_O_of_all_bodies()
        for body in self.bodies:
            body.plot()
    
    def set_q(self, q):
        for i in range(len(self.bodies)):
            self.bodies[i].q[0] = q[i]

    def compute_barycentric_quantities(self):
        for body in reversed(self.bodies):
            body.m_bar = body.m + sum(map(lambda x:x.m_bar, body.children))
            body.b = (body.m*body.dii + sum(map(lambda x:x.m_bar*x.dhi, body.children)))/body.m_bar
            body.K = body.I - sum(map(lambda x:x.m_bar*tilde(x.dhi)@tilde(x.dhi),body.children))
    
    def forward_kinematics(self):
        # initialisation
        for i in self.bodies:
            i.omega = np.zeros(3); i.omegad_c = np.zeros(3)
            i.alpha_c = np.array([0,0,-9.81]); i.beta_c = np.zeros((3,3))
            i.Om = np.zeros((len(self.bodies),3))
            i.Am = np.zeros((len(self.bodies),3))
            
        for i in self.bodies:
            Rhi = Rmat(i.phi, i.q[0])
            dzhi = i.dhi + i.psi*i.q[0]

            h = i.inbody
            if(h == None):
                h = i
                
            i.omega    = Rhi.T @ h.omega    + i.phi*i.q[1]
            i.omegad_c = Rhi.T @ h.omegad_c + tilde(i.omega)@i.phi*i.q[1]
            i.beta_c   = tilde(i.omegad_c)  + tilde(i.omega)@tilde(i.omega)
            i.alpha_c  = Rhi.T @ (h.alpha_c + h.beta_c@dzhi) + 2*tilde(i.omega)@i.psi*i.q[1]
            for k in range(i.id + 1):
                i.Om[k] = Rhi.T @  h.Om[k] + (k == i.id)*i.phi
                i.Am[k] = Rhi.T @ (h.Am[k] + tilde(h.Om[k])@dzhi) + (k == i.id)*i.psi
          
          
    def backward_dynamics(self):
        for i in reversed(self.bodies):
            dzii = i.dii + i.psi*i.q[0]
            i.Wc = i.m*(i.alpha_c + i.beta_c@dzii) - i.Fext
            i.Fc = sum(map(lambda x:x.Fc, i.children)) + i.Wc
            i.Lc = sum(map(lambda x:x.Lc + tilde(x.dhi), i.children)) + i.b@i.Wc + tilde(i.dii)@i.Fc
            
            
      
class body():
    
    N = 0
    
    def __init__(self, inbody, joint, m=0, I=np.zeros((3,3)), dii=np.zeros(3), dhi = np.zeros(3), q0=np.zeros(3)):
        # parameters of the body
        self.m = m; self.I = I; self.dii = dii; self.dhi = dhi

        # initialisation of the generalised coordinates
        self.q = q0
        
        # definition of the inbody and id
        self.inbody = inbody
        self.id = body.N
        body.N += 1
        
        # creation of the absolute position and orientation of the body
        self.Oi = np.zeros(3)
        self.Oi_prime = np.zeros(3)
        self.R0i = np.eye(3)
        self.Gi = np.zeros(3)
        
        # definition of the joint
        dico = {"T1":(e1,zero), "T2":(e2,zero), "T3":(e3,zero), "R1":(zero,e1), "R2":(zero,e2), "R3":(zero,e3)}
        self.psi, self.phi = dico[joint]
        
        # children
        self.children = []
        if(self.inbody != None):
            self.inbody.children.append(self)
            
        # barycentric quantities
        self.m_bar = 0
        self.b = np.zeros(3)
        self.K = np.zeros((3,3))
        
        
        # kinematic quantities
        self.omega = np.zeros(3)
        self.omegad_c = np.zeros(3)
        self.alpha_c = np.array([0,0,-9.81])
        self.beta_c = np.zeros((3,3))
        
        self.Om = None
        self.Am = None
        
        # dynamic quantities
        self.Wc = np.zeros(3)
        self.Fc = np.zeros(3)
        self.Lc = np.zeros(3)
        
        self.Wm = None
        self.Fm = None
        self.Lm = None
         
        #external forces and torques
        self.Fext = np.zeros(3)
        self.Lext = np.zeros(3)
    
    def initialise_kinematics(self):
        self.omega = np.zeros(3)
        self.omegad_c = np.zeros(3)
        self.alpha_c = np.array([0,0,-9.81])
        self.beta_c = np.zeros((3,3))
        self.Om = None
        self.Am = None
    
    def plot(self):
        plt.plot(self.Oi[0], self.Oi[2], 'bo')
        plt.plot(self.Oi_prime[0], self.Oi_prime[2], 'go')
        plt.plot(self.Gi[0], self.Gi[2], 'ro')
        plt.plot([self.Oi[0], self.Oi_prime[0]],[self.Oi[2], self.Oi_prime[2]], 'g')
        plt.plot([self.Oi_prime[0], self.Gi[0]], [self.Oi_prime[2], self.Gi[2]], 'k')
        for child in self.children:
            plt.plot([self.Oi_prime[0],child.Oi[0]],[self.Oi_prime[2],child.Oi[2]], 'k--')
        
        
MBS = mbs()

I = np.array([[0,0,0],[0,1,0],[0,0,0]])

MBS.add_body(None,"R2", m=1, I=I, dii=np.array([0,0,-1]), q0 = np.array([0.5,0,0]))
MBS.add_body(MBS.bodies[-1],"R2", m=1, I=I, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]), q0=np.array([0.3,0,0]))
MBS.add_body(MBS.bodies[-1],"R2", m=1, I=I, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]))
MBS.add_body(MBS.bodies[-3],"T3", m=1, I=I, dii=np.array([0,0,-1]), dhi=np.array([0,0,-2]), q0=np.array([-0.5,0,0]))

MBS.set_q([0.1,0.2,0.3,0.4])

MBS.compute_barycentric_quantities()
MBS.forward_kinematics()

def animate(i):
    MBS.set_q([1*sin(i/10),3*cos(i/10),0.5*sin(i/5),-1-0.6*cos(i/5)])
    plt.cla()
    MBS.plot()
    plt.xlim(-10,10)
    plt.ylim(-10,10)

ani = FuncAnimation(plt.gcf(), animate, interval=12)
plt.axis('equal')
plt.show()

print([x for x in map(lambda x:x.id, MBS.bodies)])
