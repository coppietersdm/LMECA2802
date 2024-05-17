from math import sin, cos, pi
import numpy as np
from scipy.integrate import solve_ivp, odeint
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.animation import FuncAnimation

e1 = np.array([1.0,0.0,0.0])
e2 = np.array([0.0,1.0,0.0])
e3 = np.array([0.0,0,1.0])
zero = np.array([0.0,0.0,0.0])
I1 = np.array([[1.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])

g = np.array([0,0,9.81])


def Rmat(phi, theta):
    return Rotation.from_rotvec(phi * theta).as_matrix()

def tilde(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

class mbs():
    def __init__(self):
        self.bodies = []
        self.base = body(None, "T1", anchor_points=[np.array([0.0,5.0,0.0]),np.array([0.0,-5.0,0.0])])
        self.q = None
        self.qd = None
        self.qdd = None
        self.t = None
        self.position_sensors = [(self.base,0),(self.base,1)]
    
    def read_q(self):
        self.q = np.loadtxt('/home/matthieu/Documents/LMECA2802/selfmade/dirdyn_q.res')
        self.qd = np.loadtxt('/home/matthieu/Documents/LMECA2802/selfmade/dirdyn_qd.res')
    
    def add_body(self, inbody, joint, m=0.0, I=I1*0, dii=np.array([0.0,0.0,0.0]), dhi = np.array([0.0,0.0,0.0]), q0=np.array([0.0,0.0,0.0]), Q=lambda t,q,qd: 0.0, anchor_points = []):
        self.bodies.append(body(inbody, joint, m=m, I=I, dii=dii, dhi=dhi, q0=q0, Q=Q, anchor_points=anchor_points))
        for i in range(len(anchor_points)):
            self.position_sensors.append((self.bodies[-1],i))
        
    def compute_q_dependent_variables(self):
        for body in self.bodies:
            body.compute_q_dependent_variables()

    def compute_q_and_qd_dependent_variables(self):
        for body in self.bodies:
            body.compute_q_and_qd_dependent_variables()
            
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

    def forward_kinematics(self):
        # initialisation
        for i in ([self.base] + self.bodies):
            i.Om = np.array([[0.0,0.0,0.0]]*len(self.bodies))
            i.Am = np.array([[0.0,0.0,0.0]]*len(self.bodies))
        
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
            i.Wm = np.array([[0.0,0.0,0.0]]*len(self.bodies))
            i.Fm = np.array([[0.0,0.0,0.0]]*len(self.bodies))
            i.Lm = np.array([[0.0,0.0,0.0]]*len(self.bodies))
            
        for i in reversed(self.bodies):
            i.Wc = i.m*(i.alpha_c + i.beta_c@i.dzii) - i.Fext
            i.Fc = sum(map(lambda j:j.Rhi @ j.Fc, i.children)) + i.Wc
            i.Lc = sum(map(lambda j:j.Rhi @ j.Lc + tilde(j.dzhi)@j.Rhi@j.Fc, i.children)) + tilde(i.dzii)@i.Wc - i.Lext + i.I@i.omegad_c + tilde(i.omega)@i.I@i.omega
            for k in range(i.id):
                i.Wm[k] = i.m*(i.Am[k]+tilde(i.Om[k])@i.dzii)
                i.Fm[k] = sum(map(lambda j:j.Rhi @ j.Fm[k], i.children)) + i.Wm[k]
                i.Lm[k] = sum(map(lambda j:j.Rhi @ j.Lm[k] + tilde(j.dzhi)@j.Rhi @ j.Fm[k], i.children)) + tilde(i.dzii)@i.Wm[k] + i.I@i.Om[k]
    
    def spring(self, anchor_point1, anchor_point2, plot = False, prt = False):
        body1 = self.position_sensors[anchor_point1][0]
        x1 = body1.anchor_points[self.position_sensors[anchor_point1][1]]
        Ox1 = body1.anchor_points_inertial_frame[self.position_sensors[anchor_point1][1]]
        Odx1 = body1.anchor_points_speed_inertial_frame[self.position_sensors[anchor_point1][1]]
        
        body2 = self.position_sensors[anchor_point2][0]
        x2 = body2.anchor_points[self.position_sensors[anchor_point2][1]]
        Ox2 = body2.anchor_points_inertial_frame[self.position_sensors[anchor_point2][1]]
        Odx2 = body2.anchor_points_speed_inertial_frame[self.position_sensors[anchor_point2][1]]
        
        body1.Fext += -body1.R0i.T @(1e6* (Ox1 - Ox2) + 1e4*(Odx1 - Odx2))
        body2.Fext += -body2.R0i.T @(1e6* (Ox2 - Ox1) + 1e4*(Odx2 - Odx1))
        
        body1.Lext += -tilde(x1-body1.dii)@body1.R0i.T @(1e6* (Ox1 - Ox2) + 1e4*(Odx1 - Odx2))
        body2.Lext += -tilde(x2-body2.dii)@body2.R0i.T @(1e6* (Ox2 - Ox1) + 1e4*(Odx2 - Odx1))
        if(plot):
            plt.plot([Ox1[1],Ox2[1]],[-Ox1[2],-Ox2[2]], 'r')
        return (1e6* (Ox1 - Ox2) + 1e4*(Odx1 - Odx2))
                    
    def compute_external_forces(self,t, prt = False):
        #return
        for body in self.bodies:
            body.Fext = np.array([0.0,0.0,0.0])
            body.Lext = np.array([0.0,0.0,0.0])
        self.spring(0,6,prt=prt)
        self.spring(1,5,prt=prt)
        
        self.spring(2,6,prt=prt)
        self.spring(3,5,prt=prt)
        self.spring(4,7,prt=prt)
        if(prt):
            for i, body in enumerate(self.bodies):
                print(i, np.round(body.R0i @ body.Fext/1e6,2), np.round(body.R0i @ body.Lext/1e6,2))
        
        pos_sens = self.position_sensors[4]
        pos_sens[0].Fext += pos_sens[0].R0i.T @ np.array([0.0,50.0e3,0.0])*t/10
        x2 = pos_sens[0].anchor_points[pos_sens[1]]
        Ox2 = pos_sens[0].anchor_points_inertial_frame[pos_sens[1]]
        pos_sens[0].Lext += tilde(x2-pos_sens[0].dii)@pos_sens[0].R0i.T @ np.array([0.0,50.0e3,0.0])*t/10
        
        return
        
        
    
    def second_derivative(self,t, prt = False):
        self.compute_q_dependent_variables()
        self.forward_kinematics()
        self.compute_q_and_qd_dependent_variables()
        self.compute_external_forces(t, prt = prt)
        self.backward_dynamics()
        print(t)
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
        self.set_q([9.276096e-18, 6.751184e-03, -7.835096e-20, -3.461583e-04, -2.307754e-04, 3.461583e-04, 2.307754e-04, -2.499200e+00, -4.325782e+00, -1.047116e+00, -1.732811e-04, -1.154966e-04, 1.733112e-04, 1.155521e-04, 2.499200e+00, -4.325782e+00, 1.047116e+00, -1.733112e-04, -1.155521e-04, 1.732811e-04, 1.154966e-04])
        q = self.get_q()
        # q[7] += 1.0
        self.set_q(q)
        y0 = np.array(list(self.get_q()) + list(self.get_qd()))
        def derivative(t,y, prt = False):
            self.set_q(y[:len(self.bodies)])
            self.set_qd(y[len(self.bodies):2*len(self.bodies)])
            self.second_derivative(t, prt = prt)
            return np.array(list(self.get_qd()) + list(self.get_qdd()))
        self.t = np.round(np.arange(0.0,t+dt,dt),4)
        
        sol = solve_ivp(fun=lambda t,y: derivative(t,y), t_span=(0,t), y0=y0, t_eval=self.t, method = 'BDF')
        
        self.q = np.zeros((len(self.t), len(self.bodies)+1))
        self.qd = np.zeros((len(self.t), len(self.bodies)+1))
        self.qdd = np.zeros((len(self.t), len(self.bodies)+1))
        
        self.q.T[1:] = sol.y[:len(self.bodies)]
        self.qd.T[1:] = sol.y[len(self.bodies):2*len(self.bodies)]
        self.qdd.T[1:] = np.array([derivative(t,y,prt = True)[len(self.bodies):] for y in sol.y.T[0:1]]).T
        
        self.q.T[0] = self.t
        self.qd.T[0] = self.t
        self.qdd.T[0] = self.t
        
        np.savetxt('/home/matthieu/Documents/LMECA2802/selfmade/dirdyn_q.res', self.q,fmt='%.6e')
        np.savetxt('/home/matthieu/Documents/LMECA2802/selfmade/dirdyn_qd.res', self.qd,fmt='%.6e')
        np.savetxt('/home/matthieu/Documents/LMECA2802/selfmade/dirdyn_qdd.res', self.qdd,fmt='%.6e')
        
                
class body():
    N = 0
    def __init__(self, inbody, joint, m=0.0, I=I1*0, dii=np.array([0.0,0.0,0.0]), dhi = np.array([0.0,0.0,0.0]), q0=np.array([0.0,0.0,0.0]), Q=lambda t,q,qd:0.0, anchor_points = []):
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

        self.Oi, self.Oi_prime, self.Gi = (np.array([0.0,0.0,0.0]),np.array([0.0,0.0,0.0]),np.array([0.0,0.0,0.0]))
        self.R0i = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.z, self.dzhi, self.dzii = (np.array([0.0,0.0,0.0]),np.array([0.0,0.0,0.0]),np.array([0.0,0.0,0.0]))
        self.Rhi = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        
        # ---------------------------------------------------------------------------------------------
        # q and qd dependent variables
        # ---------------------------------------------------------------------------------------------
        self.Oid = np.array([0.0,0.0,0.0])
        
        # ---------------------------------------------------------------------------------------------
        # kinematics
        # ---------------------------------------------------------------------------------------------
        self.omega = np.array([0.0,0.0,0.0])
        self.omegad_c = np.array([0.0,0.0,0.0])
        self.alpha_c = -g
        self.beta_c = np.array([0.0,0.0,0.0])
        
        self.Om = None
        self.Am = None
        
        # ---------------------------------------------------------------------------------------------
        # dynamics
        # ---------------------------------------------------------------------------------------------        
        self.Wc = np.array([0.0,0.0,0.0])
        self.Fc = np.array([0.0,0.0,0.0])
        self.Lc = np.array([0.0,0.0,0.0])
        
        self.Wm = None
        self.Fm = None
        self.Lm = None
         
        #external forces and torques
        self.Fext = np.array([0.0,0.0,0.0])
        self.Lext = np.array([0.0,0.0,0.0])
        
        #position sensors
        self.anchor_points = anchor_points
        self.anchor_points_inertial_frame = anchor_points
        self.anchor_points_speed_inertial_frame = np.array(anchor_points)*0
        
        
    def compute_q_dependent_variables(self):
        self.z          = self.q[0]*self.psi
        self.dzhi       = self.inbody.z + self.dhi
        self.dzii       = self.z + self.dii
        self.Rhi        = Rmat(self.phi, self.q[0])
        
        self.R0i        = self.inbody.R0i @ self.Rhi
        self.Oi         = self.inbody.Oi + self.inbody.R0i @ self.dzhi
        self.Oi_prime   = self.Oi + self.inbody.R0i @ self.z
        self.Gi         = self.Oi + self.R0i @ self.dzii
        
        self.anchor_points_inertial_frame = list(map(lambda x: self.Oi + self.R0i @ x, self.anchor_points))

        
    def compute_q_and_qd_dependent_variables(self):
        self.Oid = self.inbody.Oid + self.inbody.R0i@(tilde(self.inbody.omega)@self.dzhi + self.inbody.psi*self.inbody.q[1])
        self.anchor_points_speed_inertial_frame = list(map(lambda x: self.Oid + self.R0i@tilde(self.omega)@x + self.psi*self.q[1], self.anchor_points))
        
    def plot(self):
        plt.plot(self.Oi[1], -self.Oi[2], 'bo')
        plt.plot(self.Oi_prime[1], -self.Oi_prime[2], 'go')
        plt.plot(self.Gi[1], -self.Gi[2], 'ro')
        plt.plot([self.Oi[1], self.Oi_prime[1]],[-self.Oi[2], -self.Oi_prime[2]], 'g')
        plt.plot([self.Oi_prime[1], self.Gi[1]], [-self.Oi_prime[2], -self.Gi[2]], 'k')
        for child in self.children:
            plt.plot([self.Oi_prime[1],child.Oi[1]],[-self.Oi_prime[2],-child.Oi[2]], 'k--')
            
        for anchor_point in self.anchor_points_inertial_frame:
            plt.plot([self.Oi_prime[1],anchor_point[1]],[-self.Oi_prime[2],-anchor_point[2]], 'y--')
            plt.plot(anchor_point[1],-anchor_point[2], 'yo')
        