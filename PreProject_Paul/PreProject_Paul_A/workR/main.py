#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - the equilibrium module
 - the modal module
 - the inverse dynamic module
 - the solverkin module

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

import numpy as np

# %%===========================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData("../dataR/PreProject.mbs")

# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# =============================================================================
# Equilibrium
# =============================================================================
# mbs_data.process = 2
# mbs_equil = Robotran.MbsEquil(mbs_data)
# mbs_equil.set_options(method=1, senstol=1e-2, verbose=1)
# mbs_equil.run()

# =============================================================================
# Modal Analysis
# =============================================================================
# mbs_data.process = 4
# mbs_modal = Robotran.MbsModal(mbs_data)
# mbs_modal.set_options(save_result=1, save_anim=1, mode_ampl=0.2)
# mbs_modal.run()

# =============================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
# mbs_data.q[10] = -np.pi/3
# mbs_data.q[17] =  np.pi/3
# mbs_data.q[8]  = -2.5
# mbs_data.q[15] =  2.5
# mbs_data.q[9]  = -4.32578154575652
# mbs_data.q[16] = -4.32578154575652
# mbs_data.q[2]  =  0.0
# mbs_data.q[1:] = [8.561999e-017, 6.751184e-003, 4.314010e-018, -3.461583e-004, -2.307754e-004, 3.461583e-004, 2.307754e-004, -2.499200e+000, -4.325782e+000, -1.047116e+000, -1.732811e-004, -1.154966e-004, 1.733112e-004, 1.155521e-004, 2.499200e+000, -4.325782e+000, 1.047116e+000, -1.733112e-004, -1.155521e-004, 1.732811e-004, 1.154966e-004]


mbs_data.q[1:] = [-2.772961e-016, -3.459110e+000, 2.189960e-017, -1.154214e-004, 1.154214e-004, -2.499500e+000, -4.326019e+000, -1.047032e+000, -1.845991e-004, -1.211598e-004, 1.902723e-004, 1.240520e-004, 2.499500e+000, -4.326019e+000, 1.047032e+000, -1.902723e-004, -1.240520e-004, 1.845991e-004, 1.211598e-004]
#  1.000000e+001  -2.772961e-016  -3.459110e+000  2.189960e-017  -1.154214e-004  1.154214e-004  -2.499500e+000  -4.326019e+000  -1.047032e+000  -1.845991e-004  -1.211598e-004  1.902723e-004  1.240520e-004  2.499500e+000  -4.326019e+000  1.047032e+000  -1.902723e-004  -1.240520e-004  1.845991e-004  1.211598e-004

mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=10.0, save2file=1) #Dopri5
mbs_dirdyn.run()
# print("q8 ==   ", mbs_data.q[8])
# print("q15 ==  ", mbs_data.q[15])
# print("q9 ==   ", mbs_data.q[9])
# print("q16 ==  ", mbs_data.q[16])
# print("q2 ==   ", mbs_data.q[2])

# =============================================================================
# Inverse Kinematics
# =============================================================================
# mbs_data.process = 5
# mbs_solvekin = Robotran.MbsSolvekin(mbs_data)
# mbs_solvekin.set_options(trajectoryqname="../resultsR/dirdyn_q.res")
# mbs_solvekin.set_options(trajectoryqdname="../resultsR/dirdyn_qd.res")
# mbs_solvekin.set_options(trajectoryqddname="../resultsR/dirdyn_qdd.res")
# mbs_solvekin.set_options(t0=1.3333, tf=1.4, dt=1e-4, framerate=10000)
# mbs_solvekin.set_options(motion="trajectory")
# mbs_solvekin.run()

# =============================================================================
# Inverse Dynamics
# =============================================================================
# mbs_data.process = 6
# mbs_invdyn = Robotran.MbsInvdyn(mbs_data)
# mbs_invdyn.set_options(trajectoryqname="../resultsR/dirdyn_q.res")
# mbs_invdyn.set_options(trajectoryqdname="../resultsR/dirdyn_qd.res")
# mbs_invdyn.set_options(trajectoryqddname="../resultsR/dirdyn_qdd.res")
# mbs_invdyn.set_options(t0=0.0, tf=5.0, dt=1e-3)
# mbs_invdyn.set_options(motion="trajectory")
# mbs_invdyn.run()
