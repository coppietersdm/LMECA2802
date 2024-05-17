# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019
import numpy as  np
from MBsysPy.mbsyspy.mbs_sensor import *

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """
    Kconn = 1.0e6
    Dconn = 1.0e4
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]

    if ixF == mbs_data.extforce_id["ExtForce_0"]:
        P_Y1 = mbs_data.S1_P[2]
        P_Z1 = mbs_data.S1_P[3]
        V_Y1 = mbs_data.S1_V[2]
        V_Z1 = mbs_data.S1_V[3]
        
        Fy = -((PxF[2]-P_Y1)*Kconn+Dconn*(VxF[2]-V_Y1))
        Fz = -((PxF[3]-P_Z1)*Kconn+Dconn*(VxF[3]-V_Z1))

        #Wind force
        # Fy += 5000.0*tsim

    elif ixF == mbs_data.extforce_id["ExtForce_1"]:
        P_Y0 = mbs_data.S0_P[2]
        P_Z0 = mbs_data.S0_P[3]
        V_Y0 = mbs_data.S0_V[2]
        V_Z0 = mbs_data.S0_V[3]

        Fy = -((PxF[2]-P_Y0)*Kconn+Dconn*(VxF[2]-V_Y0))
        Fz = -((PxF[3]-P_Z0)*Kconn+Dconn*(VxF[3]-V_Z0))

    elif ixF == mbs_data.extforce_id["ExtForce_2"]:
        P_Y3 = mbs_data.S3_P[2]
        P_Z3 = mbs_data.S3_P[3]
        V_Y3 = mbs_data.S3_V[2]
        V_Z3 = mbs_data.S3_V[3]

        Fy = -((PxF[2]-P_Y3)*Kconn+Dconn*(VxF[2]-V_Y3))
        Fz = -((PxF[3]-P_Z3)*Kconn+Dconn*(VxF[3]-V_Z3))
    
        #wall force
        FyWall = -((PxF[2]+5)*Kconn+Dconn*VxF[2])
        FzWall = -((PxF[3])*Kconn+Dconn*VxF[3])

        mbs_data.WallHorF_left.append(FyWall)
        mbs_data.WallVertF_left.append(FzWall)

        Fy += FyWall
        Fz += FzWall

    elif ixF == mbs_data.extforce_id["ExtForce_3"]:
        P_Y2 = mbs_data.S2_P[2]
        P_Z2 = mbs_data.S2_P[3]
        V_Y2 = mbs_data.S2_V[2]
        V_Z2 = mbs_data.S2_V[3]

        Fy = -((PxF[2]-P_Y2)*Kconn+Dconn*(VxF[2]-V_Y2))
        Fz = -((PxF[3]-P_Z2)*Kconn+Dconn*(VxF[3]-V_Z2))

    elif ixF == mbs_data.extforce_id["ExtForce_4"]:
        P_Y5 = mbs_data.S5_P[2]
        P_Z5 = mbs_data.S5_P[3]
        V_Y5 = mbs_data.S5_V[2]
        V_Z5 = mbs_data.S5_V[3]

        Fy = -((PxF[2]-P_Y5)*Kconn+Dconn*(VxF[2]-V_Y5))
        Fz = -((PxF[3]-P_Z5)*Kconn+Dconn*(VxF[3]-V_Z5))

        FyWall = -((PxF[2]-5)*Kconn+Dconn*VxF[2])
        FzWall = -((PxF[3])*Kconn+Dconn*VxF[3])
        
        mbs_data.WallHorF_right.append(FyWall)
        mbs_data.WallVertF_right.append(FzWall)

        #wall
        Fy += FyWall
        Fz += FzWall


    elif ixF == mbs_data.extforce_id["ExtForce_5"]:
        P_Y4 = mbs_data.S4_P[2]
        P_Z4 = mbs_data.S4_P[3]
        V_Y4 = mbs_data.S4_V[2]
        V_Z4 = mbs_data.S4_V[3]

        Fy = -((PxF[2]-P_Y4)*Kconn+Dconn*(VxF[2]-V_Y4))
        Fz = -((PxF[3]-P_Z4)*Kconn+Dconn*(VxF[3]-V_Z4))






    # Example : Contact force with a wall when X coordinate is higher than 1m.
    #           The force is perfectly horizontal (inertial frame)
    # xlim = 1.0 # m
    # kwall= 1e5 # N/m
    # if PxF[1]>xlim:
    #     Fx = (PxF[1]-xlim)*kwall

    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]

    return Swr
