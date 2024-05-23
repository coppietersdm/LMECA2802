# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    mbs_data.Qq[1:] = 0.
    
    circular_spring_id = [mbs_data.joint_id["Joint_3"],mbs_data.joint_id["Joint_5"],mbs_data.joint_id["Joint_13"],mbs_data.joint_id["Joint_14"],mbs_data.joint_id["Joint_15"],mbs_data.joint_id["Joint_16"],mbs_data.joint_id["Joint_17"],mbs_data.joint_id["Joint_18"],mbs_data.joint_id["Joint_19"],mbs_data.joint_id["Joint_20"]]
    Kflex = 8.5e6
    Dflex = 1.0e4

    for j in circular_spring_id:
        mbs_data.Qq[j] = -Kflex * mbs_data.q[j] - Dflex * mbs_data.qd[j]


    # Example: damping in joint number 5
    # D = 0.5 # N/(m/s)
    # mbs_data.Qq[5] = -D * mbs_data.qd[5]

    return
