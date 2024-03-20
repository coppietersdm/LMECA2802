# -*- coding: utf-8 -*-
"""
Module for the definition of the derivatives of the user states.

Summary
-------
The user derivatives allows to add some equations to be time-integrated with
the multibody system.
"""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_derivatives(mbs_data):
    """Compute the derivatives of the user states.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project. The derivatives values
        must be filled in the 'uxd' attribute which is a numpy.ndarray. The first
        index (mbs_data.uxd[0]) must not be modified. The first index to be
        filled is mbs_data.uxd[1].

    Returns
    -------
    None
    """

    # Example: integration of sin(t) in first user state variable
    # mbs_data.uxd[1] = np.cos(mbs_data.tsim)
    
    id_crank = mbs_data.joint_id['R2_crank']
    rho = mbs_data.user_model['Motor']['rho']
    U   = mbs_data.user_model['Motor']['U']
    Rmot= mbs_data.user_model['Motor']['Rmot']
    kphi= mbs_data.user_model['Motor']['Kphi']
    L   = mbs_data.user_model['Motor']['L']

    # The user derivative index are retrieved in the user model state value.
    # This is important if your model contain more than one user state.
    id_state =  mbs_data.user_model['Motor']['i'][1]

    omMot = -mbs_data.qd[id_crank]*rho
    iMot = mbs_data.ux[id_state]
    mbs_data.uxd[id_state] = (U-Rmot*iMot-kphi*omMot)/L

    return
