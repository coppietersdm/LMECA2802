# -*- coding: utf-8 -*-
"""Module for the definition of functions related to Equilibrium analysis."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

import MBsysPy as Robotran
import matplotlib.pyplot as plt
import numpy as np
def user_dirdyn_init(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user before running direct dynamic.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """

    # Example: Creating and storing a sensor in mbs_data then create a list to store
    #          the vertical velocity of the sensor. Two new fields are added to
    #          the MbsData instance to store the sensor and the list.
    #
    # import MBsysPy as Robotran # Should be done outside the function.
    #

    mbs_data.S0 = Robotran.MbsSensor(mbs_data)
    mbs_data.S0_P = [0,0,0,0]
    mbs_data.S0_V = [0,0,0,0]

    mbs_data.S1 = Robotran.MbsSensor(mbs_data)
    mbs_data.S1_P = [0,0,0,0]
    mbs_data.S1_V = [0,0,0,0]

    mbs_data.S2 = Robotran.MbsSensor(mbs_data)
    mbs_data.S2_P = [0,0,0,0]
    mbs_data.S2_V = [0,0,0,0]

    mbs_data.S3 = Robotran.MbsSensor(mbs_data)
    mbs_data.S3_P = [0,0,0,0]
    mbs_data.S3_V = [0,0,0,0]
    
    mbs_data.S4 = Robotran.MbsSensor(mbs_data)
    mbs_data.S4_P = [0,0,0,0]
    mbs_data.S4_V = [0,0,0,0]

    mbs_data.S5 = Robotran.MbsSensor(mbs_data)
    mbs_data.S5_P = [0,0,0,0]
    mbs_data.S5_V = [0,0,0,0]

    mbs_data.WallVertF_left = [0,0,0,0]
    mbs_data.WallHorF_left = [0,0,0,0]
    mbs_data.WallVertF_right = [0,0,0,0]
    mbs_data.WallHorF_right = [0,0,0,0]

    mbs_data.RoofLatP_left = [0,0,0,0]
    mbs_data.RoofLatP_right = [0,0,0,0]

    return


def user_dirdyn_loop(mbs_data, mbs_dirdyn):
    """Run specific operation required by the user at the end of each integrator step.

    In case of multistep integrator, this function is not called at intermediate
    steps.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.
    """

    # Example: The sensor, created in `user_dirdyn_init`, is computed (fields
    #          are updated) and the vertical velocity is added in the list.
    #
    mbs_data.S0.comp_s_sensor(mbs_data.extforce_id["ExtForce_0"])
    mbs_data.S0_P = [0,mbs_data.S0.P[1],mbs_data.S0.P[2],mbs_data.S0.P[3]]
    mbs_data.S0_V = [0,mbs_data.S0.V[1],mbs_data.S0.V[2],mbs_data.S0.V[3]]
    mbs_data.RoofLatP_left.append(mbs_data.S0.P[2])

    mbs_data.S1.comp_s_sensor(mbs_data.extforce_id["ExtForce_1"])
    mbs_data.S1_P = [0,mbs_data.S1.P[1],mbs_data.S1.P[2],mbs_data.S1.P[3]]
    mbs_data.S1_V = [0,mbs_data.S1.V[1],mbs_data.S1.V[2],mbs_data.S1.V[3]]
    # print("premier en theorie ###################################")
    # print(len(mbs_data.S1_P))
    mbs_data.RoofLatP_right.append(mbs_data.S1.P[2])


    mbs_data.S2.comp_s_sensor(mbs_data.extforce_id["ExtForce_2"])
    mbs_data.S2_P = [0,mbs_data.S2.P[1],mbs_data.S2.P[2],mbs_data.S2.P[3]]
    mbs_data.S2_V = [0,mbs_data.S2.V[1],mbs_data.S2.V[2],mbs_data.S2.V[3]]

    mbs_data.S3.comp_s_sensor(mbs_data.extforce_id["ExtForce_3"])
    mbs_data.S3_P = [0,mbs_data.S3.P[1],mbs_data.S3.P[2],mbs_data.S3.P[3]]
    mbs_data.S3_V = [0,mbs_data.S3.V[1],mbs_data.S3.V[2],mbs_data.S3.V[3]]

    mbs_data.S4.comp_s_sensor(mbs_data.extforce_id["ExtForce_4"])
    mbs_data.S4_P = [0,mbs_data.S4.P[1],mbs_data.S4.P[2],mbs_data.S4.P[3]]
    mbs_data.S4_V = [0,mbs_data.S4.V[1],mbs_data.S4.V[2],mbs_data.S4.V[3]]

    mbs_data.S5.comp_s_sensor(mbs_data.extforce_id["ExtForce_5"])
    mbs_data.S5_P = [0,mbs_data.S5.P[1],mbs_data.S5.P[2],mbs_data.S5.P[3]]
    mbs_data.S5_V = [0,mbs_data.S5.V[1],mbs_data.S5.V[2],mbs_data.S5.V[3]]

    return


def user_dirdyn_finish(mbs_data, mbs_dirdyn):
    """Run specific operations required by the user when direct dynamic analysis ends.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    mbs_dirdyn : MBsysPy.MbsDirdyn
        The instance of the current direct dynamic process.

    Returns
    -------
    None.

    """
    plt.plot(np.linspace(0,50000,len(mbs_data.RoofLatP_left)),mbs_data.RoofLatP_left, label='left')
    plt.plot(np.linspace(0,50000,len(mbs_data.RoofLatP_right)),mbs_data.RoofLatP_right, label='right')
    plt.xlabel('Wind Force [N]')
    plt.ylabel("Horizontal deplacement top structure [m]")
    plt.legend()
    plt.show()
    # print("Roof lat pos left init", mbs_data.RoofLatP_left[0:10])
    # print("Roof lat pos right init", mbs_data.RoofLatP_right[0:10])

    print("Roof lat pos left:  ", mbs_data.RoofLatP_left[-1])
    print("Roof lat pos right: ", mbs_data.RoofLatP_right[-1])

    plt.plot(mbs_data.WallVertF_left, label = 'left')
    plt.plot(mbs_data.WallVertF_right, label = 'right')
    plt.xlabel("t")
    plt.ylabel("Wall Vertical Force [N]")
    plt. legend()
    plt.show()

    plt.plot(mbs_data.WallHorF_left, label ='left')
    plt.plot(mbs_data.WallHorF_right, label ='right')
    plt.xlabel("t")
    plt.ylabel("Wall Horizontal Force [N]")
    plt.legend()
    plt.show()
    # Example: The velocities are saved to a file, then the fields created in
    #          the MbsData instance during the function `user_dirdyn_init` are
    #          removed.
    #
    # import numpy as np # Should be done outside the function.
    #
    # np.savetxt("myfile.txt", mbs_data.my_sensor_v)
    # del mbs_data.my_sensor, mbs_data.my_sensor_v

    return
