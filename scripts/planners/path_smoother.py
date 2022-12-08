import numpy as np
import scipy.interpolate


def compute_smoothed_traj(path, V_des, k, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        k (int): The degree of the spline fit.
            For this assignment, k should equal 3 (see documentation for
            scipy.interpolate.splrep)
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        t_smoothed (np.array [N]): Associated trajectory times
        traj_smoothed (np.array [N,7]): Smoothed trajectory
    Hint: Use splrep and splev from scipy.interpolate
    """
    assert(path and k > 2 and k < len(path))
    ########## Code starts here ##########
    # Hint 1 - Determine nominal time for each point in the path using V_des
    # Hint 2 - Use splrep to determine cubic coefficients that best fit given path in x, y
    # Hint 3 - Use splev to determine smoothed paths. The "der" argument may be useful.
    pathN = np.asarray(path).shape[0] -1 # before goal
    pathA = np.asarray(path)

    t_c = 0
    t_s = [0]

    # Get all time steps
    for i in range(pathN):
        t_c += 1/V_des*np.linalg.norm(np.array(path[i])-np.array(path[i+1]))
        t_s.append(t_c)
    
    
    # TIME
    t_final = t_s[-1]
    t_s = np.asarray(t_s)
    # Get the x_spline & y_spline Get the spline once then replcae teh values
    x_spl = scipy.interpolate.splrep(t_s, pathA[:,0], s=alpha)
    y_spl = scipy.interpolate.splrep(t_s, pathA[:,1], s=alpha)

    tsmooth = np.asarray(np.linspace(0, t_final, int(np.ceil(t_final / dt))))
    t_smoothed = tsmooth

    # POSE
    x_d=scipy.interpolate.splev(t_smoothed, x_spl, der=0)
    y_d=scipy.interpolate.splev(t_smoothed, y_spl, der=0)

    # POSE DOT
    xd_d=scipy.interpolate.splev(t_smoothed, x_spl, der=1)
    yd_d=scipy.interpolate.splev(t_smoothed, y_spl, der=1)
    theta_d = np.arctan2(yd_d,xd_d)

    # DOUBLE DOT
    xdd_d=scipy.interpolate.splev(t_smoothed, x_spl, der=2)
    ydd_d=scipy.interpolate.splev(t_smoothed, y_spl, der=2)

    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
