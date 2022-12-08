import numpy as np

EPSILON_OMEGA = 1e-3

def compute_Gx(xvec, u, dt):
    """
    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
    Outputs:
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
    """
    ########## Code starts here ##########
    # TODO: Compute Gx
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.
    
    V = u[0]
    om = u[1]
    theta = xvec[2] 

    if np.abs(om) < EPSILON_OMEGA:
        Gx = np.array([[1.0, 0.0, -V*np.sin(theta)*dt],
                       [0.0, 1.0, V*np.cos(theta)*dt],
                       [0.0, 0.0, 1.0]])
    else:
        Gx = np.array([[1.0, 0.0, V/om*(np.cos(theta + om*dt ) - np.cos(theta))],
                       [0.0, 1.0, V/om*(np.sin(theta + om*dt ) - np.sin(theta))],
                       [0.0, 0.0, 1.0]])

    ########## Code ends here ##########
    return Gx
    

def compute_Gu(xvec, u, dt):
    """
    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
    Outputs:
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute Gu
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.
    
    V = u[0]
    om = u[1]
    theta = xvec[2] 

    if np.abs(om) < EPSILON_OMEGA:
        Gu = np.array([[np.cos(theta)*dt, -V * pow(dt, 2) * np.sin(theta)],
                       [np.sin(theta)*dt, V * pow(dt, 2) * np.cos(theta)],
                       [0.0, dt]])
    else:
    
        Gu = np.array([[1./om*(np.sin(theta + om*dt ) - np.sin(theta)),-V/(om**2)*(np.sin(theta + om*dt ) - np.cos(theta + om*dt )*dt*om - np.sin(theta))], 
                       [-1./om*(np.cos(theta + om*dt ) - np.cos(theta)),V/(om**2)*(np.cos(theta + om*dt ) + np.sin(theta + om*dt )*dt*om - np.cos(theta))], 
                       [0.0, dt]])

    ########## Code ends here ##########
    return Gu


def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.

    V = u[0]
    om = u[1]
    theta = xvec[2] 
    x = xvec[0]
    y = xvec[1]

    theta_1 = theta + om*dt 
    if np.abs(om) < EPSILON_OMEGA:

        theta_1 = theta + om*dt
        x_1 = x + V*np.cos(theta)*dt
        y_1 = y + V*np.sin(theta)*dt
        
    else:

        theta_1 = theta + om*dt 
        x_1 = x + V/om * (np.sin(theta + om*dt) - np.sin(theta))
        y_1 = y - V/om * (np.cos(theta + om*dt) - np.cos(theta))
   
    g = np.array([x_1, y_1 , theta_1])

    Gx = compute_Gx(xvec, u, dt)
    Gu = compute_Gu(xvec, u, dt)

    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)

    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    theta = x[2]
    x_is = x[0]
    y_is = x[1]
    x_cam, y_cam, theta_cam =  tf_base_to_camera

    p_w = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ np.array([x_cam, y_cam]).T + np.array([x_is, y_is]).T
    xcam_w = p_w[0]
    ycam_w = p_w[1]
    
    thetacam_w = theta_cam + x[2]

    dhdth = (ycam_w-y_is) * np.cos(alpha) - (xcam_w-x_is) * np.sin(alpha)
    
             
    #r_in_cam = pow(x_t**2 + y_t**2, 0.5)
    r_in_cam =  r - (xcam_w * np.cos(alpha) + ycam_w * np.sin(alpha))
    alpha_in_cam = alpha - thetacam_w

    h = [alpha_in_cam, r_in_cam]

    Hx = np.array([[0, 0, -1],
                   [-np.cos(alpha), -np.sin(alpha), dhdth]])




    
    

    
    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
