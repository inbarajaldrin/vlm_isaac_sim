from geometry_msgs.msg import PoseStamped, Point, Quaternion

import sys
import casadi as ca
import numpy as np
from pytransform3d.rotations import quaternion_from_euler
import math


def gen_push_cartesian_trajectory(
        xb = 0,
        yb = 0.635,
        zb = 0.395,
        thetab = 1.57,
        xf = 0.09,
        yf = 0.635,
        thetaf = 0,
    ):
    """Generates a Cartesian trajectory"""
    a = 0.01
    b = 0.02 
    # L = 0.05
    T = 5 #1 is smooth
    N =  30

    # # Calculate if final position is on the left or right side of the block's line
    # direction_vector = np.array([xf - xb, yf - yb])
    # block_line_vector = np.array([np.cos(thetab), np.sin(thetab)])
    # cross_product = np.cross(np.append(block_line_vector, 0), np.append(direction_vector, 0))

    # # Determine the initial push position for the end effector
    # if cross_product[2] > 0:  # If the final pose is on the right side
    #     push_angle = thetab + np.pi / 2  # Go left of the block
    # else:
    #     push_angle = thetab - np.pi / 2  # Go right of the block

    x0 = xb #+ (b / 2 + 0.002) * np.cos(push_angle) 
    y0 = yb #+ (b / 2 + 0.002) * np.sin(push_angle)
    z0 = zb #+ a / 2 + 0.001  # The z0 is set as a/2 to be at the center height of the block

    #x0 = x0 - 0.52
    #y0 = y0 + 0.45
    #z0 = z0 + 0.38

    #xf = xf - 0.52
    #yf = yf + 0.45
    #zf = zf + 0.38

    theta0 = thetab

    # Initialize variables for optimization
    dt = T / N
    vxi, vyi, omegai = 0.0, 0.0, 0.0  # Assume initial velocities are zero
    vxf, vyf, omegaf = 0.0, 0.0, 0.0  # Assume final velocities are zero

    # Define quintic polynomial trajectory coefficients
    a_x = ca.MX.sym('a_x', 6)
    a_y = ca.MX.sym('a_y', 6)
    a_theta = ca.MX.sym('a_theta', 6)

    # Time variable
    t = ca.MX.sym('t')

    # Quintic polynomials for x, y, and theta
    x_poly = a_x[0] + a_x[1] * t + a_x[2] * t**2 + a_x[3] * t**3 + a_x[4] * t**4 + a_x[5] * t**5
    y_poly = a_y[0] + a_y[1] * t + a_y[2] * t**2 + a_y[3] * t**3 + a_y[4] * t**4 + a_y[5] * t**5
    theta_poly = a_theta[0] + a_theta[1] * t + a_theta[2] * t**2 + a_theta[3] * t**3 + a_theta[4] * t**4 + a_theta[5] * t**5

    # Derivatives for velocity
    x_poly_dot = ca.jacobian(x_poly, t)
    y_poly_dot = ca.jacobian(y_poly, t)
    theta_poly_dot = ca.jacobian(theta_poly, t)  # Angular velocity (theta_dot)
    
    # Second derivatives for acceleration
    x_poly_ddot = ca.jacobian(x_poly_dot, t)
    y_poly_ddot = ca.jacobian(y_poly_dot, t)
    theta_poly_ddot = ca.jacobian(theta_poly_dot, t)  # Angular acceleration (theta_ddot)

    # Simplified cost function based on the difference
    cost = 0
    for k in range(N):
        t_k = k * dt

        # Substitute polynomial derivatives at current time
        x_dot_k = ca.substitute(x_poly_dot, t, t_k)
        y_dot_k = ca.substitute(y_poly_dot, t, t_k)
        theta_k = ca.substitute(theta_poly, t, t_k)

        # Cost based on the difference between x_poly_dot * sin(theta_poly) and y_poly_dot * cos(theta_poly)
        step_cost = (x_dot_k * ca.sin(theta_k) - y_dot_k * ca.cos(theta_k))**2
        cost += 0.3*step_cost

        # Acceleration cost
        x_ddot_k = ca.substitute(x_poly_ddot, t, t_k)
        y_ddot_k = ca.substitute(y_poly_ddot, t, t_k)
        theta_ddot_k = ca.substitute(theta_poly_ddot, t, t_k)

        # Add acceleration cost (penalty on accele ration)
        acceleration_cost = x_ddot_k**2 + y_ddot_k**2 + theta_ddot_k**2
        cost += 1.3*acceleration_cost

    # Boundary conditions (initial and final positions, velocities, and angular velocity)
    g = []
    g += [ca.substitute(x_poly, t, 0) - x0,
            ca.substitute(y_poly, t, 0) - y0,
            ca.substitute(theta_poly, t, 0) - theta0]
    g += [ca.substitute(x_poly_dot, t, 0) - vxi,
            ca.substitute(y_poly_dot, t, 0) - vyi,
            ca.substitute(theta_poly_dot, t, 0) - omegai]
    g += [ca.substitute(x_poly, t, T) - xf,
            ca.substitute(y_poly, t, T) - yf,
            ca.substitute(theta_poly, t, T) - thetaf]
    g += [ca.substitute(x_poly_dot, t, T) - vxf,
            ca.substitute(y_poly_dot, t, T) - vyf,
            ca.substitute(theta_poly_dot, t, T) - omegaf]

    # Define the NLP problem
    nlp = {'x': ca.vertcat(a_x, a_y, a_theta), 'f': cost, 'g': ca.vertcat(*g)}

    # Create an NLP solver
    opts = {'ipopt.print_level': 0, 'print_time': False}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    # Initial guess for optimization variables
    x0_guess = np.zeros(18)  # Polynomial coefficients (18)

    # Solve the problem
    sol = solver(x0=x0_guess, lbg=0, ubg=0)

    # Extract the solution
    x_opt = sol['x'].full().flatten()

    # Extract the optimal trajectory
    a_x_opt = x_opt[:6]
    a_y_opt = x_opt[6:12]
    a_theta_opt = x_opt[12:18]

    # Time vector and duration list
    time = np.linspace(0, T, N)
    duration_list = [t_k for t_k in time]


    # Generate the trajectory points
    waypoints = []
    for t_k in duration_list:
        x_k = np.polyval(a_x_opt[::-1], t_k)
        y_k = np.polyval(a_y_opt[::-1], t_k)
        theta_k = np.polyval(a_theta_opt[::-1], t_k)

        # Create position and orientation using Vector3 and Quaternion
        position = Point(x=x_k, y=y_k, z=z0)
        # This gives quaternion in WXYZ format. 
        quaternion_val = quaternion_from_euler(e=[-math.pi, 0, theta_k],  # Vector of Euler rots
                                               i=0, j=1, k=2,   # Order of rotations
                                               extrinsic=True)
        orientation = Quaternion(x=quaternion_val[1], y=quaternion_val[2], 
                                 z=quaternion_val[3], w=quaternion_val[0])

        # Create the Pose message 
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.pose.position = position
        goal_pose.pose.orientation = orientation
        # Append to the waypoints
        waypoints.append(goal_pose)
        
        print(x_k,y_k,t_k)
        
    return waypoints


if __name__ =="__main__":
    gen_push_cartesian_trajectory()