import numpy as np
import math
from typing import List, Tuple
import control

from . import controller

import hrlsim
import hrlsim.airsim
import hrlsim.utility

class LQR(controller.Controller):
    """
    Class to compute the LQR controls.
    """

    def __init__(self, trajType: hrlsim.traj.Trajectory, maxThrust: float, mass: float) -> None:
        """
        Initilize controller

        Args:
            traj_generator (hrlsim.traj.Trajectory): Trajectory generator use to compute states
            maxThrust (float) Maximal possible thrust in body z direction
            mas
        """
        super().__init__(trajType, maxThrust, mass)

        #self.Qa = np.diag([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0,5.0,5.0,10.0])
        self.Qa = np.diag([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0,0.1,0.1,0.1])
        self.R = np.diag([1.0, 1.0, 2.0e1, 1.0])
        self.C = np.block([[np.eye(3.0), np.zeros((3,7))]])

        self.Kstar = np.zeros((4, 10), dtype=np.float64)
        self.Ki = np.zeros((4,3), dtype=np.float64)
        self.Ktilde = np.zeros((4,3), dtype=np.float64)

        self.xi = np.zeros((3,1), dtype=np.float64)

        self.mass = mass
        self.max_thrust = maxThrust

        self.linearized_rotation = hrlsim.airsim.to_quaternion(0.0,0.0,0.0)
        self.first = True

        self.prev_gain_time = -999

    def setGoals(self, waypoints: np.ndarray, ic: np.ndarray, fc: np.ndarray, avg_spd: float) -> None:
        """
        Sets the waypoints, initial conditions, and final conditions for the controller.

        Args:
            waypoints (np.ndarray): Waypoints to set
            ic (np.ndarray): Initial conditions [velocity, acceleration, jerk]
            fc (np.ndarray): Final conditions [velocity, acceleration, jerk]
            avg_spd (float): Average speed over trajectory
        """

        tmp = np.copy(waypoints[0,:])
        waypoints[0,:] = waypoints[1,:]
        waypoints[1,:] = tmp
        waypoints[2,:] = -waypoints[2,:]

        tmp = np.copy(ic[:,0])
        ic[:,0] = ic[:,1]
        ic[:,1] = tmp
        ic[:,2] = -ic[:,2]

        tmp = np.copy(fc[:,0])
        fc[:,0] = fc[:,1]
        fc[:,1] = tmp
        fc[:,2] = -fc[:,2]

        self.traj_generator.generate(waypoints, ic, fc, avg_spd)
        self.xi = np.zeros((3,1), dtype=np.float64)

    def computeControl(
        self, t: float, t0: float, dt: float, state: hrlsim.airsim.MultirotorState, drone_name
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the control for a given state

        Args:
            t (float): the time
            state (hrlsim.airsim.MultirotorState): the state of the drone
            body_z_accel (int): previous acceleration

        Returns:
            Tuple[np.ndarray, np.ndarray, np.matrix]: Tuple representing
        """
        p = state.kinematics_estimated.linear_velocity.to_numpy_array()
        q = state.kinematics_estimated.orientation.to_numpy_array()
        v = state.kinematics_estimated.linear_velocity.to_numpy_array()

        body_z_accel = -state.kinematics_estimated.linear_acceleration.z_val

        x = hrlsim.utility.ned2xyz(hrlsim.utility.comp2state(p,q,v))

        x0, u0 = self.traj_generator.compute(t-t0, x)

        # Augment system with integral of (x-x0)
        self.xi += (x[0:3]-x0[0:3])*dt
        X = np.concatenate((x,self.xi), 0)

        rotation = (state.kinematics_estimated.orientation*self.linearized_rotation.inverse()).to_numpy_array()

        elapsed_time = t - self.prev_gain_time
        if 2*math.atan2(np.linalg.norm(rotation[0:3]), rotation[3]) > np.deg2rad(5) or self.first or elapsed_time > 3: 
            #rospy.loginfo(str(drone_name) + " linearized: " + str(t - self.prev_gain_time))

            self.prev_gain_time = t

            self.updateGains(
                X, state.kinematics_estimated.angular_velocity.to_numpy_array(), body_z_accel
            )

            self.linearized_rotation = state.kinematics_estimated.orientation
            self.first = False


        # Compute the optimal control step
        u = u0 - np.matmul(self.Kstar, (x - x0)) - np.matmul((self.Ktilde + self.Ki), X[10:13])

        tmp = u[0,0]
        u[0,0] = u[1,0]
        u[1,0] = tmp
        u[2,0] = u[2,0]

        u[3,0] = abs(u[3,0]) * self.mass / self.max_thrust

        x0 = hrlsim.utility.xyz2ned(x0)
        return x0, u





    def set_costs(self, Q: List[int] = None, R: List[int] = None) -> None:
        """
        Sets the control matrices Q and R 

        Args:
            Q (List[int], optional): List of ints to set for Q. Must be length 10 Defaults to None.
            R (List[int], optional): List of ints to set for R. Must be length 4 Defaults to None.
        """
        if Q != None:
            assert len(Q) == 10, "Q must be a list of length 10"
            self.Qa = np.diag(Q)

        if R != None:
            assert len(R) == 4, "R must be a list of length 4"
            self.R = np.diag(R)

    def updateGains(self, X: np.ndarray, omega: hrlsim.airsim.Vector3r, body_z_accel: float) -> None:
        """
        Updates the gains for the controller

        Args:
            X (np.ndarray): State, must be 10x1 state vector
            rpydot (hrlsim.airsim.Vector3r): Angular Vector 
            body_z_accel (int): Previous Acceleration
        """
        pitch, roll, _ = hrlsim.airsim.to_eularian_angles(hrlsim.airsim.Quaternionr(X[4], X[5], X[6], X[3]))
        c = (9.8-body_z_accel)/(math.cos(roll)*math.cos(pitch))

        u = hrlsim.utility.comp2cmd(omega[0], omega[1], omega[2], c)
        
        for i in range(0, 4):
            if abs(u[i]) < 0.001:
                u[i] = 0.001

        A, B = self.computeLinearization(X[0:10], u)
        Aa = np.block([[A,      np.zeros((10,3))],
                       [self.C, np.zeros((3,3))]])
        Ba = np.block([[B],
                       [np.zeros((3,4))]])

        if np.linalg.matrix_rank(control.ctrb(Aa,Ba)) >= len(X):             
            Ka, _, _ = control.lqr(Aa, Ba, self.Qa, self.R)
            self.Kstar = Ka[:,0:10]
            self.Ki = Ka[:,10:13]

    def computeLinearization(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Updates A maxtrix from state and command lists

        Args:
            x (np.ndarray): The state list
            u (np.ndarray): The command list

        Returns:
            np.ndarray: A matrix
            np.ndarray: B matrix
        """
        # position, orientation, velocity
        (_,q,_) = hrlsim.utility.state2comp(x)

        # body roll rate, body pitch rate, body yaw rate, thrust
        (wp,wq,wr, c) = hrlsim.utility.cmd2comp(u)


        ### COMPUTE PARTIAL DERIVATIVES ###
        #

        # Compute qdotq
        Q = np.array([[0.0, -wp, -wq, -wr],
                      [wp,   0.0, wr, -wq],
                      [wq,   -wr, 0.0, wp],
                      [wr,   wq, -wp, 0.0]])

        qnorm = np.linalg.norm(q)
        qdotq =  0.5 * np.matmul(Q, (np.identity(4) - (math.pow(qnorm,-2)*q*q.T))/qnorm)


        # Compute vdotq
        Q = np.array([[ q[2],  q[3],  q[0], q[1]],
                      [-q[1], -q[0],  q[3], q[2]],
                      [ q[0], -q[1], -q[2], q[3]]])

        qnorm = np.linalg.norm(q)
        vdotq = (2.0*c*np.matmul(Q, (np.identity(4) - (math.pow(qnorm,-2)*q*q.T))/qnorm))


        # Compute qdotomega
        qdotomega = 0.5*np.array([[-q[1], -q[2], -q[3]],
                                  [ q[0], -q[3], -q[2]],
                                  [ q[3],  q[0],  q[1]],
                                  [-q[2],  q[1],  q[0]]])


        # Compute vdotc
        vdotc = np.array([[q[0]*q[2] + q[1]*q[3]],
                          [q[2]*q[3] - q[0]*q[1]],
                          [q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]]])


        #### USE PARTIALS TO COMPUTE LINEARIZATION ###
        #

        # 10x10 block matrix
        A = np.block([[np.zeros((3, 3)), np.zeros((3, 4)), np.identity(3)],
                      [np.zeros((4, 3)), qdotq,            np.zeros((4,3))],
                      [np.zeros((3, 3)), vdotq,            np.zeros((3,3))]])

        # 10x4 block matrix
        B = np.block([[np.zeros((3, 3)), np.zeros((3, 1))],
                      [qdotomega,        np.zeros((4, 1))],
                      [np.zeros((3, 3)), vdotc]])

        return A, B