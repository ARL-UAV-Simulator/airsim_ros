#! /usr/bin/python2

import time
import numpy as np
import math
import sys

import matplotlib.pyplot as plt

import lqr


class DesiredState:
    def __init__(self):
        self.pos = np.zeros((3,1))
        self.vel = np.zeros((3,1))
        self.acc = np.zeros((3,1))
        self.yaw = 0
        self.yawdot = 0



class MinimumSnap:
    def __init__(self, waypoints):
        self.t = time.time()
        self.state = np.zeros((10,1))
        
        d = waypoints[:,1:] - waypoints[:,0:-1]

        self.d0 = 0.75*np.sqrt(d[0,:]*d[0,:] + d[1,:]*d[1,:] + d[2,:]*d[2,:])

        self.traj_time = np.append(0, np.cumsum(self.d0))
        self.waypoints0 = np.copy(waypoints)

        N = np.size(waypoints, 1)-1

        self.p_c = np.zeros((7,8))
        self.p_c[0,:] = np.ones((1,8))

        p = np.poly1d(self.p_c[0,:])

        p_d1 = np.polyder(p)
        self.p_c[1,1:] = np.flip(p_d1.c)

        p_d2 = np.polyder(p_d1)
        self.p_c[2,2:] = np.flip(p_d2.c)

        p_d3 = np.polyder(p_d2)
        self.p_c[3,3:] = np.flip(p_d3.c)

        p_d4 = np.polyder(p_d3)
        self.p_c[4,4:] = np.flip(p_d4.c)

        p_d5 = np.polyder(p_d4)
        self.p_c[5,5:] = np.flip(p_d5.c)

        p_d6 = np.polyder(p_d5)
        self.p_c[6,6:] = np.flip(p_d6.c)

        head_c = np.diag(self.p_c)
        head_c = np.diag(head_c)

        head_c = np.append(head_c, np.zeros((7,1)), 1)


        A = np.zeros((8*N,8*N))
        b = np.zeros((8*N, 3))

        for i in range(1,N+1):

            A[(i-1)*8, np.arange(0,8)+(i-1)*8] = head_c[0,:]
            b[(i-1)*8, :] = waypoints[:,i-1].T

            A[(i-1)*8+1, np.arange(0,8)+(i-1)*8] = self.p_c[0,:]
            b[(i-1)*8+1, :] = waypoints[:,i].T

            if i < N:
                A[(i-1)*8+2, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[1,:], -head_c[1,:])
                A[(i-1)*8+3, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[2,:], -head_c[2,:])
                A[(i-1)*8+4, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[3,:], -head_c[3,:])
                A[(i-1)*8+5, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[4,:], -head_c[4,:])
                A[(i-1)*8+6, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[5,:], -head_c[5,:])
                A[(i-1)*8+7, np.arange(0,16)+(i-1)*8] = np.append(self.p_c[6,:], -head_c[6,:])


        A[8*N-6, np.arange(0,8)] = head_c[1,:]
        A[8*N-5, np.arange(0,8)] = head_c[2,:]
        A[8*N-4, np.arange(0,8)] = head_c[3,:]
        A[8*N-3, np.arange(0,8)+8*(N-1)] = self.p_c[1,:]
        A[8*N-2, np.arange(0,8)+8*(N-1)] = self.p_c[2,:]
        A[8*N-1, np.arange(0,8)+8*(N-1)] = self.p_c[3,:]


        x1 = np.matmul(np.linalg.inv(A), b[:,0])    
        x2 = np.matmul(np.linalg.inv(A), b[:,1])
        x3 = np.matmul(np.linalg.inv(A), b[:,2])
        
        self.alpha = np.zeros((8, N, 3))

        self.alpha[:,:,0] = np.reshape(x1, (8, N), 'F')
        self.alpha[:,:,1] = np.reshape(x2, (8, N), 'F')
        self.alpha[:,:,2] = np.reshape(x3, (8, N), 'F')




    def compute(self, t, state):
        if t > self.traj_time[-1]:
            t = self.traj_time[-1]

        t_index = 0
        for i in range(0, len(self.traj_time)):
            if self.traj_time[i] >= t:
                t_index = i
                break

        if t_index > 0:
            t = t - self.traj_time[t_index-1]


        desired_state = DesiredState()

        if t == 0:
            desired_state.pos = self.waypoints0[:,0]
            desired_state.vel = np.zeros((3,1))
            desired_state.acc = np.zeros((3,1))
        else:
            scale = t/self.d0[t_index-1]

            f_p = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[0,:], 3), (3,8), 'F')
            f_p = np.flip(f_p, 1)
            desired_state.pos = np.array( [[np.polyval(f_p[0,:], scale)],
                                          [np.polyval(f_p[1,:], scale)],
                                          [np.polyval(f_p[2,:], scale)]] )

            f_v = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[1,:],3), (3,8), 'F')
            f_v = np.flip(f_v[:,np.arange(1,8)], 1)
            desired_state.vel = np.array( [[np.polyval(f_v[0,:], scale)],
                                          [np.polyval(f_v[1,:], scale)],
                                          [np.polyval(f_v[2,:], scale)]] ) / self.d0[t_index-1]

            f_a = np.squeeze(self.alpha[:,t_index-1,:]).T * np.reshape(np.repeat(self.p_c[2,:],3), (3,8), 'F')
            f_a = np.flip(f_a[:,np.arange(2,8)], 1)

            desired_state.acc = np.array( [[np.polyval(f_a[0,:], scale)],
                                           [np.polyval(f_a[1,:], scale)],
                                           [np.polyval(f_a[2,:], scale)]] ) / self.d0[t_index-1]**2

        dx = self.waypoints0[0,-1] - state[0,0]
        dy = self.waypoints0[1, -1] - state[1,0]


        _, _, cyaw = lqr.LQR.quat2rpy(state[3:7])

        if dx**2 + dy**2 < 1:
            desired_state.yaw = cyaw # 0
        else:
            desired_state.yaw = cyaw #-math.atan2(dy,dx)

        desired_state.yawdot = 0

        x0 = np.zeros((10,1))
        x0[0:3] = np.array([desired_state.pos]).T
        x0[7:10] = desired_state.vel

        yaw = desired_state.yaw

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        roll = (1/9.8)*(desired_state.acc[0,0]*sy-desired_state.acc[1,0]*cy)
        pitch = (1/9.8)*(desired_state.acc[0,0]*cy + desired_state.acc[1,0]*sy)
        
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        x0[3:7] = np.array([lqr.LQR.rpy2quat(roll, pitch, yaw)]).T


        u0 = np.zeros((4,1))

        omega_r = cr*cp*desired_state.yawdot
        u0[2] = omega_r
        
        R_BA = np.array(  [[ cr*cp-sr*sy*sp, -cr*sy, cy*sp+cp*sr*sy],
                           [ cp*sy+cy*sp*sr, cr*cy, sy*sp-cy*cp*sr],
                           [ -cr*sp, sr, cr*cp]] )

        c = np.matmul(np.linalg.inv(R_BA), np.array([[0,0,desired_state.acc[2,0]+9.8]]).T)
        u0[3] = c[2]

        return x0, u0



if __name__ == "__main__":

    waypoints = np.array(  [[0, 0,  0],
                            [1,    1,   5],
                            [3,    2,   6],
                            [4,   5,  8],
                            [6,    7,   9],
                            [8,    9,   10]]).T

    print(waypoints.shape)

    traj_generator = MinimumSnap(waypoints)


    t = np.linspace(0,50,1000)
    states = np.empty((0, 3))

    state = traj_generator.compute(4.04, None)

    for i in t:
        x0, u0 = traj_generator.compute(i, None)
        states = np.append(states, x0[0:3].T, 0)


    plt.plot(t, states[:,:], "-")
    plt.show()