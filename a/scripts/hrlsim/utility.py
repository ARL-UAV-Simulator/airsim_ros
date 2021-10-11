import numpy as np
import math
from typing import Tuple, List
from hrlsim.airsim import Quaternionr

import os, json


def xyz2ned(x: np.ndarray) -> np.ndarray:
    """
    Convert X,Y,Z to North, East, Down

    Args:
        x (np.ndarray): North, East, Down

    Returns:
        np.ndarray: X, Y, Z
    """

    assert np.shape(x) == (10,1), "The state must be a 10x1 vector"

    xnew = np.zeros((10,1))
    xnew[0,0] =  x[1,0]
    xnew[1,0] =  x[0,0]
    xnew[2,0] = -x[2,0]

    q1 = Quaternionr(x[4,0], x[5,0], x[6,0], x[3,0])
    q_rot = Quaternionr(math.sqrt(2)/2, -math.sqrt(2)/2, 0, 0)

    qnew = (q1.rotate(q_rot)).to_numpy_array()
    xnew[4:7,0] = qnew[0:3]
    xnew[3,0] = qnew[3]
    
    xnew[7,0] =  x[8,0]
    xnew[8,0] =  x[7,0]
    xnew[9,0] = -x[9,0]

    return xnew


def ned2xyz(x: np.ndarray) -> np.ndarray:
    """
    Convert North, East, Down to X, Y, Z

    Args:
        x (np.ndarray): North, East, Down

    Returns:
        np.ndarray: X, Y, Z
    """
    assert np.shape(x) == (10, 1), "The state must be a 10x1 vector"
    xnew = np.zeros((10, 1))

    xnew[0,0] =  x[1,0]
    xnew[1,0] =  x[0,0]
    xnew[2,0] = -x[2,0]

    q1 = Quaternionr(x[4,0], x[5,0], x[6,0], x[3,0])
    q_rot = Quaternionr(math.sqrt(2) / 2, math.sqrt(2) / 2, 0, 0)

    qnew = (q1.rotate(q_rot)).to_numpy_array()

    xnew[3,0] = qnew[3]
    xnew[4:7,0] = qnew[0:3]

    xnew[7,0] =  x[8,0]
    xnew[8,0] =  x[7,0]
    xnew[9,0] = -x[9,0]

    return xnew

def cmd2comp(u: np.ndarray) -> Tuple[float,float,float,float]:
    """
    Gets the command components from an vector

    Args:
        u (np.ndarray): The command (body rollrate, body pitchrate, body yawrate, thrust)

    Returns:
        Tuple[float,float,float,float]: (body rollrate, body pitchrate, body yawrate, thrust)
    """
    assert np.shape(u) == (4, 1), "The command must be a 4x1 vector"
    return u[0,0], u[1,0], u[2,0], u[3,0]

def comp2cmd(wp: float, wq: float, wr: float, c: float) -> np.ndarray:
    """
    Gets the command vector from an components

    Args:
        wp (float): body roll rate
        wq (float): body pitch rate
        wr (float): body yaw rate
        c  (float): normalized collective thrust in body z

    Returns:
        u (np.ndarray): 4x1 command vector [body rollrate, body pitchrate, body yawrate, thrust].T
    """
    return np.array([[wp,wq,wr,c]]).T   

def state2comp(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Gets the state from an vector

    Args:
        x (np.ndarray): The state (position, orientation, velocity)

    Returns:
        Tuple[np.ndarray, np.ndarray, np.ndarray]: Tuple representing body rates
    """
    assert np.shape(x) == (10, 1), "The command must be a 10x1 vector"
    return x[0:3], x[3:7], x[7:10]

def comp2state(p: np.ndarray, q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Gets the state vector from a components

    Args:
        p (np.ndarray): 3D position
        q (np.ndarray): 4D quaternion -> assumes is from airsim.Quaternion3r.to_numpy_array()*
        v (np.ndarray): 3D velocity

    *ie, [qx,qy,qz,qw]
    Returns:
        x (np.ndarray): 10x1 state vector [position, orientation, velocity].T
    """
    assert(p.shape == (3,1) or p.shape == (3,), "Position vector must be (3x1) or(3,)")
    assert(q.shape == (4,1) or q.shape == (4,), "Orientation vector must be (4x1) or (4,)")
    assert(v.shape == (3,1) or v.shape == (3,), "Velocity vector must be (3x1) or(3,)")

    if p.shape == (3,):
        p = np.array([p]).T
    if q.shape == (4,):
        q = np.array([[q[3],q[0],q[1],q[2]]]).T
    if v.shape == (3,):
        v = np.array([v]).T

    return np.concatenate((p,q,v), 0)


def getDroneListFromSettings(settingsFilePath: str = None) -> List[str]:
    """
    Loads the list of drones from the airsim setting file

    Args:
        settingsFilePath (str, optional): Path to airsim settings file. Defaults to None.

    Returns:
        List[str]: List of vehicles in file
    """
    if settingsFilePath == None:
        HOME = os.getenv("HOME")
        settings_path = HOME + "/Documents/AirSim/settings.json"
    else:
        settings_path = settingsFilePath

    try:
        settings_file = open(settings_path, "r")
    except Exception:
        print("Error opening settings file. Exiting")
        exit()

    settings = json.loads(settings_file.read())
    settings_file.close()

    vehicle_list = list()
    for i in settings["Vehicles"]:
        vehicle_list.append(i)

    return vehicle_list