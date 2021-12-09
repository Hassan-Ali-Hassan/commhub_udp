#!/usr/bin/python3

import time
from threading import Lock

from commhub import *

from OptiTrackPython import NatNetClient
from OptiTrackPython import from_quaternion2rpy

# Parameters for the Buzz ComHub
FORWARD_FREQ = 50  # Hz
PORT = 8002
PORT = 24580
# PORT = 1510
#M_IP = '192.168.2.104'
#S_IP = '192.168.2.103'

M_IP = '192.168.0.113'
S_IP = '192.168.0.100'
multicast_address = "239.255.42.99"

GO_TO_DURATION = 0.05

# For the single marker and rigid body detection
SINGLE_MARKER = False
MIN_DISTANCE = 0.04

# rigidbody_names2track = KH4_CLIENTS
rigidbody_names2track = {"1", "2", "3"}


lock_opti = Lock()


def extract_ip():
    st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st.connect(('10.255.255.255', 1))
        ip = st.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        st.close()
    return ip


def receiveRigidBodyFrame(timestamp, id, position, rotation, rigidBodyDescriptor):
    if rigidBodyDescriptor:
        for rbname in rigidbody_names2track:
            if rbname in rigidBodyDescriptor:
                if id == rigidBodyDescriptor[rbname][0]:
                    # skips this message if still locked
                    if lock_opti.acquire(False):
                        try:
                            # rotation is a quaternion!
                            r, p, y = from_quaternion2rpy(rotation)

                            # just sending the name of the robot, its xy position and its yaw angle
                            comm_hub.update_position(int(rbname), position, y)

                        finally:
                            lock_opti.release()


if __name__ == '__main__':

    # This will create a new NatNet client
    # nnc = NatNetClient(client_ip=M_IP)
    nnc = NatNetClient(M_IP, S_IP, multicast_address)
    # Configure the streaming client to call the rigid body handler or the markers handler
    nnc.rigidBodyListener = receiveRigidBodyFrame
    nnc.run()

    comm_hub = CommHub(forward_freq=FORWARD_FREQ)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
