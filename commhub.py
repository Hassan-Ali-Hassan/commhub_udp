from threading import Thread, Lock
import socket
import struct
import time
import numpy as np
import sys
import math
from os import urandom
from collections import defaultdict

MSG_SIZE = 500


def wrap_angle(angle):
    return (( angle + np.pi) % (2 * np.pi ) - np.pi)

class Packet:
    '''
    PRIVATE
    Create a Packet to be sent over a socket
    :params x, y, z: absolute coordinates of sender
    :param sender_id: comm_id of sender
    :param msgs: list of bytes objects. Each bytes object is fed directly to the buzz script using feed_buzz_message
    '''
    def __init__(self, x, y, z, sender_id, msgs=[],theta =0,received_time=0,addr = ('0.0.0.0', 4242)):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.comm_id = sender_id
        self.msgs = msgs
        self.received_time = received_time
        self.addr=addr

    def set_rb(self,rng,bearing,elevation):
        self.x=rng
        self.y=bearing
        self.z=elevation

    '''
    PRIVATE
    Convert packet to a bytes object containing all the information.

    Contents:
        2 bytes comm_id
        4 bytes x
        4 bytes y
        4 bytes z
        4 bytes theta
        for each message {
            2 bytes message length (n)
            n bytes message
        }
        4 bytes (0000)

    :return b_string: bytes object representing entire packet
    '''
    def byte_string(self):
        b_string = struct.pack('=H4f', int(self.comm_id), float(self.x), float(self.y), float(self.z), float(self.theta))
        for msg in self.msgs:
            b_string += struct.pack('H', len(msg))
            b_string += msg
        b_string += struct.pack('I', 0)
        while len(b_string)<MSG_SIZE:
            b_string += struct.pack('B',0)
        return b_string

        
    '''	
    PRIVATE
    Create a packet from a Kh4 socket
    Block until a string of bytes comes in, and unpack these bytes into a new Packet object.
    The incomming bytes are in the form described in the documentation for Packet.byte_string
    :param s: socket object
    :return: Packet object or False if any socket.error occured
    '''
    @staticmethod
    def from_socket(s):
        addr = ('0.0.0.0', 4242)
        try:
            try:
                m , addr = s.recvfrom(MSG_SIZE)
            except:
                return False
                
            if len(m) == 0:
                # The socket is broken
                return False
                     
            # Process the message
            sender_id, x, y, z, theta = struct.unpack_from('=H4f', m)

            # print('Pos ({},{},{}) angle {} of {}'.format(x,y,z,theta,sender_id) )
            tot = struct.calcsize('=H4f')
            msgs = []
            while (tot < MSG_SIZE):
                try:
                    msg_size = struct.unpack_from('H', m, tot)[0]
                    tot +=2
                except struct.error(e):
                    print(e)
                    return False
                
                if msg_size == 0:
                    break
                msgs.append(m[tot:tot+msg_size])

                tot +=msg_size
                #print('rcv msg from {} size {} tot {}'.format(sender_id, msg_size, tot))
            return Packet(x, y, z, sender_id, msgs,received_time=time.time(),addr=addr)
        except:
            return False


class CommHub:
    '''
    Communication Hub
    Facilitate communication between the robots, as well as update their absolute positions
    :param clients: list. Destination IPs served by this CommHub
    :param forward_freq: float. Frequency of automatic calls to CommHub.forward_packets in Hertz
        Set forward_freq=0 for maximum frequency
        If left as None, CommHub.forward_packets must be called manually
    :param neighbor_distance: float. The range for communication between robots. Distance units must
        be consistent with the units used for CommHub.update_position
    :param host: string. The host of the CommHub. HOST default is "localhost"
    :param port: int. The port of the CommHub. PORT default is 8000
    '''
    def __init__(self, forward_freq=None, neighbor_distance=1.7, host='', port=4242):
        self.alive = True
        self.locs = {}  # comm_id : np.array()
        self.neighbor_distance = neighbor_distance
        self.packets = defaultdict(list)
        self.packets_lock = Lock()
        self.id2ip = {}
        
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.s.bind((host, port))
        except OSError as e:
            print("ERROR: Trying to create a CommHub on a busy address")
            raise e
        

        if forward_freq is not None:
            if forward_freq:
                period = 1/forward_freq
            else:
                period = 0

            t = Thread(target=self.auto_forward, args=(period,), name="Auto Forwarder")
            t.start()

        self.t = Thread(target=self.receive, name="Receiver")
        self.t.start()


    '''
    PRIVATE
    New thread that blocks until new packet comes. Packets are added to self.packets 
    '''
    def receive(self):
        print("Receiving...")  # Debug
        while self.alive:
            p = Packet.from_socket(self.s)
            # Update IP/id database
            self.id2ip[p.comm_id] = p.addr

            if p:
                # print("Received packet from Robot {}".format(p.comm_id))  # Debug
                self.packets_lock.acquire()
                self.packets[p.comm_id].append(p)
       	        self.packets_lock.release()	
            else:
                break

 
    '''
    PRIVATE
    Send packets to a destination
    :param destination: the id of the robot to send the packages to
    :param packets: a list of Packet objects, or just a single Packet
    '''
    def send_to(self, destination, packets,sender):
        try:
            packets[0]
        except (AttributeError, TypeError):
            self.s.sendto(packets.byte_string(),self.id2ip[destination])
            # print(" comm packet sender {} receiver {}".format(sender,destination))
            return
        # m = bytes()
        for p in packets:
            m = p.byte_string()
            self.s.sendto(m,self.id2ip[destination])

    '''
    PRIVATE
    Send packets to a destination
    :param destination: the id of the robot to send the packages to
    :param packets: a list of Packet objects, or just a single Packet
    '''
    def send_to_with_rb(self, destination, packets,rel_rb):
        try:
            packets[0]
        except (AttributeError, TypeError):
            packets.set_rb(rel_rb[0],rel_rb[1], rel_rb[2])
            self.s.sendto(packets.byte_string(),self.id2ip[destination])
            return
        # m = bytes()
        for p in packets:
            p.set_rb(rel_rb[0],rel_rb[1], rel_rb[2])
            m = p.byte_string()
            # print("Time take to send {} sender {} receiver {}".format(time.time()-p.received_time,sender,receiver))
            self.s.sendto(m,self.id2ip[destination])

    '''
    PRIVATE
    Automatically call CommHub.forward_packets at a certain frequency
    :param period: float. Time between calls to CommHub.forward_packets in seconds
    '''
    def auto_forward(self, period):
        print("Forwarding...")
        while self.alive:
            start_time = time.time()
            self.forward_packets()
            # print("Duration of Comm forward {}".format(time.time()-start_time))
            time.sleep(period)


    '''
    Keep the communication flowing between robots.
    All information shared between robots, and any updates to positions are not sent unless this function is called.
    '''
    def forward_packets(self):
        # print("Forwarding messages len of clients {}".format(len(self.clients)))

        # For all known robots, get addresses and ids
        for i1, a1 in self.id2ip.items():

            # If there are packets from these robots, put them into a data structure
            self.packets_lock.acquire()
            tmppackets = self.packets[i1][:]
            self.packets[i1] = []
            self.packets_lock.release()
            if len(tmppackets) == 0:
                tmppackets = [Packet(0.0, 0.0, 0.0, i1)]

            # Cycle through all other robots and forward the packets 
            for i2 , a2 in self.id2ip.items():
                try:

                    # Compute relative vector and distance 
                    rel_vector = self.locs[i1][:-1] - self.locs[i2][:-1]
                    distance = np.linalg.norm(rel_vector)    

                    # Send updated own location to the robot
                    if i1 == i2:
                        self.send_to(i2, Packet(self.locs[i1][0], self.locs[i1][1], self.locs[i1][2], i1,theta=self.locs[i1][3]),i1)
                        
                    # Only forward packets if within comms distance, in RAB format    
                    else: #if distance < self.neighbor_distance:
                        # Compute azimuth (theta) and elevation (phi)
                        rel_theta = np.arctan2(rel_vector[1], rel_vector[0])
                        rel_phi = np.arctan2(rel_vector[2], np.linalg.norm(rel_vector[:2]))

                        # print('R {} Raw rel bearing {}'.format(i2, rel_theta))
                        rel_theta = rel_theta - self.locs[i2][3] # convert angle to receivers coordinate
                        # Wrap angles
                        rel_theta = rel_theta + 2*np.pi if rel_theta < 0. else rel_theta
                        rel_phi = rel_phi + 2*np.pi if rel_phi < 0. else rel_phi

                        #print("R {} S {} after bearing {}".format(i2,i1,bearing))
                        self.send_to_with_rb(i2, tmppackets,np.array((distance*100.0,rel_theta,rel_phi)))#*100.0 to obtain [cm] on board
                except KeyError as e:
                    pass #print("No locs for Robot {}".format(i2))


    '''
    Update the position of the specified robot.
    :param robot_id: int. the id of the robot whose position is to be updated
    :param loc: list, tuple, or numpy.array. The updated position of the robot
    :param yaw float yaw euiler angle
    :return: bool. True if update was successful
    '''
    def update_position(self, robot_id, loc, yaw):
        self.locs[int(robot_id)] = np.append(np.array(loc),yaw)
        print("Robot {} pos {}".format(robot_id, self.locs[robot_id]))
        return True
   

    '''
    Determine if the CommHub is still alive
    '''
    def is_alive(self):
        return self.alive

    '''
    Destroy the Communication Hub, and all its clients
    Called when CommHub.forward_packets encounters an error
    '''
    def destroy(self):
        # if self.alive:  # Must check here because might have been closed automatically in CommHub.auto_forward
        self.alive = False
        self.s.sendto(urandom(MSG_SIZE),('127.0.0.1',4242))
        self.s.close()


