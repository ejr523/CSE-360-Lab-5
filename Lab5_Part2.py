import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
import copy
import math

positions = {}
rotations = {}

X = 1.8
Y = -0.5

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


if __name__ == "__main__":
    clientAddress = "192.168.0.27"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 211

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    IP_ADDRESS = "192.168.0.209"

    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')

    # goal = np.array([3, 4])
    # goal = np.array([positions[robot_id][0] + distance[0], positions[robot_id][1] + distance[1]])
    # print(rotations[robot_id])


    try:
        time.sleep(5)
        origin = time.time()
        k = 650
        o = 300
        despos = np.array([X, Y]) #np.array([positions[robot_id][0] + goal[0], positions[robot_id][1] + goal[1]])
        dist = np.array([despos[0] - positions[robot_id][0], despos[1] - positions[robot_id][1]]) 
        orientation = (np.arctan(dist[1]/dist[0]) * 180)/np.pi
        if orientation > 180:
            orientation -= 360
        
        timePlot = [time.time() - origin]
        oerr = [orientation - rotations[robot_id]]
        poserr = [dist]

        while is_running:
            if robot_id in positions:
                print('Time:', time.time())
                print('Rotation: ', rotations[robot_id])
                print('Position:', positions)
                print('Desired Position:', despos)

                dist = np.array([despos[0] - positions[robot_id][0], despos[1] - positions[robot_id][1]])   
                print('Distance to Goal:', dist)

                orientation = (np.arctan(dist[1]/dist[0]) * 180)/np.pi
                print('Desired Orientation:', orientation)
                if orientation > 180:
                    orientation -= 360

                angRot = orientation - rotations[robot_id]
                print('angRot: ', angRot)

                v = k * np.linalg.norm(dist)
                print('Velocity:', v)

                omega = o * angRot
                print('Omega: ', omega)

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                print('u values (u[0], u[1])', u[0], u[1])

                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

                # Stop robot once reached correct
                print('error:',math.sqrt(dist[0]**2 + dist[1]**2))
                if math.sqrt(dist[0]**2 + dist[1]**2) <= .15:
                    command = 'CMD_MOTOR#00#00#00#00\n'
                    s.send(command.encode('utf-8'))
                    break
                time.sleep(0.05)
                timePlot.append(time.time() - origin)
                oerr.append(orientation - rotations[robot_id])
                poserr.append(dist)



        epos = dist
        eori = orientation - rotations[robot_id]
        print('Position Error:', epos)
        print('Orientation Error:', eori)
        print('Velocity Error:', v - epos)
        print('Omega Error:', omega - eori)

        timePlot = np.array(timePlot)
        oerr = np.array(oerr)
        poserr = np.array(poserr)

        plt.grid()
        plt.xlabel("Time")
        plt.ylabel("Position Error")
        plt.plot(timePlot[:], poserr[:, 0], label="x(t)")
        plt.plot(timePlot[:], poserr[:, 1], label="y(t)")
        leg = plt.legend(loc='upper center')
        plt.show()

        plt.grid()
        plt.xlabel("Time")
        plt.ylabel("Orientation Error")
        plt.plot(timePlot[:], oerr[:], label="o(t)")
        leg = plt.legend(loc='upper center')
        plt.show()

        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        # Close the connection
        s.shutdown(2)
        s.close()


    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        # Close the connection
        s.shutdown(2)
        s.close()
    