import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import copy

positions = {}
rotations = {}


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

    # goal = np.array([4, 5])
    # goal = np.array([positions[robot_id][0] + distance[0], positions[robot_id][1] + distance[1]])
    # print(rotations[robot_id])


    try:
        time.sleep(5)
        origin = time.time()
        k = 10000
        o = 20000
        r = 1
        center = np.array([positions[robot_id][0], positions[robot_id][1]])
        prad = math.radians(time.time() - origin)
        x_act = []
        x_des = []
        y_act = []
        y_des = []

        while True:
            # print('\n\n\n\n\n\n\n\n\n\n\n\nNEW ANGLE\n\n\n\n\n\n\n\n\n\n\n')
            if (time.time() - origin) - math.degrees(prad) > 1:
                prad = math.radians(time.time() - origin)
            despos = np.array([(r * np.cos(prad)) + center[0], (r * np.sin(prad)) + center[1]])
            print('Time:', time.time()-origin)
            print('Rotation: ', rotations[robot_id])
            # print('Position:', positions[robot_id])
            print('Desired Position:', despos)

            dist = np.array([despos[0] - positions[robot_id][0], despos[1] - positions[robot_id][1]])
            print('error', math.sqrt(dist[0] ** 2 + dist[1] ** 2))
            # print('Distance to Goal:', dist)

            orientation = np.arctan2(dist[1], dist[0])
            # print('Desired Orientation:', orientation)
            rotationsRad = math.radians(rotations[robot_id])
            angRot = np.arctan2(np.sin(orientation - rotationsRad), np.cos(orientation - rotationsRad))
            # print('angRot: ', angRot)
            d = np.linalg.norm(dist)
            v = k * d
            # print('Velocity:', v)

            omega = o * angRot
            # print('Omega: ', omega)

            u = np.array([v - omega, v + omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500

            # print('u values (u[0], u[1])', u[0], u[1])
            print(u)

            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))

            # Stop robot once reached correct orientation

            # if math.sqrt(dist[0]**2 + dist[1]**2) <= .35:
            #     command = 'CMD_MOTOR#00#00#00#00\n'
            #     s.send(command.encode('utf-8'))
            #     break

            time.sleep(0.1)
            x_act.append(positions[robot_id][0])
            x_des.append(despos[0])
            y_act.append(positions[robot_id][1])
            y_des.append(despos[1])



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
        plt.xlabel("Desired X")
        plt.ylabel("Robot X")
        plt.plot(x_des[:], x_act[:], label="x")
        leg = plt.legend(loc='upper center')
        plt.show()

        plt.grid()
        plt.xlabel("Desired Y")
        plt.ylabel("Robot Y")
        plt.plot(y_des[:], y_act[:], label="y")
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

        plt.grid()
        plt.xlabel("Desired X")
        plt.ylabel("Robot X")
        plt.plot(x_des[:], x_act[:], label="x")
        leg = plt.legend(loc='upper center')
        plt.show()

        plt.grid()
        plt.xlabel("Desired Y")
        plt.ylabel("Robot Y")
        plt.plot(y_des[:], y_act[:], label="y")
        leg = plt.legend(loc='upper center')
        plt.show()

        streaming_client.shutdown()
        # Close the connection
        s.shutdown(2)
        s.close()
    