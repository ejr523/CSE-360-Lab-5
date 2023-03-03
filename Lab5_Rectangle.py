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
    despos = [[-3.5, 3], [-3.5, -2], [5, -2], [5, 3]]
    despos = np.array(despos)
    # print(despos.shape)
    # goal = np.array([positions[robot_id][0] + distance[0], positions[robot_id][1] + distance[1]])
    # print(rotations[robot_id])


    try:
        time.sleep(5)
        origin = time.time()
        k = 1000
        o = 600
        i = 0
        x_act = []
        x_des = []
        y_act = []
        y_des = []
        while i < 4:
            # despos = np.array([positions[robot_id][0] + goal[0], positions[robot_id][1] + goal[1]])
            des = despos[i]
            dist = np.array([des[0] - positions[robot_id][0], des[1] - positions[robot_id][1]])
            orientation = (np.arctan(dist[1]/dist[0]) * 180)/np.pi
            if orientation > 180:
                orientation -= 360

            while is_running:
                if robot_id in positions:
                    print('Time:', time.time())
                    print('Rotation: ', rotations[robot_id])
                    print('Position:', positions[robot_id])
                    print('Desired Position:', despos[i])

                    dist = np.array([des[0] - positions[robot_id][0], des[1] - positions[robot_id][1]])
                    print('Distance to Goal:', dist)

                    orientation = np.arctan2(dist[1], dist[0])
                    # print('Desired Orientation:', orientation)
                    rotationsRad = math.radians(rotations[robot_id])
                    angRot = np.arctan2(np.sin(orientation - rotationsRad), np.cos(orientation - rotationsRad))
                    # print('angRot: ', angRot)
                    d = np.linalg.norm(dist)
                    v = k * d
                    # print('Velocity:', v)

                    omega = o * math.degrees(angRot)
                    # print('Omega: ', omega)

                    u = np.array([v - omega, v + omega])
                    u[u > 1500] = 1500
                    u[u < -1500] = -1500
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                    s.send(command.encode('utf-8'))

                    # Stop robot once reached correct orientation
                    if math.sqrt(dist[0]**2 + dist[1]**2) <= .2:
                        command = 'CMD_MOTOR#00#00#00#00\n'
                        s.send(command.encode('utf-8'))
                        break
                    time.sleep(0.05)
                    x_act.append(positions[robot_id][0])
                    x_des.append(des[0])
                    y_act.append(positions[robot_id][1])
                    y_des.append(des[1])
            i += 1

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
        streaming_client.shutdown()
        # Close the connection
        s.shutdown(2)
        s.close()
    