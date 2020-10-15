# this code will calculate the value of angles of joint for a position, and give a series of set of angles to move the endeffector to scan the given rectangle.
# please integrate this code with dorna to move th arm at the provided angle (mentioned below)
# if you new to the dorna arm, please go through these links --> https://youtu.be/kDtoZDKv0PM


from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import numpy as np

# declaring the link length of the dorna arm, see the diagram. 
# please see the exact dimensions from dorna from it's brochure, I found brochure of dorna2 only and used that dimension, it may be few mm larger than dorna. if you find that brochure, the please update the values. 
# if you are using dorna2 then no need to change
a1 = 206.4
a2 = 95.48
a3 = 203.2
a4 = 152.4
a5 = 48.92


################################################################################################################
################################################################################################################
# forward kinematics is to verify the output and plot the graph
# i used it to check my output and simulation. if you don't need it, you can remove this entire section.

def forward_kin(j0, j1, j2, j3, j4):
    X = []
    Y = []
    Z = []

    # DH parameter table of the given robotic arm see figure, for mare details on this please follow this --> https://youtu.be/4WRhVqQaZTE
    DH = np.array([[j0        , np.pi / 2, a2, a1],
                   [j1        , 0        , a3, 0 ],
                   [j2        , 0        , a4, 0 ],
                   [j3+np.pi/2, np.pi / 2, 0 , 0 ],
                   [j4        , 0        , 0 , a5]])

    # homogeneous matrices
    H0_0 = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    H0_1 = np.matrix([[np.cos(DH[0, 0]), -np.sin(DH[0, 0]) * np.cos(DH[0, 1]), np.sin(DH[0, 0]) * np.sin(DH[0, 1]), DH[0, 2] * np.cos(DH[0, 0])],
                   [np.sin(DH[0, 0]), np.cos(DH[0, 0]) * np.cos(DH[0, 1]), -np.cos(DH[0, 0]) * np.sin(DH[0, 1]), DH[0, 2] * np.sin(DH[0, 0])],
                   [0, np.sin(DH[0, 1]), np.cos(DH[0, 1]), DH[0, 3]],
                   [0, 0, 0, 1]])

    H1_2 = np.matrix([[np.cos(DH[1, 0]), -np.sin(DH[1, 0]) * np.cos(DH[1, 1]), np.sin(DH[1, 0]) * np.sin(DH[1, 1]), DH[1, 2] * np.cos(DH[1, 0])],
                   [np.sin(DH[1, 0]), np.cos(DH[1, 0]) * np.cos(DH[1, 1]), -np.cos(DH[1, 0]) * np.sin(DH[1, 1]), DH[1, 2] * np.sin(DH[1, 0])],
                   [0, np.sin(DH[1, 1]), np.cos(DH[1, 1]), DH[1, 3]],
                   [0, 0, 0, 1]])

    H0_2 = np.dot(H0_1, H1_2)

    H2_3 = np.matrix([[np.cos(DH[2, 0]), -np.sin(DH[2, 0]) * np.cos(DH[2, 1]), np.sin(DH[2, 0]) * np.sin(DH[2, 1]), DH[2, 2] * np.cos(DH[2, 0])],
                   [np.sin(DH[2, 0]), np.cos(DH[2, 0]) * np.cos(DH[2, 1]), -np.cos(DH[2, 0]) * np.sin(DH[2, 1]), DH[2, 2] * np.sin(DH[2, 0])],
                   [0, np.sin(DH[2, 1]), np.cos(DH[2, 1]), DH[2, 3]],
                   [0, 0, 0, 1]])

    H0_3 = np.dot(H0_2, H2_3)

    H3_4 = np.matrix([[np.cos(DH[3, 0]), -np.sin(DH[3, 0]) * np.cos(DH[3, 1]), np.sin(DH[3, 0]) * np.sin(DH[3, 1]), DH[3, 2] * np.cos(DH[3, 0])],
                   [np.sin(DH[3, 0]), np.cos(DH[3, 0]) * np.cos(DH[3, 1]), -np.cos(DH[3, 0]) * np.sin(DH[3, 1]), DH[3, 2] * np.sin(DH[3, 0])],
                   [0, np.sin(DH[3, 1]), np.cos(DH[3, 1]), DH[3, 3]],
                   [0, 0, 0, 1]])

    H0_4 = np.dot(H0_3, H3_4)

    H4_5 = np.matrix([[np.cos(DH[4, 0]), -np.sin(DH[4, 0]) * np.cos(DH[4, 1]), np.sin(DH[4, 0]) * np.sin(DH[4, 1]), DH[4, 2] * np.cos(DH[4, 0])],
                   [np.sin(DH[4, 0]), np.cos(DH[4, 0]) * np.cos(DH[4, 1]), -np.cos(DH[4, 0]) * np.sin(DH[4, 1]), DH[4, 2] * np.sin(DH[4, 0])],
                   [0, np.sin(DH[4, 1]), np.cos(DH[4, 1]), DH[4, 3]],
                   [0, 0, 0, 1]])

    H0_5 = np.dot(H0_4, H4_5)

    # print("R0_6 comes out to be: ")
    # print(np.matrix(H0_6[:3, :3]))

    X.append(0)
    X.append(0)
    X.append(H0_1[0, 3])
    X.append(H0_2[0, 3])
    X.append(H0_3[0, 3])
    X.append(H0_4[0, 3])
    X.append(H0_5[0, 3])

    Y.append(0)
    Y.append(0)
    Y.append(H0_1[1, 3])
    Y.append(H0_2[1, 3])
    Y.append(H0_3[1, 3])
    Y.append(H0_4[1, 3])
    Y.append(H0_5[1, 3])

    Z.append(0)
    Z.append(a1)
    Z.append(H0_1[2, 3])
    Z.append(H0_2[2, 3])
    Z.append(H0_3[2, 3])
    Z.append(H0_4[2, 3])
    Z.append(H0_5[2, 3])

    # center of all the frames in ground frame
    X = np.reshape(X, (1, 7))
    Y = np.reshape(Y, (1, 7))
    Z = np.reshape(Z, (1, 7))

    return X, Y, Z


def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    return fig, ax


def update_plot(X, Y, Z, X1, Y1, Z1, fig, ax):
    X = np.reshape(X, (1, 7))
    Y = np.reshape(Y, (1, 7))
    Z = np.reshape(Z, (1, 7))
    ax.cla()
    ax.plot_wireframe(X, Y, Z)
    ax.plot_wireframe(X1, Y1, Z1, color = 'r')
    plt.draw()
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(.00000001)


################################################################################################################
################################################################################################################



################################################################################################################
################################################################################################################


def get_cosine_law_angle(a, b, c):
    # given all sides of a triangle a, b, c
    # calculate angle gamma between sides a and b using cosine law
    gamma = np.arccos((a*a + b*b - c*c) / (2*a*b))

    return gamma


"""
    get_inverse_kinematics_angles(px, py, pz, pitch)
        input --> takes input of position of end effector and value of pitch (-pi/2 rad to point end effector downward ).
        functionality --> performs inverse kinematics and calculate joint angles to reach at the give point
        return --> returns the value of first 4 angles in order to move end effector to the given pos.

        see the given sheets of inverse kin calculation for details.
"""
def get_inverse_kinematics_angles(px, py, pz, pitch):
    j0 = np.arctan2(py, px)
    
    Xc = px - a5*np.cos(pitch)*np.cos(j0)
    Yc = py - a5*np.cos(pitch)*np.sin(j0)
    Zc = pz - a5*np.sin(pitch)

    r1 = np.sqrt(Xc**2 + Yc**2)
    r2 = np.sqrt((r1-a2)**2 + (Zc-a1)**2)

    phi1 = np.arctan2((Zc-a1),(r1-a2))
    phi2 = get_cosine_law_angle(a3, r2, a4)
    j1 = phi1 + phi2

    phi3 = get_cosine_law_angle(a3, a4, r2)
    j2 = phi3 - np.pi

    j3 = pitch - j1 - j2

    return j0, j1, j2, j3


################################################################################################################
################################################################################################################



################################################################################################################
################################################################################################################


"""
    move_end_to(px, py, pz, pitch, roll)
        input --> position, pitch, and desired roll
        functionality --> calls the inv. kin. function to calculate angles and pass the values to dorna arm to run
        return none
"""
def move_end_to(px, py, pz, pitch, roll):

    j0, j1, j2, j3 = get_inverse_kinematics_angles(px, py, pz, pitch)
    j4 = roll
    # cmd = {"command": "move", "prm":{"path": "joint", "movement": 0, "j0":j0, "j1":j1, "j2":j2, "j3":j3, "j4":j4}}
    # robot.play(cmd)


    print("j0 : ", j0)
    print("j1 : ", j1)
    print("j2 : ", j2)
    print("j3 : ", j3)
    print("j4 : ", j4)

    ##########this is just to plot the output#################
    X, Y, Z = forward_kin(j0, j1, j2, j3, j4)
    print("X : ", X[0, 6])
    print("Y : ", Y[0, 6])
    print("Z : ", Z[0, 6])
    fig, ax = create_plot()
    update_plot(X, Y, Z, fig, ax)
    plt.show()
    ##########################################################


"""
    scan_rectangle(A, B, C, D, height, pitch)
        input --> four corners of the rectangle in order, the height from where you are scanning, and the pitch (-pi/2 rad. to point end effector downward ).
        functionality --> uses grid method to scan the rectangle. you can put you own logic according to situation to scan the rectangle.
        return none
"""
def scan_rectangle(A, B, C, D, height, pitch):
    # start is the point of end effector, and starting from A
    startx = A[0]
    starty = A[1]

    ##########this is just to plot the output#################
    fig, ax = create_plot()
    X = []; Y = []; Z = []; X1 = []; Y1 = []; Z1 = []
    i=0
    ##########################################################

    while startx <= D[0]:
        
        while starty < B[1]:
            px, py, pz, pitch = startx, starty, height, pitch
            j0, j1, j2, j3 = get_inverse_kinematics_angles(px, py, pz, pitch)
            j4 = -j0
            # cmd = {"command": "move", "prm":{"path": "joint", "movement": 0, "j0":j0, "j1":j1, "j2":j2, "j3":j3, "j4":j4}}
            # robot.play(cmd)
            starty +=10    # you can increase this value to go smother but slow
            time.sleep(.00010)  # decrease the delay to increase the speed.

            ##########this is just to plot the output#################
            X, Y, Z = forward_kin(j0, j1, j2, j3, j4)
            X1.append(X[0, 6]); Y1.append(Y[0, 6]); Z1.append(Z[0, 6])
            X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
            X11 = np.reshape(X1, (1, i + 1))
            Y11 = np.reshape(Y1, (1, i + 1))
            Z11 = np.reshape(Z1, (1, i + 1))
            update_plot(X, Y, Z, X11, Y11, Z11, fig, ax)
            i+=1
            ##########################################################
        
        startx += 10    # you can increase this value, to scan more dense.

        while starty > A[1]:
            px, py, pz, pitch = startx, starty, height, pitch
            j0, j1, j2, j3 = get_inverse_kinematics_angles(px, py, pz, pitch)
            j4 = -j0
            # cmd = {"command": "move", "prm":{"path": "joint", "movement": 0, "j0":j0, "j1":j1, "j2":j2, "j3":j3, "j4":j4}}
            # robot.play(cmd)
            starty -=10    # you can increase this value to go smother but slow
            time.sleep(.00010)  # decrease the delay to increase the speed.

            ##########this is just to plot the output#################
            X, Y, Z = forward_kin(j0, j1, j2, j3, j4)
            X1.append(X[0, 6]); Y1.append(Y[0, 6]); Z1.append(Z[0, 6])
            X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
            X11 = np.reshape(X1, (1, i + 1))
            Y11 = np.reshape(Y1, (1, i + 1))
            Z11 = np.reshape(Z1, (1, i + 1))
            update_plot(X, Y, Z, X11, Y11, Z11, fig, ax)
            i+=1
            ##########################################################

        startx += 10   # you can increase this value, to scan more dense.

    plt.show()


################################################################################################################
################################################################################################################



def main():

    # move_end_to(250, 250, 100, -np.pi/2, 0)
    scan_rectangle([150, -100], [150, 150], [400, 150], [400, -100], 100, -np.pi/2)


if __name__ == "__main__":
    main()

