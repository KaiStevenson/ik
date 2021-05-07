import math
import numpy as np
import matplotlib.pyplot as plt

section_length = 10

p1 = np.array([0,0]) #base joint
p2 = np.array([10,0]) #2nd joint
p3 = np.array([20,0]) #3rd joint
p4 = np.array([30,0]) #end of arm

def solve(tPos):
    global p1
    global p2
    global p3
    global p4
    s1,s2,s3,i = 0,0,0,0
    while True:
        i += 1
        #1
        s1 = get_vector_angle(p3, p4, tPos)
        p4 = rotate_around(p4, p3, s1)
        r1 = get_vector_angle(p3, np.array([p3[0] + 1, p3[1]]), p4)
        if np.abs(p4[0] - tPos[0]) + np.abs(p4[1] - tPos[1]) < 1 or i > 10:
            break
        #2
        s2 = get_vector_angle(p2,p4,tPos)
        p3 = rotate_around(p3, p2, s2)
        p4 = point_in_direction(p3, r1 + s2, section_length)
        r1 = get_vector_angle(p3, np.array([p3[0] + 1, p3[1]]), p4)
        r2 = get_vector_angle(p2, np.array([p2[0] + 1, p2[1]]), p3)
        #3
        s3 = get_vector_angle(p1, p4, tPos)
        p2 = rotate_around(p2, p1, s3)
        p3 = point_in_direction(p2, r2 + s3, section_length)
        p4 = point_in_direction(p3, r1 + r2 + s3, section_length)
    plot(tPos)
        
def plot(dot):
    global p1
    global p2
    global p3
    global p4
    plt.close()
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event',on_click)  
    ax.axis([-40,40,-40,40])
    ax.plot([0,p2[0]],[0,p2[1]])
    ax.plot([p2[0],p3[0]],[p2[1],p3[1]])
    ax.plot([p3[0],p4[0]],[p3[1],p4[1]])
    circle = plt.Circle(dot,0.5,color='r')
    ax.add_patch(circle)
    plt.show()

def point_in_direction(point, angle, length):
    x = (math.cos(math.radians(-angle)) * length) + point[0]
    y = (math.sin(math.radians(-angle)) * length) + point[1]
    return np.array([x,y])

def rotate_around(point, center, angle):
    a = math.radians(-angle)
    qx = center[0] + math.cos(a) * (point[0] - center[0]) - math.sin(a) * (point[1] - center[1])
    qy = center[1] + math.sin(a) * (point[0] - center[0]) + math.cos(a) * (point[1] - center[1])
    return np.array([qx, qy])

#gets the best angle for one joint
def get_vector_angle(origin, end, target):
    if np.abs(end[0] - target[0]) + np.abs(end[1] - target[1]) < 1:
        return 0
    v1 = target - origin
    v2 = end - origin
    # l1 = np.linalg.norm(v1)
    # l2 = np.linalg.norm(v2)
    # cos_th = np.dot(v1, v2)
    # sin_th = np.cross(v1, v2)
    # return math.degrees(math.atan2(sin_th, cos_th))
    v1n = v1 / np.linalg.norm(v1)
    v2n = v2 / np.linalg.norm(v2)
    a1 = math.degrees(math.atan2(v1n[0],v1n[1])) - 90
    if a1 < 0:
        a1 += 360
    a2 = math.degrees(math.atan2(v2n[0],v2n[1])) - 90
    if a2 < 0:
        a2 += 360
    a = a1 - a2
    if a < 0:
        a += 360
    return a

def on_click(event):
    solve(np.array([event.xdata,event.ydata]))

plot(np.array([0,0]))
# print(get_vector_angle(np.array([0,0]),np.array([1,0]),np.array([1,5])))