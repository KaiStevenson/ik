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
    i = 0
    while True:
        i += 1

        #1
        s1 = get_vector_angle(p3, p4, tPos)
        p4 = rotate_around(p4, p3, s1)
        plot(tPos, p3)
        #2
        s2 = get_vector_angle(p2,p4,tPos)
        p3 = rotate_around(p3, p2, s2)
        p4 = point_in_direction(p3, s1 + s2, section_length)
        plot(tPos, p2)
        #3
        s3 = get_vector_angle(p1, p4, tPos)
        p2 = rotate_around(p2, p1, s3)
        p3 = point_in_direction(p2, s2 + s3, section_length)
        p4 = point_in_direction(p3, s1 + s2 + s3, section_length)
        plot(tPos, p1)
        if np.abs(p4[0] - tPos[0]) + np.abs(p4[1] - tPos[1]) < 1 or i > 10:
            break
        
def plot(dot, o):
    global p1
    global p2
    global p3
    global p4
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event',on_click)  
    ax.axis([-40,40,-40,40])
    ax.plot([0,p2[0]],[0,p2[1]])
    ax.plot([p2[0],p3[0]],[p2[1],p3[1]])
    ax.plot([p3[0],p4[0]],[p3[1],p4[1]])
    ax.plot([o[0],dot[0]],[o[1],dot[1]])
    circle = plt.Circle(dot,0.5,color='r')
    ax.add_patch(circle)
    plt.show()

def point_in_direction(point, angle, length):
    x = (math.cos(math.radians(-angle)) * length) + point[0]
    y = (math.sin(math.radians(-angle)) * length) + point[1]
    return np.array([x,y])

def rotate_around(point, center, angle):
    v = point - center
    p = point_in_direction(center,angle,get_vector_magnitude(v))
    return p

#gets the best angle for one joint
def get_vector_angle(origin, end, target):
    v1 = target - origin
    v2 = end - origin
    l1 = get_vector_magnitude(v1)
    l2 = get_vector_magnitude(v2)
    cos_th = np.dot(v1 / l1,v2 / l2)
    sin_th = np.cross(v1 / l1,v2 / l2)
    return np.rad2deg(np.arctan2(sin_th,cos_th))

def on_click(event):
    solve(np.array([event.xdata,event.ydata]))

def get_vector_magnitude(vector):
    return math.sqrt(vector[0] * vector[0] + vector[1] + vector[1])

solve(np.array([-20,18]))
# print(get_vector_angle(np.array([0,0]),np.array([1,0]),np.array([2,2])))