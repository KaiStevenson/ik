import math
import numpy as np
import matplotlib.pyplot as plt

section_length = 10
max_angle = 90
mouse_down = False

p1 = np.array([0,0]) #base joint
p2 = np.array([10,0]) #2nd joint
p3 = np.array([20,0]) #3rd joint
p4 = np.array([30,0]) #end of arm
a1, a2, a3 = 0, 0, 0

def solve(tPos):
    global p1, p2, p3, p4
    global a1, a2, a3
    i = 0
    while True:
        #TODO: angle limits
        i += 1
        #1
        s1 = get_vector_angle(p3, p4, tPos)
        p4 = rotate_around(p4, p3, s1)
        r1 = get_vector_angle(p3, np.array([p3[0] + 1, p3[1]]), p4)
        nc = np.abs(p4[0] - tPos[0]) + np.abs(p4[1] - tPos[1]) < 1 or i > 10
        #2
        s2 = get_vector_angle(p2,p4,tPos)
        p3 = rotate_around(p3, p2, s2)
        if not nc:
            p4 = point_in_direction(p3, r1 + s2, section_length)
        c3 = get_vector_angle_nr(p1, p2)
        r1 = get_vector_angle(p3, np.array([p3[0] + 1, p3[1]]), p4)
        r2 = get_vector_angle(p2, np.array([p2[0] + 1, p2[1]]), p3)
        r2_r = r2 - c3
        r1_r = r1 - r2_r - c3
        #3
        s3 = get_vector_angle(p1, p4, tPos)
        if not nc:
            p2 = rotate_around(p2, p1, s3)
            p3 = point_in_direction(p2, r2 + s3, section_length)
            p4 = point_in_direction(p3, r1 + r2 + s3, section_length)
        a1, a2, a3 = r1_r, r2_r, c3
        if nc:
            break

    plot(tPos)
        
def plot(dot):
    global p1, p2, p3, p4
    global a1, a2, a3
    global fig, ax
    ax.cla()
    ax.axis([-40,40,-40,40])
    ax.plot([0,p2[0]],[0,p2[1]])
    ax.text(0, 0, str(round(smallest_angle(a3))) + "a3")
    ax.plot([p2[0],p3[0]],[p2[1],p3[1]])
    ax.text(p2[0], p2[1], str(round(smallest_angle(a2))) + "a2")
    ax.plot([p3[0],p4[0]],[p3[1],p4[1]])
    ax.text(p3[0], p3[1], str(round(smallest_angle(a1))) + "a1")
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

def relative_joint_angle(joint, prior_p, next_p):
    return get_vector_angle(joint, next_p, prior_p)

#gets the best angle for one joint
def get_vector_angle(origin, end, target):
    v1 = target - origin
    v2 = end - origin
    v1n = v1 / np.linalg.norm(v1)
    v2n = v2 / np.linalg.norm(v2)
    a1 = math.degrees(math.atan2(v1n[0],v1n[1])) - 90
    a2 = math.degrees(math.atan2(v2n[0],v2n[1])) - 90
    a = a1 - a2
    return a

def get_vector_angle_nr(origin, target):
    v = target - origin
    vn = v / np.linalg.norm(v)
    a = math.degrees(math.atan2(vn[0], vn[1])) - 90
    return a
    
def clamp(minvalue, value, maxvalue):
    return max(minvalue, min(value, maxvalue))

def smallest_angle(angle):
    if angle < 0:
        if angle < -180:
            return angle + 360
        else:
            return angle
    else:
        if angle > 180:
            return angle - 360
        else:
            return angle

def update(event):
    if mouse_down == True:
        solve(np.array([event.xdata,event.ydata]))
def mouse_click(event):
    global mouse_down
    mouse_down = True
    update(event)
def mouse_release(event):
    global mouse_down
    mouse_down = False

fig, ax = plt.subplots()
fig.canvas.mpl_connect('motion_notify_event', update)
fig.canvas.mpl_connect('button_press_event', mouse_click)
fig.canvas.mpl_connect('button_release_event', mouse_release)
plot(np.array([0, 0]))