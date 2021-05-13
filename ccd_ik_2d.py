import math
import numpy as np
import matplotlib.pyplot as plt
#basic parameters
section_length = 10
max_angle = 90
mouse_down = False
#initial joint positions
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
        i += 1
        #find the correct angle for the end effector joint (3)
        a3_current_absolute = get_vector_angle_nr(p3,p4) #current joint rotation, absolute
        a3_current_relative = get_vector_angle(p2,p4,p3) #current joint rotation, relative
        a3_rotation_desire = get_vector_angle(p3,p4,tPos) #relative rotation change needed for 3rd joint
        p4 = rotate_around(p4,p3,a3_rotation_desire) #rotate end effector about joint
        #check if we're close enough
        nc = np.abs(p4[0] - tPos[0]) + np.abs(p4[1] - tPos[1]) < 1 or i > 10
        if not nc:
            #find the correct angle for the second joint (2)
            a2_current_absolute = get_vector_angle_nr(p2,p3) #current joint rotation, absolute
            a2_current_relative = get_vector_angle(p1,p3,p2) #current joint rotation, relative
            a2_rotation_desire = get_vector_angle(p2,p4,tPos) #relative rotation change needed for 2nd joint
            p3 = rotate_around(p3,p2,a2_rotation_desire) #rotate about joint...
            p4 = rotate_around(p4,p2,a2_rotation_desire) #along with everything connected
            #find the correct angle for the base joint (1)
            a1_current = get_vector_angle_nr(p1,p2) #current joint rotation, absolute (& relative)
            a1_rotation_desire = get_vector_angle(p1,p4,tPos) #relative rotation change needed for 1st joint
            p2 = rotate_around(p2,p1,a1_rotation_desire)
            p3 = rotate_around(p3,p1,a1_rotation_desire)
            p4 = rotate_around(p4,p1,a1_rotation_desire)
        else:
            plot(tPos)
            break  
        
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