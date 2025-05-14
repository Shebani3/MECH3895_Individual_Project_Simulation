"""
Note: To end the simulation, close the simulation window.
To change from straight wall to cylinderical pillar, uncomment line 346 in Whisker_strain_based.xml
"""


import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt
import math
import ctypes



xml_path = 'Whisker_strain_based.xml' #xml file (assumes this is in the same folder as this file)

simend = 15 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)    

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

f_data =[]
T_data = []
Tendon_length = []
Tendon_force = []
time = []


def init_controller(model,data):
    data.sensordata[:] = 0 
    mj.mj_step(model, data)
    data.ctrl[:] = 0
    
    #initialize the controller here. This function is called once, in the beginning
    pass

def slide_test(Whisker,speed,Tendon_Len,Tendon_frc,f_data,T_data,time):

    data.ctrl[Whisker]=speed
    
    if(data.time<=simend and data.time % 0.05 < model.opt.timestep) :
            match Whisker:
                case 0:
            
                    Tendon_Len.append(data.sensordata[6:8].copy()) 
                    Tendon_frc.append(data.sensordata[8:10].copy()) 
                    time.append(data.time)
                #print("sensorrrrr1: ",data.sensordata[6:9])
                case 1:
                    Tendon_Len.append(data.sensordata[10:12].copy()) 
                    T_data.append(data.sensordata[12:14].copy())
                    time.append(data.time) 
                case 2:
                    f_data.append(data.sensordata[0:3].copy()) 
                    T_data.append(data.sensordata[3:6].copy())
                    time.append(data.time)
                #print("sensorrrrr: ",data.sensordata[0:3])



def print_world_details():

    for i in range(model.ngeom):
        geom_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, i)
        geom_pos = data.geom_xpos[i]  # Get global position
        print(f"Geom ID: {i}, Geom Name: {geom_name}, Position: {geom_pos}")

    for i in range(model.nbody):  # model.nbody gives the total number of bodies
        body_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)  # Get the name of the body
        parent_id = model.body_parentid[i]  # Get the parent ID of the body
        world_position = data.xpos[i]  # Get the position of the body in the world
        inertial_mass = model.body_mass[i]  # Get the inertial mass of the body

        # Print the information
        print(f"Body ID: {i}, Name: {body_name}, Parent ID: {parent_id}, Position: {world_position}, Mass: {inertial_mass}")

    for i in range(model.nsensor):  # model.nsensor gives the total number of sensors
        sensor_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_SENSOR, i)  # Get the name of the sensor
        sensor_type = model.sensor_type[i]  # Get the type of the sensor
        sensor_data_index = model.sensor_adr[i]  # Index in the sensordata array
        sensor_dim = model.sensor_dim[i]  # Dimensionality of the sensor's data
        sensor_data = data.sensordata[sensor_data_index : sensor_data_index + sensor_dim]  # Sensor values

        # Print sensor information and readings
        print(f"Sensor ID: {i}, Name: {sensor_name}, Type: {sensor_type}, Data: {sensor_data}")

        actuator_names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
        for i, name in enumerate(actuator_names):
            print(f"Actuator {i}: {name}, Control Value: {data.ctrl[i]}")

def plot_force_vs_time(Whisker, time,x,y,z = None):
   if Whisker == 2:
    plt.subplot(1, 3, 1)  # (nrows, ncols, index)
    plt.plot(time, x[0], label="x", color="blue", marker='o')
    plt.plot(time, x[1], label="y", color="red", marker='o')
    plt.plot(time, x[2], label="Z", color="orange", marker='o')
    
    plt.title("Force vs Time (Segmented Nylon6/6 Whisker)")
#
    plt.xlabel("time")
    plt.tight_layout()
    plt.grid()
    plt.legend()
    

    plt.subplot(1, 3, 2)
    plt.plot(time, y[1], label="fy", marker='o')
    plt.plot(time, y[0], label="fx", marker='o')
    plt.plot(time, y[2], label="fz", marker='o')
    #plt.ylim(-5, 10)  # Set Y-axis range
    plt.title("Torque vs time")
    plt.xlabel("time")
    plt.legend()

    if z is not None:
        plt.subplot(1, 3, 3)
        plt.plot(time, z, label="ay", marker='o')
        #plt.ylim(-5, 10)  # Set Y-axis range
        plt.title("angle vs time")
        plt.xlabel("time")
        plt.legend()

    plt.show()
   if Whisker != 4:
    plt.subplot(1, 3, 1)  # (nrows, ncols, index)
    plt.plot(time, x[:,0], label="Tendon len Left", color="blue", marker='o')
    plt.plot(time, x[:,1], label="Tendon len Right", color="red", marker='o')
    if Whisker == 0:
        plt.title("Tendon_Len vs Time (Composite Nylon Whisker)")
    else:
        plt.title("Tendon_Len vs Time (Segmented Nylon Wisker)")  
    plt.xlabel("time")
    plt.tight_layout()
    plt.grid()
    plt.legend()
    

    plt.subplot(1, 3, 2)
    plt.plot(time, y[:,1], label="fy", marker='o')
    plt.plot(time, y[:,0], label="fx", marker='o')
    
    #plt.ylim(-5, 10)  # Set Y-axis range
    plt.title("Tendon_Force vs time")
    plt.xlabel("time")
    plt.legend()

    if z is not None:
        plt.subplot(1, 3, 3)
        plt.plot(time, z, label="ay", marker='o')
        #plt.ylim(-5, 10)  # Set Y-axis range
        plt.title("angle vs time")
        plt.xlabel("time")
        plt.legend()

    plt.show()


def collision_detected(whisker,force_threshold):
    if (data.time<=0.004):
          data.sensordata[:] = 0 
    else:
        if(whisker):
                    if(abs(data.sensordata[6:9][1])>force_threshold):
                            print("thershold excedd1: ",data.sensordata[7])
                            return True
                    else:
                        return False
    
print_world_details()

def set_radial_speed(Vlx,Vly,theta):
     data.ctrl[3] = (Vlx*(np.cos(theta))) - (Vly*(np.sin(theta)))
     data.ctrl[1] = (Vlx*(np.sin(theta)))+(Vly*(np.cos(theta)))
     data.ctrl[5] = theta
     

# Robot states
MODE_FREE = 0
MODE_WALL_FOLLOW = 1
state = MODE_FREE  # Start in free navigation mode
SPEED = 3

# Define control gains
Kp_v = 0.5  # Gain for velocity control
Kp_r = 0.01  # Gain for rotation control

# Define target force
force_target = 0.7  # Desired force on whisker
Whisker = 0
sensor_map = {
    "composite_nylon_strain_based": 0,
    "segmented_nylon": 2
}

def controller(model, data):
    global Whisker
    global sensor_map

    sensor_number = sensor_map["composite_nylon_strain_based"]
    
    slide_test(sensor_number,10,Tendon_length,Tendon_force,f_data,T_data,time)

pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
            # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options


# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)
# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])

#show the world frame coordinates
opt.frame = mj.mjtFrame.mjFRAME_WORLD

#initialize the controller
init_controller(model,data)


#set the controller
mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()



f_data = np.vstack(f_data).T
T_data = np.vstack(T_data).T

plt.subplot(1, 3, 1)  # (nrows, ncols, index)
plt.plot(time, f_data[0], label="x", color="blue", marker='o')
plt.plot(time, f_data[1], label="y", color="red", marker='o')
plt.plot(time, f_data[2], label="Z", color="orange", marker='o')
    
plt.title("Force vs Time (Segmented Nylon6/6 Whisker)")
#
plt.xlabel("time")
plt.tight_layout()
plt.grid()
plt.legend()
    
plt.subplot(1, 3, 2)
plt.plot(time, T_data[1], label="fy", marker='o')
plt.plot(time, T_data[0], label="fx", marker='o')
plt.plot(time, T_data[2], label="fz", marker='o')
    #plt.ylim(-5, 10)  # Set Y-axis range
plt.title("Torque vs time")
plt.xlabel("time")
plt.legend()

plt.show()



