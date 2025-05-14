"""
 Force Pulse Test
    To run the Whisker Settle Time Test on one of the whisker models:
    1. Go to the 'controller(model, data)' function
    2. uncomment the line of code for the corresponding model.
"""


import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt
import math
import ctypes



xml_path = 'All_Whiskers.xml' #xml file (assumes this is in the same folder as this file)

simend = 20 #simulation time
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

force_sweep = []

angle_data0 = []
angle_data1 = []

f_data_seg = []

t_data_seg = []



def init_controller(model,data):
    mj.mj_step(model, data)
    
    #initialize the controller here. This function is called once, in the beginning
    pass

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

    #site_id = model.site("siteend").id  
    #site_pos = data.site_xpos[site_id]  # site position array
    #print("Position of siteend:", site_pos)

def deflection_return_time(segment, delta, force, settle_time):
                
        applied_force=3
        if(data.time>2 and data.time<=5):
           data.xfrc_applied[segment, :3] = [0,applied_force,0]
           if(data.time % 0.01 < model.opt.timestep) :
                delta.append(data.geom_xpos[segment][1].copy())
                #print(data.geom_xpos[segment][1])
                force.append(applied_force)
        else:
            
            data.xfrc_applied[segment, :3] = [0,0,0] 
            if(data.time<=simend and data.time % 0.01 < model.opt.timestep) :
               delta.append(data.geom_xpos[segment][1].copy())
               force.append(0)
               #return data.time
               #print(data.time)
            elif(data.time>=5+settle_time):
                delta = np.array(delta).T.tolist()
                force = np.array(force).T.tolist()
                data.time = simend
                #print("***",delta)
                #return 5+settle_time
            
mj.mj_step(model, data)
print_world_details()

delta = []
force = []

delta1 = []
force1 = []

delta2 = []
force2 = []

end_time = 0.0

def controller(model, data):
    global delta
    global force
    global end_time

    """
    To run the Whisker Settle Time Test on one of the whisker models
    uncomment the line of code for the corresponding model.
    """
    #deflection_return_time(32, delta1, force,3)#segment aluminium
    deflection_return_time(52, delta1, force,3) # Composite Nylon6/6
    #deflection_return_time(72, delta1, force,3) # Composite Aluminium
    #deflection_return_time(16, delta1, force,3) # Segment nylon

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


fig, ax1 = plt.subplots()

# Plot on ax1 (left y-axis)
#ax1.plot(np.linspace(0, 8 , num=len(delta)), delta, color="blue", marker='o', label='Composite Aluminium')
ax1.plot(np.linspace(0, 8 , num=len(force)), force , color="blue", marker='o', label='Force')
ax1.set_ylabel('Force(N)', color='b')
ax1.set_xlabel('Time(s)', color='b')
ax1.tick_params(axis='y', labelcolor='b')

# Create a second y-axis on the rightS
ax2 = ax1.twinx()
ax2.plot(np.linspace(0, 8, num=len(delta1)), delta1,color="red", marker='.', label='Deflection')
ax2.set_ylabel('Deflection (DeltaY)(m)', color='r')
ax2.tick_params(axis='y', labelcolor='r')
print(len(force))

lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='center left')

ax1.grid(True)
plt.title("Force Pulse Response")
plt.show()
