"""
    1. To select a different whisker model in line "sensor_number = sensor_map["segmented_nylon"]" instead of "segmented_nylon"
    enter the string of a different model from the sensor_map dictionary. Line 391
    2. To run a reverse sweep uncomment the "force_distance_sweep_live_reverse" function. Line 394
"""

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt
import math
import ctypes


xml_path = 'All_Whiskers.xml'
simend = 50 #simulation time
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

def force_distance_sweep(segment_num_start, segment_num_end, force):
    next_tic =0
    deflection = []
    f_data_cont = []
    t_data_cont = [] 
    sensor_select = data.sensordata[0:2].copy()
    
    for i in range(segment_num_start,segment_num_end+1): #use reversed() to sweep backwards
                    
                    match segment_num_start:
                        case 2:
                            sensor_select = data.sensordata[0:3].copy() #segmented nylon
                        case 19:
                            sensor_select = data.sensordata[6:9].copy() #segmented aluminium
                        case 36:
                            sensor_select = data.sensordata[12:15].copy() #composite nylon
                        case 57:
                            sensor_select = data.sensordata[18:21].copy() #composite aluminium
    
        
                    print(i)
                    mj.mj_step(model, data)
                    data.xfrc_applied[i, :3] = [0,force,0]
                    data.xfrc_applied[i-1, :3] = [0,0,0] # when using reveresed change i-1 to i+1

                    for i in range(7500):
                        mj.mj_step(model, data)
                    
                    deflection.append(data.geom_xpos[segment_num_end].copy()) 


                    f_data_cont.append(sensor_select) 
                    #t_data_cont.append(data.sensordata[9:12].copy()) 
                    print(data.sensordata, data.time)
                    #force = force+1

    deflection = np.array(deflection).T.tolist()
    f_data_cont = np.array(f_data_cont).T.tolist()
    #t_data_cont = np.array(t_data_cont).T.tolist()
    print(f_data_cont)
    return deflection, f_data_cont, t_data_cont

count_seg = 0

def force_distance_sweep_live(segment_num_start, deflection, force, torque):
        global last_logged_second
        global count
        global count_seg
        global Whisker_len

        applied_force=3
        selection = 0
        Whisker_len=np.linspace(0, 0.211, num=16)
        segment_num_geom= 16
        match segment_num_start:
            case 2:
                sensor_select = data.sensordata[0:3].copy() #segmented nylon
                sensor_select_t = data.sensordata[3:6].copy()
                segment_num_end = 17
                segment_num_geom= 16
                selection = 0
                Whisker_len=np.linspace(0, 0.211, num=15)
            case 19:
                sensor_select = data.sensordata[6:9].copy() #segmented aluminium
                sensor_select_t = data.sensordata[9:12].copy()
                segment_num_end = 34
                segment_num_geom= 32
                selection = 0
                Whisker_len=np.linspace(0, 0.211, num=16)
            case 36:
                sensor_select = data.sensordata[12:15].copy() #composite nylon
                sensor_select_t = data.sensordata[15:18].copy()
                segment_num_end = 55
                segment_num_geom= 52
                selection = 1
                Whisker_len=np.linspace(0, 0.2015, num=20)
            case 57:
                sensor_select = data.sensordata[18:21].copy() #composite aluminium
                sensor_select_t = data.sensordata[21:24].copy()
                segment_num_end = 76
                segment_num_geom= 72
                selection = 1
                Whisker_len=np.linspace(0, 0.2015, num=20)
        
        current_second = int(data.time)

        if current_second != last_logged_second:
            last_logged_second = current_second
            count += 1
            if (count >2):
                if(count == 4):
                    print(count, " jjj " ,data.time)
                    data.xfrc_applied[segment_num_start, :3] = [0,applied_force,0]
                    data.xfrc_applied[segment_num_start-1, :3] = [0,0,0]
                    if(selection):
                         deflection.append(data.geom_xpos[segment_num_geom].copy()) 
                    else:
                          deflection.append(data.geom_xpos[segment_num_geom].copy())
                    
                    torque.append(sensor_select_t) 
                    force.append(sensor_select) 
                elif(count >4 and count%1 ==0):
                     count_seg +=1
                     segment_num_start = segment_num_start+(count_seg)
                     data.xfrc_applied[segment_num_start, :3] = [0,applied_force,0]
                     data.xfrc_applied[segment_num_start-1, :3] = [0,0,0]

                     if(selection):
                         #deflection.append(data.xpos[segment_num_end].copy()) 
                         deflection.append(data.geom_xpos[segment_num_geom].copy()) 
                     else:
                        deflection.append(data.geom_xpos[segment_num_geom].copy())

                      
                     force.append(sensor_select) 
                     torque.append(sensor_select_t) 
                     print(count, "  " ,data.time, " ", segment_num_start)
                     if(segment_num_start == segment_num_end):
                       data.time = simend
                     
        return deflection, force,torque

def force_distance_sweep_live_reverse(segment_num_start, deflection, force, torque):
        global last_logged_second
        global count
        global count_seg
        global Whisker_len

        applied_force=3
        selection = 0
        Whisker_len=np.linspace(0, 0.211, num=16)
        segment_num_geom= 16
        match segment_num_start:
            case 2:
                sensor_select = data.sensordata[0:3].copy() #segmented nylon
                sensor_select_t = data.sensordata[3:6].copy()
                segment_num_end = 17
                segment_num_geom= 16
                selection = 0
                Whisker_len=np.linspace(0, 0.211, num=15)
            case 19:
                sensor_select = data.sensordata[6:9].copy() #segmented aluminium
                sensor_select_t = data.sensordata[9:12].copy()
                segment_num_end = 34
                segment_num_geom= 32
                selection = 0
                Whisker_len=np.linspace(0, 0.211, num=16)
            case 36:
                sensor_select = data.sensordata[12:15].copy() #composite nylon
                sensor_select_t = data.sensordata[15:18].copy()
                segment_num_end = 55
                segment_num_geom= 52
                selection = 1
                Whisker_len=np.linspace(0, 0.2015, num=20)
            case 57:
                sensor_select = data.sensordata[18:21].copy() #composite aluminium
                sensor_select_t = data.sensordata[21:24].copy()
                segment_num_end = 76
                segment_num_geom= 72
                selection = 1
                Whisker_len=np.linspace(0, 0.2015, num=20)
        
        current_second = int(data.time)

        if current_second != last_logged_second:
            last_logged_second = current_second
            count += 1
            if (count >2):
                if(count == 4):
                    print(count, " jjj " ,data.time)
                    data.xfrc_applied[segment_num_end, :3] = [0,applied_force,0]
                    data.xfrc_applied[segment_num_end+1, :3] = [0,0,0]
                    if(selection):
                         deflection.append(data.geom_xpos[segment_num_geom].copy()) 
                    else:
                          deflection.append(data.geom_xpos[segment_num_geom].copy())
                    
                    torque.append(sensor_select_t) 
                    force.append(sensor_select) 
                elif(count >4 and count%1 ==0):
                     count_seg +=1
                     segment_num_end = segment_num_end-(count_seg)
                     data.xfrc_applied[segment_num_end, :3] = [0,applied_force,0]
                     data.xfrc_applied[segment_num_end+1, :3] = [0,0,0]

                     if(selection):
                         #deflection.append(data.xpos[segment_num_end].copy()) 
                         deflection.append(data.geom_xpos[segment_num_geom].copy()) 
                     else:
                        deflection.append(data.geom_xpos[segment_num_geom].copy())

                      
                     force.append(sensor_select) 
                     torque.append(sensor_select_t) 
                     print(count, "  " ,data.time, " ", segment_num_start)
                     if(segment_num_start == segment_num_end):
                       data.time = simend
                     
                     
        #deflection = np.array(deflection).T.tolist()
        #force = np.array(force).T.tolist()
    #t_data_cont = np.array(t_data_cont).T.tolist()
        #print(force)
        return deflection, force,torque


def print_world_details():

    for i in range(model.ngeom):
        geom_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, i)
        geom_pos = data.geom_xpos[i]  # Get global position
        print(f"Geom ID: {i}, Geom Name: {geom_name}, Position: {geom_pos}")
        print(f"Geom {i} size: {model.geom_size[i]}")

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

    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "conti_whis_base_site_Nylon6/6")
    site_pos = data.site_xpos[site_id]
    print(f"Site 'conti_whis_base_site_Nylon6/6' position: {site_pos}")
    
    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "conti_whis_base_site_Al")
    site_pos_Al = data.site_xpos[site_id]
    print(f"Site 'conti_whis_base_site_Al' position: {site_pos_Al}")
    
    print(sensor_data)


def plot_force_vs_angle(x,y,t, segment_numbers ):
   
   #time_data = np.arange(0, len(angle_data0) * 0.1, 0.1)
    # Data for subplots
    # First subplot

   plt.subplot(1, 3, 1)  # (nrows, ncols, index)
   plt.plot(segment_numbers, x[0], label="x", color="blue", marker='o')
   plt.plot(segment_numbers, x[1], label="y", color="red", marker='o')
   print(len(segment_numbers), len(x[0]))
   plt.title("displacement of last segment ")
   plt.xlabel("Position on whisker")
   plt.tight_layout()
   plt.grid()
   plt.legend()
   

   plt.subplot(1, 3, 2)
   plt.plot(segment_numbers, y[1], label="fy", marker='o')
   plt.plot(segment_numbers, y[0], label="fx", marker='o')
   plt.plot(segment_numbers, y[2], label="fz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
   plt.title("Force")
   plt.xlabel("Position on whisker")
   plt.legend()

   plt.subplot(1, 3, 3)
   plt.plot(segment_numbers, t[1], label="Ty", marker='o')
   plt.plot(segment_numbers, t[0], label="Tx", marker='o')
   plt.plot(segment_numbers, t[2], label="Tz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
   plt.title("Torque")
   plt.xlabel("Position on whisker")
   plt.legend()
   
   plt.show()


   
    # Data for subplots
    # First subplot
   plt.subplot(1, 3, 1)  # (nrows, ncols, index)
   plt.plot(whisker_len, x[0], label="x", color="blue", marker='o')
   plt.plot(whisker_len, x[1], label="y", color="red", marker='o')
   
   print(len(whisker_len), len(x[0]))
   plt.title("displacement of last segment ")
   plt.xlabel("Position on whisker")
   plt.tight_layout()
   plt.grid()
   plt.legend()
   
   plt.subplot(1, 3, 2)
   plt.plot(whisker_len, y[0], label="tl", marker='o')
   plt.plot(whisker_len, y[1], label="tr", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
   plt.title("Force sweep:tendon_frc")
   plt.xlabel("Position on whisker")
   plt.legend()

   plt.subplot(1, 3, 3)
   plt.plot(whisker_len, z[0], label="tl", marker='o')
   plt.plot(whisker_len, z[1], label="tr", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
   plt.title("Force sweep:tendonpos_sensor")
   plt.xlabel("Position on whisker")
   plt.legend()
   plt.show()

   plt.show()

deviation = []
force = []
Whisker_len =[]
last_logged_second = -1  # put this outside the simulation loop
count = 0
torque = []

sensor_map = {
    "segmented_nylon": 2,
    "segmented_aluminium": 19,
    "composite_nylon": 36,
    "composite_aluminium": 57
}

def controller(model, data):
    global deviation
    global force
    global Whisker_len
    global torque
    global sensor_map
    """
    1. To select a different whisker model in line "sensor_number = sensor_map["segmented_nylon"]" instead of "segmented_nylon"
    enter the string of a different model from the sensor_map dictionary.
    2. To run a reverse sweep uncomment the "force_distance_sweep_live_reverse" function.
    """
 
    sensor_number = sensor_map["composite_aluminium"]
    
    force_distance_sweep_live(sensor_number, deviation, force,torque)
    #force_distance_sweep_live_reverse(sensor_number, deviation,force,torque)

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

print_world_details()


#whisker_len=np.linspace(0, 0.2015, num=20)
#whisker_len=np.linspace(0, 0.211, num=16)


deviation = np.array(deviation)
force = np.array(force)
torque = np.array(torque)

if(len(deviation)>17):
    segment = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
else:
    segment = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]

plt.subplot(1, 3, 1)  # (nrows, ncols, index)
plt.plot(segment, deviation[:,0], label="x", color="blue", marker='o')
plt.plot(segment, deviation[:,1], label="y", color="red", marker='o')
plt.title("displacement of last segment ")
plt.xlabel("Segment on whisker")
plt.tight_layout()
plt.grid()
plt.legend()


plt.subplot(1, 3, 2)
plt.plot(segment, force[:,1], label="fy", marker='o')
plt.plot(segment, force[:,0], label="fx", marker='o')
plt.plot(segment, force[:,2], label="fz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
plt.title("Force")
plt.xlabel("Segment on whisker")
plt.legend()

plt.subplot(1, 3, 3)
plt.plot(segment, torque[:,1], label="Ty", marker='o')
plt.plot(segment, torque[:,0], label="Tx", marker='o')
plt.plot(segment, torque[:,2], label="Tz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
plt.title("Torque")
plt.xlabel("Segment on whisker")
plt.legend()


plt.show()


