import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt
import math
import ctypes


xml_path = 'Whisker_strain_based.xml' #xml file (assumes this is in the same folder as this file)

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

count_seg = 0

def force_distance_sweep_live(segment_num_start, deflection, tendon_frc, tendon_length):
        global last_logged_second
        global count
        global count_seg
        global Whisker_len
        tendon_length_L = 0
        tendon_length_R = 0

        applied_force=10
        selection = 0
        Whisker_len=np.linspace(0, 0.211, num=16)
        segment_num_end= 16
        match segment_num_start:
            case 2:
                sensor_select = data.sensordata[4:6].copy() #segmented aluminium
                sensor_select_t = data.sensordata[6:8].copy()
                segment_num_end = 17
                selection = 0
                Whisker_len=np.linspace(0, 0.211, num=16)
            case 19:
                sensor_select = data.sensordata[0:2].copy() #composite nylon
                sensor_select_t = data.sensordata[2:4].copy()
                segment_num_end = 38
                selection = 1
                Whisker_len=np.linspace(0, 0.2015, num=20)
        
        current_second = int(data.time)

        if current_second != last_logged_second:
            last_logged_second = current_second
            count += 1
            if (count >2):
                if(count == 4):
                    print(count, " jjj " ,data.time)
                    data.xfrc_applied[segment_num_start, :3] = [0, applied_force,0]
                    data.xfrc_applied[segment_num_start-1, :3] = [0,0,0]
                    
                    deflection.append(data.geom_xpos[segment_num_end].copy())
                    
                    tendon_frc.append(sensor_select_t) 
                    
                    tendon_length_L = sensor_select[0]-0.05 
                    tendon_length_R = sensor_select[1]-0.05

                    tendon_length.append([tendon_length_L,tendon_length_R])

                elif(count >4 and count%1 ==0):
                     count_seg +=1
                     segment_num_start = segment_num_start+(count_seg)
                     data.xfrc_applied[segment_num_start, :3] = [0, applied_force,0]
                     data.xfrc_applied[segment_num_start-1, :3] = [0,0,0]
                    
                     deflection.append(data.geom_xpos[segment_num_end].copy())

                     tendon_length_L = sensor_select[0]-0.05 
                     tendon_length_R = sensor_select[1]-0.05

                     tendon_length.append([tendon_length_L,tendon_length_R])
                    #tendon_length.append(sensor_select) 
                     tendon_frc.append(sensor_select_t) 
                     print(count, "  " ,data.time, " ", segment_num_start)
                     if(segment_num_start == segment_num_end):
                       data.time = simend
                     
                     
        #deflection = np.array(deflection).T.tolist()
        #force = np.array(force).T.tolist()
    #t_data_cont = np.array(t_data_cont).T.tolist()
        #print(force)
        return deflection, tendon_frc,tendon_length

def print_world_details():
    mj.mj_step(model, data)
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

    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "sitestartR")
    site_pos_Al = data.site_xpos[site_id]
    print(f"Site 'sitestartR' position: {site_pos_Al}")

    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "siteend")
    site_pos_Al = data.site_xpos[site_id]
    print(f"Site 'siteend' position: {site_pos_Al}")
    
    print(sensor_data)


deviation = []
tendon_length = []
Whisker_len =[]
last_logged_second = -1  # put this outside the simulation loop
count = 0
tendon_frc = []
strain_based_whisker = 19

print_world_details()


def controller(model, data):
    global deviation
    global tendon_length
    global Whisker_len
    global tendon_frc
 
    force_distance_sweep_live(strain_based_whisker, deviation,tendon_frc,tendon_length)

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


#whisker_len=np.linspace(0, 0.2015, num=20)

deviation = np.array(deviation)
tendon_frc= np.array(tendon_frc)
tendon_length = np.array(tendon_length)


#
if(len(deviation)>18):
    segment = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
else:
    segment = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]

plt.subplot(1, 3, 1)  # (nrows, ncols, index)
plt.plot(segment, deviation[:,0], label="x", color="blue", marker='o')
plt.plot(segment, deviation[:,1], label="y", color="red", marker='o')
plt.title("Displacement of last segment ")
plt.xlabel("Segment Number")
plt.tight_layout()
plt.grid()
plt.legend()


plt.subplot(1, 3, 2)
plt.plot(segment, tendon_frc[:,0], label="tendon_frcL", marker='o')
plt.plot(segment, tendon_frc[:,1], label="tendon_frcR", marker='o')
#plt.plot(i,tendon_frc[:,2], label="fz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
plt.title("Constraint Force(N)")
plt.xlabel("Segment Number")
plt.legend()

plt.subplot(1, 3, 3)
plt.plot(segment, tendon_length[:,0], label="tendon_lengthL", marker='o')
plt.plot(segment, tendon_length[:,1], label="tendon_lengthR", marker='o')
#plt.plot(i, torque[:,2], label="Tz", marker='o')
   #plt.ylim(-5, 10)  # Set Y-axis range
plt.title("Tendon length(m)")
plt.xlabel("Segment Number")
plt.legend()


plt.show()


