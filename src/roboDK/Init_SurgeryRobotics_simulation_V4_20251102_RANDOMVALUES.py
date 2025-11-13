from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json
import os
import random
import time
import tkinter.font as tkFont 

# Define the relative and absolute path to the RoboDK project file
relative_path = "src/roboDK/SurgeryRobotics.rdk"
absolute_path = os.path.abspath(relative_path)
# Constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()# semaphor to manage data from 2 threads

current_vibration = 0 ####### NEW
########### REMOVE ###########
last_random_update_time = 0 
R_end, P_end, W_end = 0, 0, 0
R_grip, P_grip, W_grip = 0, 0, 0
T1, T2, TP, TY = 0, 0, 0, 0
##############################

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize RoboDK
def initialize_robodk(absolute_path):
    RDK = Robolink()
    time.sleep(2)  # wait for RoboDK to be ready
    RDK.AddFile(absolute_path)
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.setSpeed(50)
    robot.MoveL(Init_target)
    return RDK, robot, base, gripper, needle

# Transformation Endowrist to base
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360 #es col·loca el endowrist a -90º (vertical), el corregim
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

##### NEW:
##### Function to get vibration value between 0 an 255
##### As we do in Gripper - main.cpp to get vibration:
def constrain(val, min_val=0, max_val=255):
    return max(min_val, min(max_val, val))
#####

# Function to update the label with text
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values, vib_indicator): # NEW: vibr_indicator
    ###### NEW: Find vibration from torques
    torque_values_dict = {}
    splited = torque_values.split(",")
    for i in splited:
        key, value = i.split("=")
        key = key.strip()
        value = float(value.strip())
        torque_values_dict[key] = value
    R = torque_values_dict["R1"]
    P = torque_values_dict["P"]
    Y = torque_values_dict["Y"]
    torque_value = R + P + Y
    vibration = constrain(torque_value*2.5, 0, 255) ## value between 0 an 255
    color = f"#{int(vibration):02x}{int(255 - vibration):02x}00" ## set color between green and red
    vib_indicator.after(0, lambda: vib_indicator.config(bg=color))
    global current_vibration
    current_vibration = vibration  
    if not status_message:
        status_message = "None"
    full_text = f"Tool orientation: \n{tool_orientation}\n\nGripper orientation: \n{gripper_orientation}\n\nStatus message: {status_message}\n\nTorque Values: \n{torque_values}"
    font_style = tkFont.Font(family="Arial", size=10, weight="bold")
    label.after(0, lambda: label.config(text=full_text, font=font_style))
    
    
# Function to read UDP data and update the global variable
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, data_lock, Servo_torques
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE) 
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                if device_id == "G3_Endo":
                    with data_lock:
                        Endowrist_rpy = received_data
                elif device_id == "G3_Gri":
                    with data_lock:
                        Gripper_rpy = received_data
                elif device_id == "G3_Servos":
                    with data_lock:
                        Servo_torques = received_data
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error as e:
            #print(f"Socket error in UDP reader: {e}")
            sock.close()
            print("Socket closed.")
            break

# Function to process the latest UDP data and move the robot
def move_robot(robot, gripper, needle, text_label, vib_indicator):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock
    global e_roll, e_pitch, e_yaw, g_roll, g_pitch, g_yaw, s1, s2, s3, s4
    
    endowrist_orientation_msg = ""
    gripper_orientation_msg = ""
    status_message = ""
    servo_torques_msg = ""
    
    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_Servo_torques = Servo_torques

        if current_Endowrist_rpy:
            e_roll = Endowrist_rpy.get("roll")
            e_pitch = Endowrist_rpy.get("pitch")
            e_yaw = Endowrist_rpy.get("yaw")
            s3 = Endowrist_rpy.get("s3")
            s4 = Endowrist_rpy.get("s4")
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            #print(f"Endowrist: {endo_roll}, {endo_pitch}, {endo_yaw}")
            # Move Endowrist
            endowrist_pose = robot.Pose()
            Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
            endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
            if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                robot.MoveL(endowrist_pose_new, True)
                endowrist_orientation_msg = f"R={round(endo_roll)}, P={round(endo_pitch)}, Y={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = ""
            else:
                endowrist_orientation_msg = f"R={round(endo_roll)}, P={round(endo_pitch)}, Y={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = "Robot cannot reach the position"
                
            if s3 == 0 or s4 == 0:
                current_pose = robot.Pose()
                # Z movement based on S3 and S4 buttons
                Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                new_pose = current_pose * Tz  # translació relativa

                status_message = "⬆ Botó S3 premut: pujant" if s3 == 0 else "⬇ Botó S4 premut: baixant"

                if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                    robot.MoveL(new_pose, True)
                else:
                    status_message = "❌ No es pot moure més en Z (relatiu)"
                    
        if current_Gripper_rpy: #CORREGIR A PARTIR D'AQUÍ
            g_roll = Gripper_rpy.get("roll")
            # corregir roll (provisional)
            g_roll = g_roll - endo_roll
            g_pitch = Gripper_rpy.get("pitch")
            g_pitch = g_pitch - endo_pitch #not endo_yaw perquè ja està compensat en endowrist2base_orientation
            g_yaw = Gripper_rpy.get("yaw")
            g_yaw = g_yaw - endo_yaw #not endo_pitch ""
            s1 = Gripper_rpy.get("s1")
            s2 = Gripper_rpy.get("s2")
            #print(f"Gripper: {g_roll}, {g_pitch}, {g_yaw}")
            # Move Gripper
            gripper_pose = gripper.Pose()
            Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
            # Aquesta línea. Lectura ("roll") gripper - "roll" endowrist
            gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
            gripper.setPose(gripper_pose_new)
            gripper_orientation_msg = f"R={round(g_roll)}, P={round(g_pitch)}, Y={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"    
            if s1 == 0:
                #Obre la pinça → deixa anar l’agulla
                needle.setParentStatic(base)
                status_message = "🟢 S1 premut: agulla alliberada"

            elif s1 == 1:
                #Tanca la pinça → agafa l’agulla
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                status_message = "🔵 S1 no premut: agulla agafada"
        if current_Servo_torques:
            t1 = current_Servo_torques.get("Torque_roll1", 0)
            t2 = current_Servo_torques.get("Torque_roll2", 0)
            tp = current_Servo_torques.get("Torque_pitch", 0)
            ty = current_Servo_torques.get("Torque_yaw", 0)
            servo_torques_msg = f"R1={t1:.2f}, R2={t2:.2f}, P={tp:.2f}, Y={ty:.2f}"
        else:
            servo_torques_msg = "No torque data"
        
        ########### REMOVE ###########
        global last_random_update_time, R_end, P_end, W_end, R_grip, P_grip, W_grip, T1, T2, TP, TY
        current_time = time.time()
        if current_time - last_random_update_time > 2:  
            R_end = random.uniform(0, 180)
            P_end = random.uniform(0, 180)
            W_end = random.uniform(0, 180)
            R_grip = random.uniform(0, 180)
            P_grip = random.uniform(0, 180)
            W_grip = random.uniform(0, 180)
            T1 = random.uniform(0, 60)
            T2 = random.uniform(0, 60)
            TP = random.uniform(0, 60)
            TY = random.uniform(0, 60)
            last_random_update_time = current_time
        endowrist_orientation_msg = f"R={round(R_end)} P={round(P_end)} Y={round(W_end)}"
        gripper_orientation_msg = f"R={round(R_grip)} P={round(P_grip)} Y={round(W_grip)}"
        servo_torques_msg = f"R1={T1:.2f}, R2={T2:.2f}, P={TP:.2f}, Y={TY:.2f}"
        ############################## 
        
        # Update the label with the latest values
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, servo_torques_msg, vib_indicator)

        time.sleep(READ_INTERVAL_S)# define the reading interval
        

def on_vibration_click(vibration_label):
    global current_vibration
    vibration_percent = int((current_vibration / 255) * 100)
    vibration_label.config(text=f"Vibration = {vibration_percent}%")

def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Ending Socket")
        #initialize_robodk()
        #print("Program INITIALIZED")
    except Exception as e:
        #print(f"Error al tancar el socket: {e}")
        pass
    root.destroy()

# Update functions for sliders
def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# Main function
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label, absolute_path
    
    RDK, robot, base, gripper, needle = initialize_robodk(absolute_path)

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing) # Proper clossing
    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    ############ CREATE BUTTON AND VIBRATION INDICATOR
    vib_indicator = tk.Button(root, text=f"Vibration\n click to update percentage", 
                              width=20, height=2, bg="green",
                              command=lambda: on_vibration_click(vibration_label))
    vib_indicator.pack(padx=10, pady=10)
    vibration_label = tk.Label(root, text="Vibration = 0%", font=("Arial", 10, "bold"))
    vibration_label.pack(pady=(0, 20))


    # Add sliders for ZERO_YAW_TOOL and ZERO_YAW_GRIPPER
    slider_label = tk.Label(root, text="Slide to orient the axes", font=("Arial", 10))
    slider_label.pack(pady=(10, 0)) 

    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                                    command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                        command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    # Start the UDP reading thread
    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    # Start the robot movement thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label, vib_indicator))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()
    print("Pop-up menu closed")
    RDK.CloseRoboDK()
    print("RoboDK closed")

if __name__ == "__main__":
    main()