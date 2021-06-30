import rtde_control
import rtde_receive
import time
import csv
from std_msgs.msg import String
from copy import deepcopy

UR_IP = "192.168.131.40"

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)

# print actual joint position
actual_q = rtde_r.getActualQ()
print(actual_q)

#print actual cartesian position
actual_cart = rtde_r.getActualTCPPose()
print(actual_cart)

# move to hardcoded joint position
#folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
#folded_joints = [1.7080562114715576, -2.1501243750201624, 1.0715022087097168, -3.026783291493551, -1.6824644247638147, -0.1454990545855921]
#folded_joints = [1.57, -1.57, -1.1, -3.026783291493551, -1.6824644247638147, -0.1454990545855921] test servo
inp = input("Stand the robot? y/n: ")[0]
if (inp == 'y'):
    standed_joints = [1.57, -1.57, -1.1, -3.026783291493551, -1.6824644247638147, -0.1454990545855921]
    rtde_c.moveJ(standed_joints, 0.1, 0.01)
else:
    print ("Skipping")

t_list = []
q_list = []
qd_list = []
tar_qd_list = []
com_qd_list = []
q_list = []
tar_q_list = []

accel = 3
dt = 0.008
j_speed = [0.0, 0.0, 0.001, 0.0, 0.0, 0.0]

# Movement in speed mode
inp = input("Test speeding? y/n: ")[0]
if (inp == 'y'):  
    tar_qd_list.append(rtde_r.getTargetQd())
    com_qd_list.append( deepcopy(j_speed) )
    t_list.append(time.time())
    q_list.append(rtde_r.getActualQ())
    tar_q_list.append(rtde_r.getTargetQ())
    qd_list.append(rtde_r.getActualQd()) 
    for i in range (25):
        start_time = time.time()
        rtde_c.speedJ(j_speed, accel, dt)
        j_speed[2] += (i+1) * 0.001
        com_qd_list.append( deepcopy(j_speed) )
        #print(i)
        t_list.append(time.time())
        q_list.append(rtde_r.getActualQ())
        tar_q_list.append(rtde_r.getTargetQ())
        qd_list.append(rtde_r.getActualQd())
        tar_qd_list.append(rtde_r.getTargetQd())
        end_time = time.time()
        duration_1 = end_time - start_time
        if duration_1 < dt:
            time.sleep(dt - duration_1)
    for i in range (25):
        start_time = time.time()
        rtde_c.speedJ(j_speed, accel, dt)
        j_speed[2] -= (i+1) * 0.001
        tar_qd_list.append(rtde_r.getTargetQd())
        com_qd_list.append( deepcopy(j_speed) )
        #print(i)
        t_list.append(time.time())
        q_list.append(rtde_r.getActualQ())
        tar_q_list.append(rtde_r.getTargetQ())
        qd_list.append(rtde_r.getActualQd())
        end_time = time.time()
        duration_1 = end_time - start_time
        if duration_1 < dt:
            time.sleep(dt - duration_1)       
    #time.sleep(0.029)
    rtde_c.speedStop()
else:
    print ("Skipping")

inp = input("Print parameter lists? y/n: ")[0]
previous_qd = 0
if (inp == 'y'):
    for i in range (51):
        qdd = (qd_list[i][2] - previous_qd) / dt
        # print (i, t_list[i][2], com_qd_list[i][2], tar_qd_list[i][2], qd_list[i][2], qd_list[i][2], qdd)
        print (i, '%.4f' % t_list[i], '%.4f' % com_qd_list[i][2], '%.4f' % tar_qd_list[i][2], 
                  '%.4f' % qd_list[i][2], '%.4f' % qdd)
        previous_qd = qd_list[i][2]


#writing parameters into csv
inp = input("Save parameter list to .csv? y/n: ")[0]
previous_qd = 0
if (inp == 'y'):
    with open('Repos/manipulator/data_files/trial.csv', 'x', newline='') as csvfile:
        param_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        param_writer.writerow( ['number', 'timestamp','commanded_velocity', 'target_velocity', 'actual_velocity',
                                'acceleration', 'target_position', 'measured_position'] )
        for i in range (51):
            qdd = (qd_list[i][2] - previous_qd) / dt
            param_writer.writerow ([ i, t_list[i], com_qd_list[i][2], tar_qd_list[i][2], qd_list[i][2], qdd, q_list[i][2], 
                                        tar_q_list[i][2] ])
            previous_qd = qd_list[i][2]
else:
    print ("Skipping")

inp = input("Fold the robot? y/n: ")[0]
if (inp == 'y'):
    folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
    rtde_c.moveJ(folded_joints, 0.1, 0.01)
else:
    print ("Skipping")
