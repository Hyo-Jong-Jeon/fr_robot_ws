import sys
from src.make_json import JsonMaker

''' Sample Code
    js = JsonMaker(title="Tool 1 Attach")
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_relative([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel_acc=[0.05, 0.1])
    js.delay(750)
    js.joint_absolute([179.467, -69.619, -113.653, -86.736, 90.318, 89.498], vel_acc=[0.1, 0.5])
    js.joint_relative([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel_acc=[0.1, 0.5])
    js.task_list.extend(task_10())
    
    pos1 = [-0.316, 0.177, 0.325, -3.141, 0.0, -0.0] + [0.1, 0.2] + [0.001]
    pos2 = [-0.346, 0.172, 0.326, -3.141, -0.0, 0.0] + [0.01, 0.2] + [0.001]
    pos3 = [-0.346, 0.172, 0.298, -3.141, -0.0, -0.0] + [0.1, 0.2] + [0.001]
    pos4 = [-0.317, 0.172, 0.298, -3.141, -0.0, -0.0] + [0.1, 0.2] + [0.0]
    path = [pos1, pos2, pos3, pos4]
    js.linear_path(path)
    
    pos1 = [-100.832, -135.19, 145.727, -10.513, 84.662, -90.004] + [0.25, 0.5] + [0.01]
    pos2 = [-96.028, -117.262, 139.619, -22.329, 89.464, -90.002] + [0.25, 0.5] + [0.01]
    pos3 = [-92.554, -110.761, 144.248, -33.461, 92.939, -90.003] + [0.25, 0.5] + [0.01]
    pos4 = [-92.556, -107.697, 146.434, -38.71, 92.936, -89.999] + [0.02, 0.2] + [0.0]
    js.joint_path([pos1, pos2, pos3, pos4])
    
    js.save_json()
    return js.task_list
'''



def list_sum(a, b):
    length = len(a)
    temp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(length):
        temp[i] = a[i] + b[i]
    return temp

def home_pos(js:JsonMaker, vel_acc=[0.5, 1.0]):
    js.joint_absolute([179.469, -69.617, -113.655, -86.737, 90.318, 89.5], vel_acc=vel_acc)

def task_1():
    # Tool 1 Attach Posistion
    js = JsonMaker(title="Tool 1 Attach")
    
    pos = [-0.316, -0.005, 0.205, -3.141, 0.0, 0.0]
    move_z = 0.03
    move_y = 0.075
    
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, move_y, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    js.save_json()
    return js.task_list
    
def task_2():
    # Tool 1 Dettach Posistion
    js = JsonMaker(title="Tool 1 Dettach")
    
    pos = [-0.316, -0.005, 0.206, -3.141, 0.0, 0.0]
    move_z = 0.03
    move_y = 0.075
    
    js.linear_absolute(list_sum(pos, [0.0, move_y, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(250)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    js.save_json()
    return js.task_list

def task_3():
    # Tool 1 45 deg attach dettach
    js = JsonMaker(title="Tool 1 Attach")
    js.task_list.extend(demo_1())
    js.task_list.extend(demo_2())
    js.task_list.extend(demo_3())
    js.task_list.extend(demo_1_reverse())
    js.task_list.extend(demo_2_reverse())
    js.task_list.extend(demo_3_reverse())
    js.rewind()
    js.save_json()
    return js.task_list

def task_4():
    # Tool 1 Attach Posistion
    js = JsonMaker(title="Tool 1 Attach")
    
    pos = [-0.316-0.095, -0.005, 0.205, -3.141, 0.0, 0.0]
    move_z = 0.03
    move_y = 0.075
    
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, move_y, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    js.save_json()
    return js.task_list
    
def task_5():
    # Tool 1 Dettach Posistion
    js = JsonMaker(title="Tool 1 Dettach")
    
    pos = [-0.316-0.095, -0.005, 0.205, -3.141, 0.0, 0.0]
    move_z = 0.03
    move_y = 0.075
    
    js.linear_absolute(list_sum(pos, [0.0, move_y, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(250)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    js.save_json()
    return js.task_list

def task_6():
    js = JsonMaker(title="Tool 1 Attach")
    js.task_list.extend(task_1())
    js.task_list.extend(task_10())
    js.task_list.extend(task_11())
    js.task_list.extend(task_7())
    js.task_list.extend(task_8())
    js.task_list.extend(task_2())
    js.task_list.extend(task_4())
    home_pos(js)
    js.task_list.extend(task_5())
    js.rewind()
    js.save_json()
    return js.task_list

def task_7():
    # Tool 1 Attach Posistion
    js = JsonMaker(title="Tool 1 Attach")
    
    # pos = [-0.374, 0.3725, 0.1735, 2.408, -0.0, 0.001]
    # pos = [-0.374-0.0005, 0.3725-0.001, 0.1735, 2.408, -0.0, 0.001]
    pos = [-0.374-0.0005, 0.3725-0.001, 0.1735, 2.398, -0.0, 0.001]
    move_z = 0.05
    move_y = 0.05
    
    home_pos(js)
    js.linear_absolute(list_sum(pos, [0.0, move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, -move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    home_pos(js)
    js.save_json()
    return js.task_list
    
def task_8():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Tool 3 Dettach")
    
    pos = [-0.374, 0.373, 0.174, 2.408, -0.0, 0.001]
    move_z = 0.05
    move_y = 0.05
    
    js.linear_absolute(list_sum(pos, [0.0, -move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    home_pos(js)
    js.save_json()
    return js.task_list

def task_9():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 3")
    js.task_list.extend(task_7())
    js.task_list.extend(task_8())
    js.rewind()
    js.save_json()
    return js.task_list

def task_10():
    # Tool 4 Attach Posistion
    js = JsonMaker(title="Tool 1 Attach")
    
    pos = [-0.374+0.101, 0.3725, 0.1735, 2.408, -0.0, 0.001]
    move_z = 0.05
    move_y = 0.05
    
    home_pos(js)
    js.linear_absolute(list_sum(pos, [0.0, move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, -move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    home_pos(js)
    js.save_json()
    return js.task_list
    
def task_11():
    # Tool 4 Dettach Posistion
    js = JsonMaker(title="Tool 4 Dettach")
    
    pos = [-0.374+0.101, 0.373, 0.174, 2.408, -0.0, 0.001]
    move_z = 0.05
    move_y = 0.05
    
    js.linear_absolute(list_sum(pos, [0.0, -move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.1])
    js.delay(750)
    js.linear_absolute(list_sum(pos, [0.0, move_y, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.5])
    home_pos(js)
    js.save_json()
    return js.task_list

def task_12():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 4")
    pos = [-0.292, 0.453, 0.184, -3.122, -0.124, -0.049]
    move_z = 0.05
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.2])
    js.vacuum_on()
    js.delay(3000)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.2])
    js.delay(3000)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.2])
    js.delay(4000)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.05, 0.2])
    js.vacuum_off()
    js.delay(3000)
    js.linear_absolute(list_sum(pos, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.1, 0.2])
    # js.task_list.extend(zeus_DRAM_grasp())
    js.save_json()
    return js.task_list

def task_16():
    js = JsonMaker(title="Demo 4")
    js.task_list.extend(demo_1())
    js.task_list.extend(demo_2())
    js.task_list.extend(demo_3())
    js.save_json()
    return js.task_list

def task_17():
    js = JsonMaker(title="Demo 4")
    js.task_list.extend(zeus_DRAM_grasp())
    js.save_json()
    return js.task_list 

def demo_1():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos1 = [-0.289, 0.67, 0.242, -3.142, -0.01, -0.001]
    pos2 = [-0.022, 0.688, 0.226, -3.141, -0.01, -0.001]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_1())
    js.gripper_init()
    js.delay(5000)
    home_pos(js)
    js.task_list.extend(task_10())
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(500)
    js.delay(1000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(2500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(500)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_11())
    
    js.save_json()
    return js.task_list

def demo_1_reverse():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos2 = [-0.289, 0.67, 0.242, -3.142, -0.01, -0.001]
    pos1 = [-0.022, 0.688, 0.226, -3.141, -0.01, -0.001]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_1())
    js.gripper_init()
    js.delay(5000)
    home_pos(js)
    js.task_list.extend(task_10())
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(500)
    js.delay(1000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(2500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(500)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_11())
    
    js.save_json()
    return js.task_list

def demo_2():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos1 = [-0.179, 0.671, 0.239, -2.214, -2.228, -0.0]
    pos2 = [0.128, 0.688, 0.225, -2.214, -2.228, -0.001]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_7())
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(2000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(2000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_8())
    js.task_list.extend(task_2())
    
    js.save_json()
    return js.task_list

def demo_2_reverse():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos2 = [-0.179, 0.671, 0.239, -2.214, -2.228, -0.0]
    pos1 = [0.128, 0.688, 0.225, -2.214, -2.228, -0.001]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_7())
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(2000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(2000)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_8())
    js.task_list.extend(task_2())
    
    js.save_json()
    return js.task_list

def demo_3():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos1 = [-0.376, 0.67, 0.126, -3.141, -0.0, -0.0]
    pos2 = [0.125, 0.549, 0.12, -3.141, -0.0, -0.0]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_4())
    js.gripper_init()
    js.delay(2000)
    home_pos(js)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(1500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_5())
    
    js.save_json()
    return js.task_list

def demo_3_reverse():
    # Tool 3 Dettach Posistion
    js = JsonMaker(title="Demo 1")
    pos2 = [-0.376, 0.67, 0.126, -3.141, -0.0, -0.0]
    pos1 = [0.125, 0.549, 0.12, -3.141, -0.0, -0.0]
    move_z = 0.12
    home_pos(js)
    js.task_list.extend(task_4())
    js.gripper_init()
    js.delay(2000)
    home_pos(js)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(1000)
    js.delay(1500)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    js.delay(1500)
    js.linear_absolute(list_sum(pos2, [0.0, 0.0, move_z, 0.0, 0.0, 0.0]), vel_acc=[0.2, 0.5])
    js.gripper_finger_pos(0)
    home_pos(js)
    js.task_list.extend(task_5())
    
    js.save_json()
    return js.task_list

def zeus_DRAM_grasp():
    # 제우스 램 파지 작업업
    js = JsonMaker(title="Zeus DRAM Grasp Task!")
    pos1 = [-0.411, 0.296, 0.3, -3.141, 0.0, -0.0]
    home_pos(js)
    js.task_list.extend(task_4())
    home_pos(js)
    js.linear_absolute(list_sum(pos1, [0.0, 0.0, -0.01, 0.0, 0.0, 0.0]))
    js.task_list.extend(task_5())
    
    js.save_json()
    return js.task_list

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Error: No task specified. Use 'task1', 'task2', or other available tasks.")
        sys.exit(1)

    # 명령줄 인자로 받은 작업 이름
    task_name = sys.argv[1].lower()

    # 동적으로 함수 호출
    available_tasks = {name: func for name, func in globals().items() if callable(func) and name.startswith("task")}

    if task_name in available_tasks:
        available_tasks[task_name]()
    else:
        print(f"Error: Unknown task '{task_name}'. Available tasks: {', '.join(available_tasks.keys())}.")