import time

from fairino import Robot

class FrState:
    def __init__(self):
        try:
            self.robot = Robot.RPC('192.168.58.2')
        except Exception as e:
            print(e)

    def close_robot(self):
        return self.robot.CloseRPC()

    # mode: 0-Manual mode, 1-Auto mode
    def mode_switch(self, mode):
        return self.robot.Mode(mode) #Robot cuts to autorun mode
    
    # drag mode: 1-entry into drag-indicator mode, 0-exit from drag-indicator mode
    def teach_mode(self, mode):
        return self.robot.DragTeachSwitch(mode)
    
    def is_teach_mode(self):
        return self.robot.IsInDragTeach()

    # up_down_enable_robot: 1 - up enable, 0 - down enable
    def up_down_enable_robot(self, state):
        return self.robot.RobotEnable(state)
    
    # ref: 0 - joint point movement, 2 - base coordinate system point movement, 4 - tool coordinate system point movement, 8 - workpiece coordinate system point movement;
    # nb: 1-1 joint (x-axis), 2-2 joint (y-axis), 3-3 joint (z-axis), 4-4 joint (rx), 5-5 joint (ry), 6-6 joint (rz).
    # dir: 0 - negative direction, 1 - positive direction.
    # max_dis: maximum angle/distance of a single tap in ° or mm;
    def jog_start(self, ref, nb, dir, max_dis):
        return self.robot.StartJOG(ref, nb, dir, max_dis)
    
    # ref:1 - joint point stop, 3 - base coordinate system point stop, 5 - tool coordinate system point stop, 9 - workpiece coordinate system point stop
    def jog_stop(self, ref):
        return self.robot.StopJOG()
    
    def imm_jog_stop(self):
        return self.robot.ImmStopJOG()
    
    # joint_pos: target joint position in [°];
    # tool: tool number, [0 to 14];
    # user: artifact number, [0 to 14];
    
    # desc_pos: target Cartesian position in [mm][°] Default initial value [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls positive kinematics to solve for the return value.
    # vel: percentage of speed, [0~100] default 20.0.
    # acc: percentage of acceleration, [0~100], not open yet;
    # ovl: velocity scaling factor, [0~100] default 100.0.
    # exaxis_pos: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    # blendT:[-1.0]-motion in place (blocking), [0~500.0]-smoothing time (non-blocking) in [ms] default -1.0;
    # offset_flag:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    # offset_pos: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0];
    def movej(self, joint_pos, vel = 20, asynchronous = -1.0):
        return self.robot.MoveJ(joint_pos, vel=vel, tool=0, user=0, blendT=asynchronous)
    
    # desc_pos: target Cartesian position in [mm][°];
    # tool: tool number, [0 to 14];x
    # user: artifact number, [0 to 14];

    # joint_pos: target joint position in [°] Default initial value is [0.0,0.0,0.0,0.0,0.0,0.0,0.0], default value calls inverse kinematics to solve for the return value.
    # vel: percentage of speed, [0~100] default 20.0;
    # acc: acceleration percentage, [0~100], not open Default 0.0;
    # ovl: velocity scaling factor, [0~100] default 100.0;
    # blendR:blendR:[-1.0]-movement in place (blocking), [0~1000]-smoothing radius (non-blocking) in [mm] default -1.0;
    # exaxis_pos: external axis 1 position ~ external axis 4 position Default [0.0,0.0,0.0,0.0].
    # search: [0] - no wire search, [1] - wire search;
    # offset_flag:offset_flag:[0]-no offset, [1]-offset in workpiece/base coordinate system, [2]-offset in tool coordinate system Default 0;
    # offset_pos: position offset in [mm][°] default [0.0,0.0,0.0,0.0,0.0,0.0]
    # overSpeedStrategy: over speed handling strategy, 0 - strategy off; 1 - standard; 2 - stop on error when over speeding; 3 - adaptive speed reduction, default 0
    # speedPercent: Percentage of allowable speed reduction threshold [0-100], default 10%
    def movel(self, desc_pos, vel = 20, asynchronous = -1.0):
        return self.robot.MoveL(desc_pos, vel=vel, tool = 0, user = 0, overSpeedStrategy = 0, blendR = asynchronous)

    # desc_pos: target Cartesian position;
    # tool: tool number, [0 to 14];
    # user: artifact number, [0 to 14];
    def movel_path(self, desc_pos, asynchronous = -1.0):
        # 나중에 경로 리스트 받아서 정리하기
        return self.robot.MoveCart(desc_pos, tool = 0, user = 0, blendT = asynchronous)

    # desc_pos_p: path point Cartesian position in [mm][°];
    # tool_p: pathpoint tool number, [0~14].
    # user_p: pathpoint artifact number, [0~14].
    # desc_pos_t: Cartesian position of the target point in [mm][°].
    # tool_t: tool number, [0 to 14];
    # user_t: artifact number, [0~14];
    def movec(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t):
        return self.robot.MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t)

    def stop_move(self):
        return self.robot.StopMotion()
    
    def getRobotMotionDone(self):
        return self.robot.GetRobotMotionDone()
    
    def getRobotErrorCode(self):
        return self.robot.GetRobotErrorCode()

    def set_payload(self, payload):
        return self.robot.SetLoadWeight(payload) # kg
    
    def get_payload(self, flag=1):
        return self.robot.GetTargetPayload(flag=flag) # kg
    
    def set_load_center(self, tcp):
        return self.robot.SetLoadCoord(tcp) # mm, mm, mm

    # method: 0-flat loading, 1-side loading, 2-hanging loading
    def set_mounting_method(self, method):
        return self.robot.SetRobotInstallPos(method) # 0-no mounting, 1-mounting on base, 2-mounting on tool, 3-mounting on workpiece
    
    # flag: 0-blocking, 1-non-blocking, default 1
    def getActualQ(self, flag = 1):
        return self.robot.GetActualJointPosDegree(flag=flag) # [°]
    # flag: 0-blocking, 1-non-blocking, default 1
    
    def getActualQd(self, flag = 1):
        return self.robot.GetActualJointSpeedsDegree(flag=flag)
    
    def getActualQdd(self, flag = 1):
        return self.robot.GetJointDriverTorque(flag=flag)
    
    def getActualTCPPose(self, flag = 1):
        return self.robot.GetActualTCPPose(flag=flag) # [m, m, m, °, °, °]
    
    def getActualTCPSpeed(self, flag = 1):
        return self.robot.GetActualTCPSpeed(flag=flag) # [mm/s, mm/s, mm/s, °/s, °/s, °/s]

    def getActualToolFlangePose(self, flag = 1):
        return self.robot.GetActualToolFlangePose(flag=flag) # [m, m, m, °, °, °]

    # type: 0-absolute position (base coordinate system), 1-relative position (base coordinate system), 2-relative position (tool coordinate system)
    # desc_pose:[x,y,z,rx,ry,rz], tool position in [mm][°]
    def getInverseKinematics(self, type, desc_pos, config=-1):
        return self.robot.GetInverseKin(type, desc_pos, config=config) # [°]

    def getJointTorques(self, flag = 1):
        return self.robot.GetJointTorques(flag=flag) # [Nm]
