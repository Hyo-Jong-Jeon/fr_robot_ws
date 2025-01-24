import sys
import time

# import rtde_receive
# import rtde_control
from fairino import Robot

from threading import Thread
from math import pi
from Flexible_Clinet import FlexibleClient #241231_plotjuggler

class FR_Interface(Thread):
    def __init__(self, ip='192.168.58.2'):
        self.robot = None
        self.plotjuggler = False
        self.refreshRate = 0.01 # Second
        self.connection = False
        # self.stop_state = {'ProtectiveStopped': False, 'EmergencyStopped': False}
        # self.robot_state = {'RobotMode': '', 'RobotStatus': '', 'SafetyMode': '', 'ActualRobotVoltage': 0, 'ActualRobotCurrent': 0}
        self.joint_state = {'actualQ': [0,0,0,0,0,0], 'actualQd': [0,0,0,0,0,0], 'actualCurrent': [0,0,0,0,0,0]}
        self.tcp_state = {'actualPos': [0,0,0,0,0,0], 'actualSpeed': [0,0,0,0,0,0], 'actualForce': [0,0,0,0,0,0]}
        self.error_code = 0
        
        try:
            super().__init__()
            # self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
            # self.rtde_c = rtde_control.RTDEControlInterface(ip)
            self.robot = Robot.RPC(ip)
            if self.plotjuggler:
                try:
                    self.UDP_client = FlexibleClient(protocol='UDP')
                    print(">> PlotJuggler successfully connected.")
                except Exception as e:
                    print(e)
        except Exception as e:
            self.connection = False
            print(e)
            print('<<ERROR>>: Try again')
        else:
            self.connection = True
            print('>> FR Successfully Connected.', )
        
    def run(self):
        try:
            print("Start local thread.")
            while True:
                # self.stop_state['ProtectiveStopped'] = self.isProtectiveStopped()
                # self.stop_state['EmergencyStopped'] = self.isEmergencyStopped()
                # self.robot_state['ActualRobotVoltage'] = self.getActualRobotVoltage()
                # self.robot_state['ActualRobotCurrent'] = self.getActualRobotCurrent()
                # self.robot_state['RobotMode'] = self.getRobotMode()
                # self.robot_state['RobotStatus'] = self.getRobotStatus()
                # self.robot_state['SafetyMode'] = self.getSafetyMode()
                self.joint_state['actualQ'] = self.getActualQ()
                self.joint_state['actualQd'] = self.getActualQd()
                self.joint_state['actualCurrent'] = self.getActualCurrent()
                self.tcp_state['actualPos'] = self.getActualTCPPose()
                self.tcp_state['actualSpeed'] = self.getActualTCPSpeed()
                self.error_code = self.getRobotErrorCode()
                # self.tcp_state['actualForce'] = self.getFtRawWrench()
                # self.tcp_state['actualForce'] = self.getActualTCPForce() 
                if self.plotjuggler:  #241231_plotjuggler             
                    self.UDP_client.send_data(self.joint_state)
                    self.UDP_client.send_data(self.tcp_state) 
                    # print(self.joint_state)                
                time.sleep(self.refreshRate)

        except KeyboardInterrupt:
            self.disconnect()
            print("\nDisconnect the communication.")
        except Exception as e:
            self.connection = False
            self.disconnect()
            print(e)
    # Read UR Status
    def readFR(self):
        try:
            if self.connection:
                # self.stop_state['ProtectiveStopped'] = self.isProtectiveStopped()
                # self.stop_state['EmergencyStopped'] = self.isEmergencyStopped()
                # self.robot_state['ActualRobotVoltage'] = self.getActualRobotVoltage()
                # self.robot_state['ActualRobotCurrent'] = self.getActualRobotCurrent()
                # self.robot_state['RobotMode'] = self.getRobotMode()
                # self.robot_state['RobotStatus'] = self.getRobotStatus()
                # self.robot_state['SafetyMode'] = self.getSafetyMode()
                self.joint_state['actualQ'] = self.getActualQ()
                self.joint_state['actualQd'] = self.getActualQd()
                self.joint_state['actualCurrent'] = self.getActualCurrent()
                self.tcp_state['actualPos'] = self.getActualTCPPose()
                self.tcp_state['actualSpeed'] = self.getActualTCPSpeed()
                # self.tcp_state['actualForce'] = self.getFtRawWrench()
                # self.tcp_state['actualForce'] = self.getActualTCPForce()
            # if self.plotjuggler:  #241231_plotjuggler             
            #      self.UDP_client.send_data(self.joint_state)           
        except Exception as e:
            self.connection = False
            self.disconnect()
            print(e)
    
    # Can be used to disconnect from the robot
    def disconnect(self):
        self.connection = False
        try:
            # self.rtde_r.disconnect()
            # self.rtde_c.disconnect()
            temp = self.robot.CloseRPC()
            print(">> FR Disconnected.", "RetVal:",temp)
        except Exception as e:
            print(e)
    
    # Connection status for RTDE, useful for checking for lost connection.
    # def isConnected(self):
    #     return self.rtde_r.isConnected() # bool
    
    # # Target joint positions
    # def getTargetQ(self):
    #     return self.rtde_r.getTargetQ() # vector<double>
    
    # # Target joint velocities
    # def getTargetQd(self):
    #     return self.rtde_r.getTargetQd() # vector<double>
    
    # # Target joint currents
    # def getTargetCurrent(self):
    #     return self.rtde_r.getTargetCurrent() # vector<double>
    
    # # Target joint moments(torques)
    # def getTargetMoment(self):
    #     return self.rtde_r.getTargetMoment() # vector<double>
          
    # Actual joint positions
    def getActualQ(self, flag=1):
        # return self.rtde_r.getActualQ()
        return self.robot.GetActualJointPosDegree(flag=flag) # [°]
    
    # Actual joint velocities
    def getActualQd(self, flag=1):
        # return self.rtde_r.getActualQd()
        return self.robot.GetActualJointSpeedsDegree(flag=flag)
    
    # Actual joint currents
    def getActualCurrent(self, flag=1):
        # return self.rtde_r.getActualCurrent()
        return self.robot.GetJointDriverTorque(flag=flag)
    
    # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    def getActualTCPPose(self, flag=1):
        # return self.robot.GetActualToolFlangePose(flag=flag) # [m, m, m, °, °, °] ????
        # return self.rtde_r.getActualTCPPose()
        return self.robot.GetActualTCPPose(flag=flag) # [m, m, m, °, °, °]
    
    # Actual speed of the tool given in Cartesian coordinates
    def getActualTCPSpeed(self, flag=1):
        # return self.rtde_r.getActualTCPSpeed()
        return self.robot.GetActualTCPSpeed(flag=flag) # [mm/s, mm/s, mm/s, °/s, °/s, °/s]
    
    def getRobotErrorCode(self):
        return self.robot.GetRobotErrorCode()
        
    # Generalized forces in the TCP
    # def getFtRawWrench(self):
    #     return self.rtde_r.getFtRawWrench()
    
        # Generalized forces in the TCP
    # def getActualTCPForce(self):
    #     return self.rtde_r.getActualTCPForce()
    
    # Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    # def getTargetTCPPose(self):
    #     return self.rtde_r.getTargetTCPPose()
    
    # # Target speed of the tool given in Cartesian coordinates
    # def getTargetTCPSpeed(self):
    #     return self.rtde_r.getTargetTCPSpeed()
    
    # Temperature of each joint in degrees Celsius
    # def getJointTemperatures(self):
    #     return self.rtde_r.getJointTemperatures()
    
    # Controller real-time thread execution time
    # def getActualExecutionTime(self):
    #     return self.rtde_r.getActualExecutionTime() # double
    
    # Robot mode
    # -1 = ROBOT_MODE_NO_CONTROLLER 0 = ROBOT_MODE_DISCONNECTED 1 = ROBOT_MODE_CONFIRM_SAFETY 2 = ROBOT_MODE_BOOTING 3 = ROBOT_MODE_POWER_OFF 
    # 4 = ROBOT_MODE_POWER_ON 5 = ROBOT_MODE_IDLE 6 = ROBOT_MODE_BACKDRIVE 7 = ROBOT_MODE_RUNNING 8 = ROBOT_MODE_UPDATING_FIRMWARE
    # def getRobotMode(self):
    #     mode = self.rtde_r.getRobotMode() # int32
    #     if mode == -1: return 'No Ctrl'
    #     elif mode == 0: return 'Disconnect'
    #     elif mode == 1: return 'Confirm_Safety'
    #     elif mode == 2: return 'Booting'
    #     elif mode == 3: return 'Power Off'
    #     elif mode == 4: return 'Power On'
    #     elif mode == 5: return 'Idle'
    #     elif mode == 6: return 'Backdirve'
    #     elif mode == 7: return 'Running'
    #     elif mode == 8: return 'Update Firm'
    #     else: return 'None'
        
    # # Robot status Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
    # def getRobotStatus(self):
    #     status = self.rtde_r.getRobotStatus() # uint32
    #     if status == 0: return 'Power off'
    #     elif status == 1: return 'Power on'
    #     elif status == 2: return 'Running'
    #     elif status == 4: return 'Teaching'
    #     elif status == 8: return 'Pwr Btn'
    #     else: return 'None'
    
    # # Joint control modes
    # def getJointMode(self):
    #     return self.rtde_r.getJointMode() # vector<int32>
    
    # # Safefy mode
    # def getSafetyMode(self):
    #     safetyMode = self.rtde_r.getSafetyMode() # int32
    #     if safetyMode == 0: return 'Normal'
    #     elif safetyMode == 1: return 'Reduced'
    #     elif safetyMode == 2: return 'Protective stopped'
    #     elif safetyMode == 3: return 'Recovery'
    #     elif safetyMode == 4: return 'Safeguard stop'
    #     elif safetyMode == 5: return 'System emergency'
    #     elif safetyMode == 6: return 'Robot emergency'
    #     elif safetyMode == 7: return 'Emegency stop'
    #     elif safetyMode == 8: return 'Violation'
    #     elif safetyMode == 7: return 'Fault'
    #     elif safetyMode == 8: return 'Stop due safety'
    #     else: return 'None'    
        
    # # Safety status bits Bits 0-10: Is normal mode | Is reduced mode | Is protective stopped | Is recovery mode
    # #                       | Is safeguard stopped | Is system emergency stopped | Is robot emergency stopped
    # #                       |Is emergency stopped | Is violation | Is fault | Is stopped due to safety
    # def getSafetyStatusBits(self):
    #     return self.rtde_r.getSafetyStatusBits() # int32
    
    # # Safety Control Board: Main voltage
    # def getActualMainVoltage(self):
    #     return self.rtde_r.getActualMainVoltage() # double
    
    # # Safety Control Board: Robot voltage (48V)
    # def getActualRobotVoltage(self):
    #     return self.rtde_r.getActualRobotVoltage() # double
    
    # # Safety Control Board: Robot current
    # def getActualRobotCurrent(self):
    #     return self.rtde_r.getActualRobotCurrent() # double
    
    # # Actual joint voltages
    # def getActualJointVoltage(self):
    #     return self.rtde_r.getActualJointVoltage() # vector<double>
    
    # # Program state
    # def getRuntimeState(self):
    #     return self.rtde_r.getRuntimeState() # uint32
    
    # # a bool indicating if the robot is in 'Protective stop'
    # def isProtectiveStopped(self):
    #     return self.rtde_r.isProtectiveStopped() # bool
    
    # # a bool indicating if the robot is in 'Emergency stop'
    # def isEmergencyStopped(self):
    #     return self.rtde_r.isEmergencyStopped() # bool
    
    # Get the payload of the robot in [kg].
    def getPayload(self, flag=1):
        # return self.rtde_r.getPayload() # double
        return self.robot.GetTargetPayload(flag=flag) # kg
    
    # Get the payload Center of Gravity (CoGx, CoGy, CoGz) in [m]
    # def getPayloadCog(self):
    #     return self.rtde_r.getPayloadCog() # vector<double>
    
    '''
    RTDE Control Function
    '''
    def stopL(self, a = 0.5):
        # self.rtde_c.stopL(a)
        return self.robot.StopMotion()
        
    def stopJ(self):
        # self.rtde_c.stopJ()
        return self.robot.StopMotion()
        
    # bool moveJ(const std::vector<double> &q, double speed = 1.05, double acceleration = 1.4, bool asynchronous = false)
    # a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function.
    # Default is false, this means the function will block until the movement has completed.
    def moveJ(self, target_q, speed = 1.05, acceleration = 1.4, asynchronous = -1.0):
        # return self.rtde_c.moveJ(target_q, speed, acceleration, asynchronous)
        return self.robot.MoveJ(target_q, vel=speed, tool=0, user=0, blendT=asynchronous)
        
    # def moveJ_IK(self, target_q, speed = 1.05, acceleration = 1.4, asynchronous = False):
    #     return self.rtde_c.moveJ_IK(target_q, speed, acceleration, asynchronous)
        
    # bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool asynchronous = false)
    # a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function.
    # Default is false, this means the function will block until the movement has completed.
    def moveL(self, target_pos, speed = 0.25, acceleration = 1.2, asynchronous = -1.0):
        # return self.rtde_c.moveL(target_pos, speed, acceleration, asynchronous)
        return self.robot.MoveL(target_pos, vel=speed, tool = 0, user = 0, overSpeedStrategy = 0, blendR = asynchronous)
    
    def movel_path(self, desc_pos, asynchronous = -1.0):
        # 나중에 경로 리스트 받아서 정리하기
        return self.robot.MoveCart(desc_pos, tool = 0, user = 0, blendT = asynchronous)
    
    # def moveL_FK(self, target_pos, speed = 0.25, acceleration = 1.2, asynchronous = False):
    #     return self.rtde_c.moveL_FK(target_pos, speed, acceleration, asynchronous)
    
    # def getForwardKinematics(self):
    #     return self.rtde_c.getForwardKinematics()
    
    # def getInverseKinematics(self, target_pos):
    #     return self.rtde_c.getInverseKinematics(target_pos)
    
    def speedJ(self, target_qd, acceleration = 0.5, time = 0.0):
        # return self.rtde_c.speedJ(target_qd, acceleration, time)
        for i in range(6):
            qd = target_qd[i]
            if qd >= 0: dir = 1
            else: dir = 0
            self.robot.StartJOG(0, i+1, dir, 30)
    
    def speedL(self, target_xd, acceleration = 0.25, time = 0.0):
        # return self.rtde_c.speedL(target_xd, acceleration, time)
        for i in range(6):
            qd = target_xd[i]
            if qd >= 0: dir = 1
            else: dir = 0
            self.robot.StartJOG(2, i+1, dir, 30)
            
    def tool_speedL(self, target_xd, acceleration = 0.25, time = 0.0):
        # return self.rtde_c.speedL(target_xd, acceleration, time)
        for i in range(6):
            qd = target_xd[i]
            if qd >= 0: dir = 1
            else: dir = 0
            self.robot.StartJOG(4, i+1, dir, 30)
    
    def speedStop(self, a = 10.0):
        # return self.rtde_c.speedStop(a)
        return self.robot.StopJOG()
    
    def isSteady(self):
        # retVal = self.rtde_c.isSteady()
        # return retVal
        temp = self.robot.GetRobotMotionDone()
        if temp == 0: return True
        else: return False
        
    
    # Mass in kilograms
    # Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the displacement (in meters) from the toolmount.
    # If not specified the current CoG will be used.
    def setPayLoad(self, mass, COG):
        # return self.rtde_c.setPayLaod(mass, COG)
        self.robot.SetLoadCoord(COG) # mm, mm, mm
        self.robot.SetLoadWeight(mass) # kg
        return 0
        
    # def setTcp(self, tcp_offset):
    #     return self.rtde_c.setTcp(tcp_offset)
    
    # def getTCPOffset(self):
    #     return self.rtde_c.getTCPOffset()
        
    def teachMode(self):
        # return self.rtde_c.teachMode()
        return self.robot.DragTeachSwitch(1)
    
    def endTeachMode(self):
        # return self.rtde_c.endTeachMode()
        return self.robot.DragTeachSwitch(0)
    
    # def moveUntilContact(self, xd, direction = {0, 0, 0, 0, 0, 0}, acceleration = 0.5):
    #     return self.rtde_c.moveUntilContact(xd, direction, acceleration)
    
    # Robot status Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
    # There is a synchronization gap between the three interfaces RTDE Control RTDE Receive and Dashboard Client.
    # RTDE Control and RTDE Receive open its own RTDE connection and so the internal state is not in sync.
    # That means, if RTDE Control reports, that program is running, RTDE Receive may still return that program is not running.
    # The update of the Dashboard Client even needs more time.
    # That means, the dashboard client still returns program not running after some milliseconds have passed after RTDE Control already reports program running.
    # def getRobotStatus(self):
    #     return self.rtde_c.getRobotStatus()
    
    # def getActualToolFlangePos(self):
    #     return self.rtde_c.getActualToolFlangePos()
    
    # def zeroFtSensor(self):
    #     return self.rtde_c.zeroFtSensor()
    def initialize(self):
        self.robot.SetAxleLuaEnable(1)
        self.robot.ActGripper(1,1)
        
    def open(self):
        self.robot.MoveGripper(1,100,0,0,100,0)
        
    def close(self):
        self.robot.MoveGripper(1,0,0,0,100,0)
        
    def finger_pos_set(self, pos):
        if pos > 100: pos = 100
        elif pos < 0: pos = 0
        self.robot.MoveGripper(1,pos,0,0,100,0)
        
'''
Local function declaration
'''
def rad2deg(value):
    if type(value) is list:
        temp = []
        for i in range(6):
            temp.append(value[i]*(180.0/pi))
        return temp
    else:
        return value*(180.0/pi)
        

    
    
if __name__ == '__main__':
    recv_instance = FR_Interface()
    pass