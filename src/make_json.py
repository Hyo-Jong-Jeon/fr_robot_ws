import json
import os
import inspect

class JsonMaker:
    def __init__(self, title):
        self.task_list = []
        
        function_name = inspect.currentframe().f_back.f_code.co_name  
        self.filename = function_name + ".json"

        self.title = title
        cwd = os.getcwd()        
        self.directory = cwd + "/task_list"

    def set_title(self, title):
        """타이틀 설정"""
        self.title = title

    def set_filename(self, filename):
        """파일 이름 설정"""
        self.filename = filename  


    def joint_absolute(self, target, vel_acc = [0.1, 0.2], unit="deg, deg/s, deg/s^2"):    
        task = {"task": "Joint_absolute", "target": target, "vel_acc": vel_acc, "unit": unit}
        self.task_list.append(task)

    def joint_relative(self, target, vel_acc = [0.1, 0.2], unit="deg, deg/s, deg/s^2"):    
        task = {"task": "Joint_relative", "target": target, "vel_acc": vel_acc, "unit": unit}
        self.task_list.append(task)
        
    def joint_path(self, target, unit="joint[6], vel, acc, radius"):
        task = {"task": "Joint_path", "target": target, "unit": unit}
        self.task_list.append(task)
        #ssssss


    def linear_absolute(self, target, vel_acc = [0.1, 0.2], unit="m, rad, m/s, m/s^2"):
        task = {"task": "Linear_absolute", "target": target, "vel_acc": vel_acc, "unit": unit}
        self.task_list.append(task)

    def linear_relative(self, target, vel_acc = [0.1, 0.2], unit="m, rad, m/s, m/s^2"):
        task = {"task": "Linear_relative", "target": target, "vel_acc": vel_acc, "unit": unit}
        self.task_list.append(task)
        
    def linear_path(self, target, unit="joint[6], vel, acc, radius"):
        task = {"task": "Linear_path", "target": target, "unit": unit}
        self.task_list.append(task)
        
    def tool_linear(self, target, vel_acc = [0.1, 0.2], unit="m, rad, m/s, m/s^2"):
        task = {"task": "Tool_Linear", "target": target, "vel_acc": vel_acc, "unit": unit}
        self.task_list.append(task)
        

    def delay(self, target, unit="ms"):
        task = {"task": "Delay", "target": target, "unit": unit}
        self.task_list.append(task)


    def gripper_init(self):
        task = {"task": "Gripper_init", "target": 0, "unit": " "}
        self.task_list.append(task)

    def gripper_open(self):
        task = {"task": "Gripper_open", "target": 0, "unit": " "}
        self.task_list.append(task)

    def gripper_close(self):
        task = {"task": "Gripper_close", "target": 0, "unit": ""}
        self.task_list.append(task)


    def gripper_finger_pos(self, target, unit=""):
        task = {"task": "Gripper_finger_pos", "target": target, "unit": unit}
        self.task_list.append(task)

    def motor_pos(self, target, unit=""):
        task = {"task": "Motor_pos", "target": target, "unit": unit}
        self.task_list.append(task)


    def vacuum_on(self):
        task = {"task": "Vacuum_on", "target": 0, "unit": " "}
        self.task_list.append(task)

    def vacuum_off(self):
        task = {"task": "Vacuum_off", "target": 0, "unit": " "}
        self.task_list.append(task)
        
        
    def rewind(self):
        task = {"task": "Rewind", "target": 0}
        self.task_list.append(task)


    def save_json(self):
        filepath = os.path.join(self.directory, self.filename)

        with open(filepath, "w") as file:
            file.write('{\n')  # JSON 시작
            file.write(f'    "title": "{self.title}",\n')  # title 추가
            file.write('    "task_list": [\n')  # task_list 시작
            for i, task in enumerate(self.task_list):
                json_line = json.dumps(task)  # 개별 작업 JSON 변환
                if i < len(self.task_list) - 1:  # 마지막 작업이 아니면 쉼표 추가
                    json_line += ","
                file.write(f"        {json_line}\n")  # 줄바꿈과 들여쓰기
            file.write("    ]\n}")  # JSON 끝
        print(f"JSON saved to {filepath}")

    def clear_tasks(self):
        """작업 리스트 초기화"""
        self.task_list = []
