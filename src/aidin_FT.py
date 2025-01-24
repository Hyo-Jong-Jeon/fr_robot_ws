import time, signal, os
import socket, select
from threading import Thread
from Flexible_Clinet import FlexibleClient

class DataProcessor:
    def __init__(self): 
        self.force = {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0}
        self.bias = {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0}

    def process_data(self, recv_data):
        Fx = Fy = Fz = 0
        if recv_data[4] == 0x01:
            Fx = ((recv_data[6] * 256 + recv_data[7]) / 100.) - 300.
            Fy = ((recv_data[8] * 256 + recv_data[9]) / 100.) - 300.
            Fz = ((recv_data[10] * 256 + recv_data[11]) / 100.) - 300.

            # Apply bias
            Fx -= self.bias["Fx"]
            Fy -= self.bias["Fy"]
            Fz -= self.bias["Fz"]

            # Update force values
            self.force["Fx"] = Fx
            self.force["Fy"] = Fy
            self.force["Fz"] = Fz
        return self.force

    def set_bias(self, Fx, Fy, Fz):
        self.bias["Fx"] = Fx
        self.bias["Fy"] = Fy
        self.bias["Fz"] = Fz

class aidin_FT(Thread):
    IP_ADDRESS = "192.168.0.223"  # TCP 서버의 IP 주소
    PORT = 4001  # TCP 서버의 포트
    connection = False  # 서버 연결 상태

    def __init__(self, port = 4001):
        super().__init__()
        self.PORT = port
        self.CMD_TYPE_SENSOR_ID = '01'
        self.SENSOR_TRANSMIT_TYPE_MODE = '03'
        self.SENSOR_TRANSMIT_TYPE_SET_TEMP = '01'
        self.data_processor = DataProcessor()

    def run(self):
        try:
            self.tcp_connect(self.IP_ADDRESS, self.PORT)
            self.udp_client = FlexibleClient(port=4000, protocol='UDP')
        except Exception as e:
            print(e)
            self.connection = False
        else:
            print('>> Aidin FT sensor socket is connected.')
            self.connection = True
        
        while True:
            if self.connection:
            #     self.status_recv()
            #     time.sleep(0.2)  # 주기적으로 서버 데이터 수신
                ready_to_read, _, _ = select.select([self.client_socket], [], [], 1.0) 
                if ready_to_read:
                    self.status_recv()
                else:
                    time.sleep(0.1)  # 데이터가 없으면 짧게 대기

    def start_client(self):
        self.start()

    def tcp_connect(self, ip, port):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((ip, port))

    def tcp_close(self):
        self.connection = False
        self.client_socket.close()
        print('>> Aidin FT sensor socket is disconneced.')

    def status_recv(self):
        try:
            # 서버로부터 데이터 수신
            response = self.recv_msg()

            data = self.data_processor.process_data(response)      
            
            temp = {"Aidin_Force_data": data} # 데이터 감싸기
            self.udp_client.send_data(temp)

        except socket.error as e:
            print("Socket error:", e)
        except Exception as e:
            print(e)

    def set_sample_rate(self,hz):
        # 샘플링 속도 설정 명령어 생성
        rate_param: int = 1_000_000 // hz
        applied_hz: int = 1_000_000 // rate_param
        print(f"Request: {hz} Hz, Applied: {applied_hz} Hz")
        field_2 = format(rate_param // 256,'02X')
        field_3 = format(rate_param % 256,'02X')
        sample_rate_command = '04' + '00' + '00' + '01' + '02' + '06' + self.CMD_TYPE_SENSOR_ID + '05'+ field_2 + field_3 
        # print(sample_rate_command)
        sample_rate_command = bytearray.fromhex(sample_rate_command)
        self.client_socket.send(sample_rate_command)
        # self.recv_msg()

    def set_bias(self):            
        values_Fx = []
        values_Fy = []
        values_Fz = []

        # Collect 100 data points
        for _ in range(100):
            recv_data = self.recv_msg()
            if recv_data[4] == 0x01:
                Fx = ((recv_data[6] * 256 + recv_data[7]) / 100.) - 300.
                Fy = ((recv_data[8] * 256 + recv_data[9]) / 100.) - 300.
                Fz = ((recv_data[10] * 256 + recv_data[11]) / 100.) - 300.
                values_Fx.append(Fx)
                values_Fy.append(Fy)
                values_Fz.append(Fz)

        # Sort and exclude the top and bottom 10 values
        values_Fx.sort()
        values_Fy.sort()
        values_Fz.sort()

        trimmed_Fx = values_Fx[10:-10]
        trimmed_Fy = values_Fy[10:-10]
        trimmed_Fz = values_Fz[10:-10]

        # Calculate the average of the remaining values
        bias_Fx = sum(trimmed_Fx) / len(trimmed_Fx)
        bias_Fy = sum(trimmed_Fy) / len(trimmed_Fy)
        bias_Fz = sum(trimmed_Fz) / len(trimmed_Fz)

        # Set the bias
        self.data_processor.set_bias(bias_Fx, bias_Fy, bias_Fz)
        print(f'Bias set to - X: {bias_Fx}, Y: {bias_Fy}, Z: {bias_Fz}')

            # bias_command = '04' + '00' + '00' + '01' + '02' + '06' + self.CMD_TYPE_SENSOR_ID + '02'+ '01'  
            # # print(sample_rate_command)
            # bias_command = bytearray.fromhex(bias_command)
            # self.s.send(bias_command)

    def init_sensor(self):
        send_data = '04' + '00' + '00' + '01' + '02' + '06' + self.CMD_TYPE_SENSOR_ID + self.SENSOR_TRANSMIT_TYPE_MODE + self.SENSOR_TRANSMIT_TYPE_SET_TEMP
        send_data = bytearray.fromhex(send_data)
        self.client_socket.send(send_data)
        recv_data = self.recv_msg()

    def recv_msg(self):
        recv_data = bytearray(self.client_socket.recv(14))
        # self.print_msg(recv_data)
        return recv_data

    # def print_msg(self, msg):
    #     data_str = "DATA: "
    #     for i in range(6, 14):
    #         data_str += str(msg[i]) + " "
    #     # print(data_str)

    # def receive_data_thread(self):
    #     while self.running:
    #         try:
    #             recv_data = self.recv_msg()
    #             self.data_processor.process_data(recv_data)
    #         except socket.timeout:
    #             continue
    #         except Exception as e:
    #             print(f"Error receiving data: {e}")
    #             break

    # def add_event_handler(self, event_handler):
    #     self.data_processor.add_event_handler(event_handler)

def signal_handler(signal, frame):
    os._exit(-1)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    FT_sensor = aidin_FT()
    FT_sensor.start_client()

    while True:
        if FT_sensor.connection:
            break

    print('Gripper init')
    # gripper.tcp_close()