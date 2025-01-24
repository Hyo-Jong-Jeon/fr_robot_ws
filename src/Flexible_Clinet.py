import socket
import threading
import time
import json


class FlexibleClient:
    hostname = "BOOK-E1I23S8BLS.local"
    def __init__(self, port=4000, protocol='UDP', send_interval=0.1, receive_interval=1.0):
        """
        TCP/UDP 클라이언트 초기화
        :param protocol: 'TCP' 또는 'UDP'
        :param send_interval: 주기적으로 데이터를 전송하는 간격 (초)
        :param receive_interval: 수신 데이터를 기다리는 간격 (초)
        """        
        try:
            self.ip_address = socket.gethostbyname(self.hostname)
            print(f"Resolved IP address: {self.ip_address}")
        except socket.gaierror:
            print(f"Failed to resolve hostname: {self.hostname}")
            exit()
        self.port = port
        self.protocol = protocol.upper()  # TCP 또는 UDP
        self.send_interval = send_interval
        self.receive_interval = receive_interval
        self.running = True

        if self.protocol == 'TCP':
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        elif self.protocol == 'UDP':
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            raise ValueError("Invalid protocol. Use 'TCP' or 'UDP'.")

    def connect(self):
        """
        TCP 연결 설정
        """
        if self.protocol == 'TCP':
            self.client_socket.connect((self.ip_address, self.port))
            print(f"Connected to {self.ip_address}:{self.port} via TCP")

    def send_data(self, data):
        """
        JSON 데이터를 서버로 전송
        """

        try:
            if self.protocol == 'TCP':
                self.client_socket.sendall(json.dumps(data).encode('utf-8'))
            elif self.protocol == 'UDP':
                self.client_socket.sendto(json.dumps(data).encode('utf-8'), (self.ip_address, self.port))
            # print(f"Sent: {data}")
        except Exception as e:
            print(f"Error sending data: {e}")
            self.running = False

    def receive_data(self):
        """
        서버로부터 데이터 수신
        """
        while self.running:
            try:
                if self.protocol == 'TCP':
                    response = self.client_socket.recv(1024).decode('utf-8')
                    if not response:
                        print("Server closed the connection.")
                        break
                elif self.protocol == 'UDP':
                    response, address = self.client_socket.recvfrom(1024)
                    print(f"Received from {address}: {response.decode('utf-8')}")

                print(f"Received: {response}")
            except socket.timeout:
                # 타임아웃 시 다음 루프로 진행
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
                self.running = False

            # CPU 점유율을 낮추기 위해 대기
            time.sleep(self.receive_interval)


    def close(self):
        """
        클라이언트 종료
        """
        self.running = False
        self.client_socket.close()
        print("Connection closed")

    def start(self):
        """
        클라이언트를 시작하고 송신 및 수신 스레드 실행
        """
        try:
            # TCP 연결일 경우 서버에 연결
            if self.protocol == 'TCP':
                self.connect()

            # 수신 스레드 실행
            # receive_thread = threading.Thread(target=self.receive_data)
            # receive_thread.start()

            # 주기적으로 데이터 전송
            while self.running:
                self.send_data(self.generate_random_value())
                time.sleep(self.send_interval)

            # # 수신 스레드가 종료될 때까지 대기
            # receive_thread.join()
        except KeyboardInterrupt:
            print("\nShutting down client...")
            self.close()

    def generate_random_value(self):
        """
        랜덤 데이터 생성 (예: 0 ~ 100 사이의 값)
        """
        import random
        data = {
            "timestamp": round(time.time(),3),
            "value": random.randint(0, 100)
        }
        # print(data)        
        return data


if __name__ == "__main__":
    # TCP 또는 UDP 클라이언트 실행
    client = FlexibleClient(protocol='UDP')  # 'TCP' 또는 'UDP' 선택
    client.start()
  
