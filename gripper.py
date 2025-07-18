from pymodbus.client import ModbusTcpClient
import time

start_address=0

class Gripper: 
    def __init__(self, ip_address='192.168.1.11', port=502):
        self.client = ModbusTcpClient(ip_address, port=port)
        self.client.connect()
        time.sleep(0.5)
        activiation_cmd = [0x0100, 0x0000, 0x6464, 0, 0, 0, 0, 0]
        self.client.write_registers(start_address, activiation_cmd)
        self.gripper_state = 0  # 0: open, 1: closed
        self.position = 0  # 0: home, 1: open, 2: closed
        response = self.client.read_input_registers(address=0, count=8)
        # print("当前状态寄存器值：", response.registers)

    def open_gripper(self):
        if self.gripper_state == 1:
            open_cmd = [0x0900, 0x0000, 0x6464, 0, 0, 0, 0, 1]
            self.client.write_registers(start_address, open_cmd)
            time.sleep(1)  # Wait for the gripper to open
            self.gripper_state = 0
            response = self.client.read_input_registers(address=0, count=8)
            # print("当前状态寄存器值：", response.registers)

    def close_gripper(self):
        if self.gripper_state == 0:
            close_cmd = [0x0900, 0x00FF, 0x6464, 0, 0, 0, 0, 1]
            self.client.write_registers(start_address, close_cmd)
            time.sleep(1)  # Wait for the gripper to close
            self.gripper_state = 1
            response = self.client.read_input_registers(address=0, count=8)
            # print("当前状态寄存器值：", response.registers)

    def position_gripper(self, position):
        if position >= 0 and position <= 255:
            self.position = position
            pst_cmd = [0x0900, position, 0x6464, 0, 0, 0, 0, 1]
            self.client.write_registers(start_address, pst_cmd)
            time.sleep(0.5)
            response = self.client.read_input_registers(address=0, count=8)
            # print("当前状态寄存器值：", response.registers)
        else:
            print("Invalid position. Position must be between 0 and 255.")
    def disconnect(self):
        self.client.close()
        time.sleep(0.2)
