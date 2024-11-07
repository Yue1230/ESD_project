import serial
import time

class RobotArm:
    def __init__(self, port='COM11', baud_rate=9600):
        # 初始化串口連接
        self.arduino = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # 等待Arduino重置

        # 定義舵機號碼
        self.shoulderServo1 = 0 
        self.shoulderServo2 = 1 
        self.elbowServo = 2     
        self.wristServo = 3     
        self.gripperServo = 4
        self.baseServo = 5
        
        # 定義角度限制
        self.shoulder_min, self.shoulder_max = 0, 180
        self.elbow_min, self.elbow_max = 20, 180
        self.wrist_min, self.wrist_max = 0, 180
        self.gripper_min, self.gripper_max = 20, 100
        self.base_min, self.base_max = 0, 360
        
        # 設定 A 點和 B 點的固定角度
        self.a_point_angles = {'base': 90, 'shoulder': 42, 'elbow': 20, 'wrist': 156, 'gripper': 50}
        self.b_point_angles = {'base': 40, 'shoulder': 82, 'elbow': 20, 'wrist': 156, 'gripper': 20}

    def gradual_move(self, servo_num, start_angle, end_angle):
        """逐步將舵機從起始角度移動到目標角度"""
        step = 1 if end_angle > start_angle else -1
        for angle in range(start_angle, end_angle + step, step):
            command = f"{servo_num},{angle}\n"
            self.arduino.write(command.encode())
            time.sleep(0.02)  # 控制每度的延遲，調整此值以控制速度

    def move_to_angle(self, servo_num, angle):
        """從當前角度逐步移動到指定角度"""
        # 假設初始角度為90，可根據實際需要調整或在類中記錄每個舵機的當前角度
        # 根據需要追蹤每個舵機的當前角度
        self.gradual_move(servo_num, 90, angle)

    def move_to_a_point_and_grab(self):
        """移動到 A 點並夾取物體"""
        print("移動到 A 點並夾取物體")
        self.move_to_angle(self.elbowServo, self.a_point_angles['elbow'])
        self.move_to_angle(self.wristServo, self.a_point_angles['wrist'])
        self.move_to_angle(self.shoulderServo1, self.a_point_angles['shoulder'])
        self.move_to_angle(self.shoulderServo2, 180 - self.a_point_angles['shoulder'])
        self.move_to_angle(self.gripperServo, self.a_point_angles['gripper'])

    def move_to_b_point(self):
        """移動到 B 點並釋放物體"""
        print("移動到 B 點並釋放物體")
        self.move_to_angle(self.shoulderServo1, self.b_point_angles['shoulder'])
        self.move_to_angle(self.shoulderServo2, 180 - self.b_point_angles['shoulder'])
        self.move_to_angle(self.baseServo, self.b_point_angles['base'])
        self.move_to_angle(self.gripperServo, self.b_point_angles['gripper'])

    def reset_position(self):
        """重置機械臂到初始位置"""
        print("重置到初始位置")
        self.move_to_angle(self.baseServo, 90)
        self.move_to_angle(self.shoulderServo1, 90)
        self.move_to_angle(self.shoulderServo2, 90)
        self.move_to_angle(self.elbowServo, 90)
        self.move_to_angle(self.wristServo, 90)
        self.move_to_angle(self.gripperServo, self.gripper_min)

    def close(self):
        """關閉串口連接"""
        self.arduino.close()

if __name__ == '__main__':
    # 測試代碼
    robot = RobotArm()
    try:
        # 重置到初始位置
        robot.reset_position()
        time.sleep(1)  # 等待5秒

        # 移動到 A 點並夾取物體
        robot.move_to_a_point_and_grab()
        time.sleep(2)
        # 移動到 B 點並釋放物體
        robot.move_to_b_point()

        # 重置位置
        robot.reset_position()
        
    finally:
        robot.close()
