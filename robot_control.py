import serial
import time
from math import atan2, sqrt, pi, acos, sin, cos

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

        # 機械臂參數
        self.L1 = 0.1391  # 肩關節到手肘的長度(m)
        self.L2 = 0.1723  # 手肘到手腕的長度(m)
        self.L3 = 0.052   # 手腕到手心的長度(m)
        self.BASE_HEIGHT = 0.1  # 底座高度(m)

    def camera_to_robot_coordinates(self, camera_pos):
        """將相機座標轉換為機器人座標"""
        x = camera_pos[0]
        y = camera_pos[1]
        z = camera_pos[2]
        
        robot_x = z  # 相機的z軸對應機器人的x軸
        robot_y = -x  # 相機的x軸對應機器人的-y軸
        robot_z = y - self.BASE_HEIGHT  # 相機的y軸對應機器人的z軸
        
        return [robot_x, robot_y, robot_z]

    def is_position_reachable(self, pos):
        """檢查位置是否在機械臂工作範圍內"""
        x, y, z = pos
        distance = sqrt(x*x + y*y + z*z)
        return distance <= (self.L1 + self.L2 + self.L3)

    def set_servo_angle(self, servo_num, angle):
        """設定舵機角度"""
        if 0 <= servo_num < 16:
            command = f"{servo_num},{angle}\n"
            self.arduino.write(command.encode())
            time.sleep(0.05)

    def inverse_kinematics(self, x, y, z):
        """計算逆運動學"""
        # 計算底座旋轉角度
        base_angle = atan2(y, x) * 180 / pi
        
        # 計算在x-z平面的投影距離
        r = sqrt(x*x + y*y)
        
        # 計算肩關節和手肘關節角度
        d = sqrt(r*r + z*z)
        cos_theta2 = (d*d - self.L1*self.L1 - self.L2*self.L2) / (2*self.L1*self.L2)
        cos_theta2 = min(1, max(-1, cos_theta2))
        
        theta2 = acos(cos_theta2)
        theta1 = atan2(z, r) + atan2(self.L2*sin(theta2), self.L1 + self.L2*cos(theta2))
        
        shoulder_angle = theta1 * 180 / pi
        elbow_angle = theta2 * 180 / pi
        
        return base_angle, shoulder_angle, elbow_angle

    def grab_object(self, pos):
        """控制機械臂抓取物體"""
        try:
            robot_pos = self.camera_to_robot_coordinates(pos)
            if not self.is_position_reachable(robot_pos):
                raise Exception("目標位置超出機械臂工作範圍")
                
            base_angle, shoulder_angle, elbow_angle = self.inverse_kinematics(*robot_pos)
            
            # 控制底座
            self.set_servo_angle(self.baseServo, base_angle)
            time.sleep(0.5)
            
            # 控制肩關節
            self.set_servo_angle(self.shoulderServo1, shoulder_angle)
            self.set_servo_angle(self.shoulderServo2, 180 - shoulder_angle)
            time.sleep(0.5)
            
            # 控制手肘
            self.set_servo_angle(self.elbowServo, elbow_angle)
            time.sleep(0.5)
            
            # 控制夾爪
            self.set_servo_angle(self.gripperServo, self.gripper_max)
            time.sleep(1)
            self.set_servo_angle(self.gripperServo, self.gripper_min)
            time.sleep(1)
            
            return True
        except Exception as e:
            print(f"抓取失敗: {e}")
            return False

    def reset_position(self):
        """重置機械臂到初始位置"""
        self.set_servo_angle(self.baseServo, 90)
        time.sleep(0.5)
        self.set_servo_angle(self.shoulderServo1, 90)
        self.set_servo_angle(self.shoulderServo2, 90)
        time.sleep(0.5)
        self.set_servo_angle(self.elbowServo, 90)
        time.sleep(0.5)
        self.set_servo_angle(self.wristServo, 90)
        time.sleep(0.5)
        self.set_servo_angle(self.gripperServo, self.gripper_min)

    def close(self):
        """關閉串口連接"""
        self.arduino.close()

if __name__ == '__main__':
    # 測試代碼
    robot = RobotArm()
   