import subprocess
import threading
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import mediapipe as mp
import numpy as np
import pyzbar.pyzbar as pyzbar
import math
from geometry_msgs.msg import PoseArray
import transforms3d as t3d
from math import sqrt
from std_msgs.msg import Int32

standard_step = 0.40
standard_distance = 0.76
standard_angle_7 = 870

# 定义输出文件路径
output_file = '/home/robot/btmon_output.txt'
axes1 = 0
axes2 = 0
axes3 = 0
axes4 = 0
axes5 = 0
axes6 = -1
axes7 = 0
axes8 = 1

z_value = 0
msg_flag = 2
axes_mode = 1
step = 0
stop_time = 0
c_angle = 0
standard_step_limit = 0.06
diagonal_distance = 0
standard_angle = 480
euler_angles = None
distance_to_qr_code = 0

def read_file_in_real_time(file_path):
    # 初始化变量
    last_size = 0
    search_bytes = b'\x00\x04\x00\x1b'  # 要搜索的十六进制序列
    acl_handle_found = False  # 标记是否找到了 ACL Handle
    global axes5
    global axes3
    global axes4
    global axes6
    global axes7
    global axes8
    global axes_mode
    while True:
        # 获取当前文件大小
        current_size = os.path.getsize(file_path)
        # 如果文件大小增加了
        if current_size > last_size:
            # 打开文件并读取新增的内容
            with open(file_path, 'rb') as file:
                file.seek(last_size)
                new_data = file.read(current_size - last_size)
                # 在新数据中搜索特定的十六进制序列
                search_result = new_data.find(search_bytes)
                while search_result != -1:
                    # 计算 ACL Handle 的索引
                    acl_handle_index = search_result + len(search_bytes)
                    if acl_handle_index + 4 <= len(new_data):
                        # 读取 ACL Handle
                        acl_handle = new_data[acl_handle_index:acl_handle_index + 2]
                        # 读取 ACL Handle 后紧接着的两个字节
                        acl_data = new_data[acl_handle_index + 2:acl_handle_index + 4]
                        #print(f"Found ACL Handle: {acl_handle.hex()}")
                        print(f"ACL Data: {acl_data.hex()}")
                        if acl_handle.hex() == "1f00" and acl_data.hex() == "0050":
                            if axes_mode==1:
                                axes3 = 0.0
                                axes4 = 0.0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0010":
                            axes3 = -1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0090":
                            axes3 = 1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0040":
                            axes4 = 1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0060":
                            axes4 = -1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "8050":
                            if axes5 == 1:
                                axes5 = -1
                            else:
                                axes5 = 1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "1052":
                            if axes6 == -1:
                                axes6 = 0
                            else:
                                axes6 = -1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "4050":
                            if axes_mode == 1:
                                axes_mode = 0
                            else:
                                axes_mode = 1
                                axes3=0
                                axes4 = 0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0851":
                            if axes7 == 0:
                                axes7 = 1
                            else:
                                axes7 = 0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0150":
                            if axes8 == 0.5:
                                axes8 = 1
                            else:
                                axes8 = 0.5

                        acl_handle_found = True  # 标记为已找到
                    else:
                        # 数据不足，等待下一次读取
                        print("New data is not enough to contain ACL Handle and ACL Data after the sequence.")

                    # 移动搜索起始点，继续搜索下一个可能的匹配项（如果有的话）
                    search_result = new_data.find(search_bytes, search_result + 1)

            # 更新 last_size 以反映当前文件大小
            last_size = current_size

        # 如果文件大小减小（虽然这种情况很少见，但理论上可能发生），更新 last_size
        if current_size < last_size:
            last_size = current_size

        # 如果已经找到了 ACL Handle 并且您想停止循环，可以在这里添加逻辑
        # 例如：if acl_handle_found: break

        # 等待一段时间再检查
        time.sleep(0.1)

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher_bt')
        self.publisher_ = self.create_publisher(Joy, '/tita3037207/joy', 10)  # 创建一个发布者
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.02, self.timer_callback1)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )
        self.subscription_state = self.create_subscription(
            Int32,
            '/state',
            self.state_callback,
            10  # QoS参数
        )
        self.subscription  # 防止被垃圾回收

    def state_callback(self, msg):
        global axes_mode
        # Int32消息的回调函数
        if msg.data == 1:
            axes_mode = 0
            print("入仓充电")

    def listener_callback(self, msg):
        global axes5, axes3, axes4, axes6, axes_mode,c_angle, euler_angles,z_value,step, distance_to_qr_code,msg_flag
        msg_flag = 1
        c_angle = int(msg.header.frame_id.split('_')[0])
        c_id = int(msg.header.frame_id.split('_')[-1])
        if c_id == 2:
            for pose in msg.poses:
                position = pose.position
                orientation = pose.orientation
                distance_to_qr_code = 1.6*math.sqrt(position.x**2 + position.y**2 + position.z**2)
                z_value = orientation.w
                if orientation.w / orientation.x > 0:
                    z_value = abs(z_value)
                else:
                    z_value = -abs(z_value)
            #假设 orientation 是一个包含四元数 [w, x, y, z] 的对象
            quaternion = [orientation.w, orientation.x, orientation.y, orientation.z]

            #将四元数转换为旋转矩阵
            rotation_matrix = t3d.quaternions.quat2mat(quaternion)

            #将旋转矩阵转换为欧拉角（使用ZYX顺序，即Tait-Bryan角，也称为yaw, pitch, roll）
            euler_angles = t3d.euler.mat2euler(rotation_matrix, axes='rzyx')  # 注意这里是'zyx'，不是'sxyz'
            # euler_angles = 2 * sqrt(1 - (orientation.w) ** 2) * z_value
            # 将欧拉角从弧度转换为度
            # yaw_deg = math.degrees(euler_angles[0])
            # pitch_deg = math.degrees(euler_angles[1])
            # roll_deg = math.degrees(euler_angles[2])
            # print("欧拉角",yaw_deg,pitch_deg,roll_deg)

    def decode_qr_code(self):
        global axes5, axes3, axes4, axes6, axes_mode,c_angle,z_value,msg_flag,step,distance_to_qr_code,stop_time,diagonal_distance,euler_angles

        if axes_mode == 1:
            step = 0
            if euler_angles != None:
                print("步数:", step,distance_to_qr_code,c_angle,z_value,math.degrees(euler_angles[2]))
            return
        if euler_angles != None:
            angle_radians = euler_angles[2]  # 这里不需要转换，因为假设它已经是弧度了
            safe_angle_radians = max(abs(angle_radians), 1e-10)  # 避免除以零
        # if step < 5:
        #     if math.degrees(euler_angles[1]) < -30:
        #         distance_to_qr_code = distance_to_qr_code + 0.10
        #     if math.degrees(euler_angles[1]) > -20:
        #         distance_to_qr_code = distance_to_qr_code - 0.35
        if diagonal_distance > 2:
            diagonal_distance = 0
            step = 0
            
        if step == 0:
            axes4 = 0.5
            if msg_flag == 1:
                axes4 = 0.0
                step = 1
        if step == 1:
            if standard_angle-10 <= c_angle < standard_angle+10:
                axes4 = 0
                step = 2
                diagonal_distance = standard_step / math.sin(safe_angle_radians) -  0.1*math.sin(safe_angle_radians)
            elif c_angle < standard_angle-10:
                axes4 = (standard_angle-10 - c_angle)/200
                if axes4 > 0.5:
                    axes4 = 0.5
            elif c_angle > standard_angle+10:
                axes4 = (standard_angle+10 - c_angle)/200
                if axes4 < -0.5:
                    axes4 = -0.5
        if step == 2:
            if diagonal_distance-standard_step_limit < distance_to_qr_code < diagonal_distance+standard_step_limit :
                axes3= 0.0
                step = 3
            if distance_to_qr_code > diagonal_distance+standard_step_limit :
                axes3= (diagonal_distance+standard_step_limit - distance_to_qr_code)*3.5
                if axes3 < -0.5:
                    axes3 = -0.5
                stop_time = 0
            if distance_to_qr_code < diagonal_distance-standard_step_limit :
                axes3= (diagonal_distance-standard_step_limit - distance_to_qr_code)*1.5
                if axes3 > 0.2:
                    axes3 = 0.2
                stop_time = 0
        if step == 3:
            stop_time = stop_time + 1
            if stop_time >= 100:
                if abs(diagonal_distance -  standard_step / math.sin(safe_angle_radians) + 0.1*math.sin(safe_angle_radians)) > 0.05:
                    step = 0
                if diagonal_distance-standard_step_limit < distance_to_qr_code < diagonal_distance+standard_step_limit :
                    step = 4
                else:
                    step = 2
        if step == 4:
            if msg_flag == 1:
                if  0 < math.degrees(euler_angles[2]) < 176:
                    axes4 = (math.degrees(euler_angles[2]) - 176)/50
                elif -180 < math.degrees(euler_angles[2]) < 0:
                    axes4 = (math.degrees(euler_angles[2]) + 180)/50
                else:
                    axes4 = 0.0
                    step = 5
            elif msg_flag > 30:
                axes4 = 0
                axes3= 0.2
                step = -1
                stop_time = 0
        if step == 5:
            if msg_flag < 50:
                if standard_distance-standard_step_limit < distance_to_qr_code < standard_distance+standard_step_limit :
                    axes3= 0.0
                    step = 6
                elif distance_to_qr_code > standard_distance+standard_step_limit :
                    axes3= (standard_distance+standard_step_limit - distance_to_qr_code)*3.5
                    if axes3 < -0.4:
                        axes3 = -0.4
                    stop_time = 0
                elif distance_to_qr_code < standard_distance-standard_step_limit :
                    axes3= (standard_distance-standard_step_limit - distance_to_qr_code)*1.5
                    if axes3 > 0.2:
                        axes3 = 0.2
                    stop_time = 0
            else:
                axes3= 0.2
                step = -1
        if step == 6:
            stop_time = stop_time + 1
            if stop_time >= 100:
                if standard_distance-standard_step_limit < distance_to_qr_code < standard_distance+standard_step_limit :
                    step = 7
                else:
                    step = 5
        if step == 7:
            if standard_angle_7-50 <= c_angle < standard_angle_7+50:
                step = 8
            else:
                step = 0
        if step == 8:
            axes3= -0.5
            step = 9
            stop_time = 0
        if step == 9:
            stop_time = stop_time + 1
            if stop_time >= 200:
                axes3 = 0
                step = 10

        if step == -1:
            stop_time = stop_time + 1
            if stop_time >= 100:
                axes3 = 0
                step = 0

        if euler_angles != None:
            print("步数:", step,distance_to_qr_code,c_angle,z_value,diagonal_distance,msg_flag,math.degrees(euler_angles[2]))

        if msg_flag < 1000:
            msg_flag = msg_flag + 1

    def decodeDisplay(self,image):
        #取出二维码区域并转为灰度图，求出三个定位点，返回三个定位点的坐标已经二维码区域的中心点坐标
        lista = []
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
            # (x, y, w, h) = barcode.rect
            (y, x, w, h) = barcode.rect#暂时不知道什么缘故xy需要互换位置
            print("x:{} y:{} w:{} h:{}".format(x, y,w,h))
            centerRotation = [w/2,h/2]
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
            thresh1 = image[x:x+h,y:y+w]    #对数组进行剪裁
            GrayImage=cv2.cvtColor(thresh1,cv2.COLOR_BGR2GRAY)
            ret,frame=cv2.cv2.threshold(GrayImage,127,255,cv2.THRESH_BINARY)
            #plt.imshow(frame,'gray')
            #plt.show()
            for pix_x in range(5,h-5):
                for pix_y in range(5,w-5):
                    # gery = image[pix_x][pix_y]
                    # print("pix_x:{} pix_y:{} gery:{}".format(pix_x, pix_y, gery))
                    # if gery==0:
                    #     nb +=1
                    nb = 0
                    for row in range(pix_x-5,pix_x+5):
                        for clo in range(pix_y-5,pix_y+5):
                            gery = frame[row][clo]
                            # print("row:{} clo:{} gery:{}".format(row, clo, gery))
                            if gery==0:
                                nb +=1
                    # print(nb)
                    if(nb>99):
                        lista.append([pix_y,pix_x])
        print(lista)
        return image,lista,centerRotation

    def intercept(self,a,b):
        #求出两点的截距
        selfx=a[0]-b[0]
        selfy=a[1]-b[1]
        selflen= math.sqrt((selfx**2)+(selfy**2))
        return selflen

    def SeekingRightAngles(self,lista):
        # 求三点连成三角形的三个角度，返回最大角度（直角）所对应的点
        a = self.intercept(lista[0],lista[1]) #intercept01;
        b = self.intercept(lista[1],lista[2]) #intercept12;
        c = self.intercept(lista[2],lista[0]) #intercept20;
        A=math.degrees(math.acos((a*a-b*b-c*c)/(-2*b*c)))#点2处的夹角
        B=math.degrees(math.acos((b*b-a*a-c*c)/(-2*a*c)))#点0处的夹角
        C=math.degrees(math.acos((c*c-a*a-b*b)/(-2*a*b)))#点1处的夹角
        print(A)
        print(B)
        print(C)
        if A>B and A>C:return lista[2]
        if B>A and B>C:return lista[0]
        if C>B and C>A:return lista[1]

    def RotationAngle(self,rightpoint,centerRotation):
        # 以左上角45度为参照求旋转角度，旋转角度为二维码区域的中心点分别与 （0，0） 以及 90度角对应的点 的 连线 的夹角
        lista = [[0,0],rightpoint,centerRotation]
        slope = rightpoint[1]/rightpoint[0]#求直角点的斜率，用于判断顺时针还是逆时针旋转
        if slope == 1:#如果斜率等于1 则有0和180度两种可能
            if rightpoint[0]<centerRotation[0]/2: return 0#关键点的x坐标小于旋转中心的x坐标
            else:return 180
        else:
            a = intercept(lista[0],lista[1]) #intercept01;
            b = intercept(lista[1],lista[2]) #intercept12;
            c = intercept(lista[2],lista[0]) #intercept20;
            A=math.degrees(math.acos((a*a-b*b-c*c)/(-2*b*c)))#旋转角
            if slope<1: return -A#如果斜率小于1 逆时针旋转
            if slope>1: return A#如果斜率大于1 顺时针旋转

    def timer_callback1(self):
        try:
            self.decode_qr_code()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def timer_callback(self):
        global axes5
        global axes3
        global axes4
        global axes6
        global axes7
        global axes8
        global axes_mode
        joy_msg = Joy()
        joy_msg.header.frame_id = "joy"
        joy_msg.axes = [
            float(0),
            float(0),
            float(axes3),
            float(axes4),
            float(axes5),
            float(axes6),
            float(axes7),
            float(axes8),
            float(0)
        ]
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

    process = subprocess.Popen(['sudo', 'btmon', '-w', output_file], stderr=subprocess.PIPE)
    reader_thread = threading.Thread(target=read_file_in_real_time, args=(output_file,))
    reader_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        reader_thread.join()
        print("程序已终止")


if __name__ == '__main__':
    main()