import subprocess
import threading
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pyzbar.pyzbar as pyzbar
import math
from geometry_msgs.msg import PoseArray
import transforms3d as t3d
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

camSet = "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1200, format=(string)NV12, framerate=(fraction)60/1 ! queue ! nvvidconv ! video/x-raw,width=1920,height=1200,format=BGRx ! queue ! videoconvert ! appsink drop=True"

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
standard_step = 0.35
standard_step_limit = 0.06
diagonal_distance = 0
standard_distance = 0.76
standard_angle = 480
euler_angles = None
distance_to_qr_code1 = 0
distance_to_qr_code2 = 0
output = None

# 定义要执行的命令，注意这里使用了列表形式来避免shell注入风险
command = [
    'ros2', 'topic', 'list',
    '|', 'grep', 'tita',
    '|', 'awk', '-F/', "'{print $2}'",
    '|', 'head', '-n', '1',
    '|', 'sed', "'s/\\..*//'"
]

# 由于管道操作符'|'在shell中用于连接多个命令，但在Python的subprocess中不能直接使用
# 我们需要将命令拆分成多个部分，并逐个执行，通过管道连接它们的输出和输入
# 但为了简化，这里我们采用一种方法，将整个命令作为一个字符串传递给shell（注意风险）
# 如果要更安全地执行，需要分别调用每个命令并处理它们的输出
# 但由于您的命令相对简单且固定，这里采用简化方法
# 注意：这种方法存在shell注入风险，如果命令中的输入（如'tita'）来自不可信的源，则不应使用
command_str = ' '.join(command)

try:
    # 使用subprocess.run来执行命令并捕获输出
    result = subprocess.run(command_str, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    # 获取命令的输出（stdout）
    output = result.stdout.strip()
    # 检查输出是否以'tita'开头（这里假设大小写敏感）
    if not output.startswith('tita'):
        # 输出错误消息或引发异常
        print("Error: Extracted topic name does not start with 'tita'. Extracted name:", output)
        raise ValueError("Extracted topic name does not start with 'tita'.")
    else:
        print("Extracted topic name:", output)
except subprocess.CalledProcessError as e:
    # 如果命令执行失败（返回非零退出码），则捕获异常并打印错误信息
    raise ValueError("Error executing command")
    print("Error executing command:", e)
    print("Stderr output:", e.stderr)

class JoyPublisher(Node):
    def __init__(self):
        global output
        super().__init__('code_follow')
        self.publisher_ = self.create_publisher(Joy, f"/{output}/joy", 10)  # 创建一个发布者
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.02, self.timer_callback1)
        # self.camera = cv2.VideoCapture(camSet, cv2.CAP_GSTREAMER)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )
        self._lock_axes_mode = False
        self._lock_timer = None
        self.subscription = self.create_subscription(
            Joy,
            f"/{output}/joy",
            self.listener_callback1,
            10
        )
        self.subscription  # 防止被垃圾回收

    def listener_callback1(self, msg):
        global axes_mode,stop_time
        # print(f"The 9th axis value is: {msg.axes[8]}")
        # print(stop_time)
        if len(msg.axes) > 8:
            if stop_time <= 1000:
                stop_time = stop_time + 1
            if msg.axes[8] == 4 and not self._lock_axes_mode:
                self._lock_axes_mode = True
                if axes_mode == 0:
                    axes_mode = 1
                else:
                    axes_mode = 0
                    stop_time = 0
                # 取消任何现有的定时器
                if self._lock_timer is not None:
                    self._lock_timer.cancel()
                self._lock_timer = threading.Timer(1.0, self.unlock_wrapper)
                self._lock_timer.start()
        if stop_time > 1000:
            axes_mode = 1

    # 设置一个新的定时器
    def unlock_axes_mode(self):
        self._lock_axes_mode = False  # 注意：这里有一个问题，self在全局作用域中未定义

    # 注意：由于self._lock_axes_mode是实例变量，我们应该使用实例方法来访问它
    # 因此，我们需要一个绑定到当前实例的方法作为定时器回调
    def unlock_wrapper(self):
        self.unlock_axes_mode()

    def listener_callback(self, msg):
        global axes5, axes3, axes4, axes6, axes_mode,c_angle, euler_angles,z_value,step, distance_to_qr_code1,distance_to_qr_code2,msg_flag
        msg_flag = 1
        c_angle = int(msg.header.frame_id.split('_')[0])
        c_id = int(msg.header.frame_id.split('_')[-1])
        if c_id == 2:
            for pose in msg.poses:
                position = pose.position
                orientation = pose.orientation
                distance_to_qr_code1 = 0.55*math.sqrt(position.x**2 + position.y**2 + position.z**2)
                z_value = orientation.z
                if orientation.z / orientation.x > 0:
                    z_value = abs(z_value)
                else:
                    z_value = -abs(z_value)
            # 假设 orientation 是一个包含四元数 [w, x, y, z] 的对象
            quaternion = [orientation.w, orientation.x, orientation.y, orientation.z]

            # 将四元数转换为旋转矩阵
            rotation_matrix = t3d.quaternions.quat2mat(quaternion)

            # 将旋转矩阵转换为欧拉角（使用ZYX顺序，即Tait-Bryan角，也称为yaw, pitch, roll）
            euler_angles = t3d.euler.mat2euler(rotation_matrix, axes='sxyz')  # 注意这里是'zyx'，不是'sxyz'

            # 将欧拉角从弧度转换为度
            # yaw_deg = math.degrees(euler_angles[0])
            # pitch_deg = math.degrees(euler_angles[1])
            # roll_deg = math.degrees(euler_angles[2])

    def decode_qr_code(self):
        global axes5, axes3, axes4, axes6, axes_mode,c_angle,z_value,msg_flag,step,distance_to_qr_code1,distance_to_qr_code2,stop_time,diagonal_distance,euler_angles

        if axes_mode == 1:
            step = 0
            #print("遥控模式:", step,distance_to_qr_code1,c_angle)
            return

        if msg_flag == 1:
            # if 0.50 < distance_to_qr_code1 < 0.80 :
            #     axes3= 0.0
            # if distance_to_qr_code1 > 0.80 :
            #     if distance_to_qr_code1 > 1.2 :
            #         axes3= -1.5
            #     else:
            #         axes3= -1.0

            if distance_to_qr_code1 < 0.50 :
                axes3= (0.50-distance_to_qr_code1)*2
            else:
                axes3= (0.50-distance_to_qr_code1)*3

            # if standard_angle-30 <= c_angle < standard_angle+10:
            #     axes4 = 0
            # elif c_angle < standard_angle-20:
            #     axes4 = 0.2
            # elif c_angle > standard_angle+20:
            #     axes4 = -0.2
            axes4 = (standard_angle - c_angle )* 1.8 / 480

        elif msg_flag > 50:
            axes3= 0.0
            if axes4 < 0: 
                axes4 = -0.2
            if axes4 > 0: 
                axes4 = 0.2
        #print("跟随模式:", distance_to_qr_code1,c_angle)

        if msg_flag < 1000:
            msg_flag = msg_flag + 1

    def decodeDisplay(self,image):
        lista = []
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
            (y, x, w, h) = barcode.rect
            print("x:{} y:{} w:{} h:{}".format(x, y,w,h))
            centerRotation = [w/2,h/2]
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
            thresh1 = image[x:x+h,y:y+w]
            GrayImage=cv2.cvtColor(thresh1,cv2.COLOR_BGR2GRAY)
            ret,frame=cv2.cv2.threshold(GrayImage,127,255,cv2.THRESH_BINARY)
            #plt.imshow(frame,'gray')
            #plt.show()
            for pix_x in range(5,h-5):
                for pix_y in range(5,w-5):
                    nb = 0
                    for row in range(pix_x-5,pix_x+5):
                        for clo in range(pix_y-5,pix_y+5):
                            gery = frame[row][clo]
                            if gery==0:
                                nb +=1
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
        if axes_mode == 1:
            return
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
        ]
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

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
