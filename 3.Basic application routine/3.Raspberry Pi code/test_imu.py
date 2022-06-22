# 导入必要的库
import serial
import numpy as np

uart = serial.Serial("COM3", 115200, timeout=0.001)

FRAME_HEAD = 0xfc
FRAME_END = 0xfd
TYPE_IMU = 0x40
TYPE_AHRS = 0x41

IMU_LEN = 0x38    # 56+8  8组数据
AHRS_LEN = 0x30   # 48+8  7组数据

IMU_RS = 64
AHRS_RS = 56
INSGPS_RS = 80

Count = 0
last_rsnum = 0
rx_imu_flag = 0
rx_ahrs_flag = 0

temp_data = [0 for i in range(64)]
raw_imu = [0 for i in range(64)]
raw_ahrs = [0 for i in range(56)]
new_imu_data = 0
new_ahrs_data = 0

def Byte2Hex(dat):
    return int(''.join(["%02X" % x for x in dat]).strip(), 16)

# Composite timestamp  合成时间戳
def Makeup_Timestamp(Data_1, Data_2, Data_3, Data_4):
    transition_32 = 0
    transition_32 |= Data_4 << 24
    transition_32 |= Data_3 << 16
    transition_32 |= Data_2 << 8
    transition_32 |= Data_1
    return transition_32

# generated data  合成数据
def Makeup_Data(Data_1, Data_2, Data_3, Data_4):
    tmp = 0
    sign = 0
    exponent = 0
    mantissa = 0
    transition_32 = 0
    transition_32 |= Data_4 << 24
    transition_32 |= Data_3 << 16
    transition_32 |= Data_2 << 8
    transition_32 |= Data_1
    if (transition_32 & 0x80000000) == 0:
        sign = 1
    else:
        sign = -1
    exponent = ((transition_32 >> 23) & 0xff) - 127
    mantissa = 1 + ((transition_32 & 0x7fffff) / 0x7fffff)
    tmp = sign * mantissa * pow(2, exponent)
    return tmp

# 接收串口数据，判断是否符合协议
# Receives serial port data and checks whether it complies with the protocol
def Rx_imu_data(rx_temp):
    global Count, last_rsnum, rx_imu_flag, rx_ahrs_flag
    global new_imu_data, new_ahrs_data, raw_imu, raw_ahrs
    temp_data[Count] = rx_temp
    if ((last_rsnum == FRAME_END) and (rx_temp == FRAME_HEAD)) or Count > 0:
        Count = Count + 1
        if (temp_data[1] == TYPE_IMU) and (temp_data[2] == IMU_LEN):
            rx_imu_flag = 1
        if (temp_data[1] == TYPE_AHRS) and (temp_data[2] == AHRS_LEN):
            rx_ahrs_flag = 1
    else:
        Count = 0
    last_rsnum = rx_temp

    if rx_imu_flag == 1 and Count == IMU_RS:
        Count = 0
        rx_imu_flag = 0
        new_imu_data = 1
        if temp_data[IMU_RS - 1] == FRAME_END:
            raw_imu = temp_data
    
    if rx_ahrs_flag == 1 and Count == AHRS_RS:
        Count = 0
        rx_ahrs_flag = 0
        new_ahrs_data = 1
        if temp_data[AHRS_RS - 1] == FRAME_END:
            raw_ahrs = temp_data
    

# 解析九轴惯性传感器模块的数据
# Parse the data of the 9-axis IMU module 
def Parse_imu_data():
    global new_imu_data, new_ahrs_data
    if new_ahrs_data == 1:
        new_ahrs_data = 0
        if raw_ahrs[1] == TYPE_AHRS and raw_ahrs[2] == AHRS_LEN:
            RollSpeed = Makeup_Data(raw_ahrs[7], raw_ahrs[8], raw_ahrs[9], raw_ahrs[10])       #横滚角速度
            PitchSpeed = Makeup_Data(raw_ahrs[11], raw_ahrs[12], raw_ahrs[13], raw_ahrs[14])   #俯仰角速度
            HeadingSpeed = Makeup_Data(raw_ahrs[15], raw_ahrs[16], raw_ahrs[17], raw_ahrs[18]) #偏航角速度

            Roll = Makeup_Data(raw_ahrs[19], raw_ahrs[20], raw_ahrs[21], raw_ahrs[22])    #横滚角
            Pitch = Makeup_Data(raw_ahrs[23], raw_ahrs[24], raw_ahrs[25], raw_ahrs[26])   #俯仰角
            Heading = Makeup_Data(raw_ahrs[27], raw_ahrs[28], raw_ahrs[29], raw_ahrs[30]) #偏航角
            print(f"Heading: {np.degrees(Heading)}")

            Q1 = Makeup_Data(raw_ahrs[31], raw_ahrs[32], raw_ahrs[33], raw_ahrs[34])     #四元数
            Q2 = Makeup_Data(raw_ahrs[35], raw_ahrs[36], raw_ahrs[37], raw_ahrs[38])
            Q3 = Makeup_Data(raw_ahrs[39], raw_ahrs[40], raw_ahrs[41], raw_ahrs[42])
            Q4 = Makeup_Data(raw_ahrs[43], raw_ahrs[44], raw_ahrs[45], raw_ahrs[46])

            Timestamp = Makeup_Timestamp(raw_ahrs[47], raw_ahrs[48], raw_ahrs[49], raw_ahrs[50]) #时间戳

            # print("RollSpeed:{0}, PitchSpeed:{1}, HeadingSpeed:{2}".format(RollSpeed, PitchSpeed, HeadingSpeed))
            # print("Roll:{0}, Pitch:{1}, Heading:{2}".format(Roll, Pitch, Heading))
            # print("Quaternion:{0}, {1}, {2}, {3}".format(Q1, Q2, Q3, Q4))
            # print("Timestamp:", Timestamp)

    if new_imu_data == 1:
        new_imu_data = 0
        if raw_imu[1] == TYPE_IMU and raw_imu[2] == IMU_LEN:
            Gyroscope_X = Makeup_Data(raw_imu[7], raw_imu[8], raw_imu[9], raw_imu[10]) # 角速度
            Gyroscope_Y = Makeup_Data(raw_imu[11], raw_imu[12], raw_imu[13], raw_imu[14])
            Gyroscope_Z = Makeup_Data(raw_imu[15], raw_imu[16], raw_imu[17], raw_imu[18])

            Accelerometer_X = Makeup_Data(raw_imu[19], raw_imu[20], raw_imu[21], raw_imu[22]) # 线加速度
            Accelerometer_Y = Makeup_Data(raw_imu[23], raw_imu[24], raw_imu[25], raw_imu[26])
            Accelerometer_Z = Makeup_Data(raw_imu[27], raw_imu[28], raw_imu[29], raw_imu[30])

            Magnetometer_X = Makeup_Data(raw_imu[31], raw_imu[32], raw_imu[33], raw_imu[34]) # 磁力计数据
            Magnetometer_Y = Makeup_Data(raw_imu[35], raw_imu[36], raw_imu[37], raw_imu[38])
            Magnetometer_Z = Makeup_Data(raw_imu[39], raw_imu[40], raw_imu[41], raw_imu[42])

            Timestamp = Makeup_Timestamp(raw_imu[55], raw_imu[56], raw_imu[57], raw_imu[58]); # 时间戳

            # print("Gyroscope:{0}, {1}, {2}".format(Gyroscope_X, Gyroscope_Y, Gyroscope_Z))
            # print("Accelerometer:{0}, {1}, {2}".format(Accelerometer_X, Accelerometer_Y, Accelerometer_Z))
            # print("Magnetometer:{0}, {1}, {2}".format(Magnetometer_X, Magnetometer_Y, Magnetometer_Z))
            # print("Timestamp:", Timestamp)
        
while True:
    count = uart.inWaiting()
    if (count):
        temp = uart.read(1)
        # print(temp)                 #打印原始数据b'xff'
        # print(Byte2Hex(temp))       #打印十六进制数
        # print(temp.decode('utf-8')) #打印字符串
        # uart.write(temp)
        Rx_imu_data(Byte2Hex(temp))

    Parse_imu_data()    
