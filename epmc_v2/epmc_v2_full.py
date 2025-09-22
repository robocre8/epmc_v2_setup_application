import serial
import struct


# Serial Protocol Command IDs -------------
START_BYTE = 0xAA
WRITE_VEL = 0x01
WRITE_PWM = 0x02
READ_POS = 0x03
READ_VEL = 0x04
READ_UVEL = 0x05
READ_TVEL = 0x06
SET_PPR = 0x07
GET_PPR = 0x08
SET_KP = 0x09
GET_KP = 0x0A
SET_KI = 0x0B
GET_KI = 0x0C
SET_KD = 0x0D
GET_KD = 0x0E
SET_RDIR = 0x0F
GET_RDIR = 0x10
SET_CUT_FREQ = 0x11
GET_CUT_FREQ = 0x12
SET_MAX_VEL = 0x13
GET_MAX_VEL = 0x14
SET_PID_MODE = 0x15
GET_PID_MODE = 0x16
SET_CMD_TIMEOUT = 0x17
GET_CMD_TIMEOUT = 0x18
SET_I2C_ADDR = 0x19
GET_I2C_ADDR = 0x1A
RESET_PARAMS = 0x1B
READ_MOTOR_DATA = 0x2A
#---------------------------------------------



class EPMC_V2_FULL:
    def __init__(self, port, baud=921600, timeOut=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeOut)
    
    #------------------------------------------------------------------------
    def send_packet_without_payload(self, cmd):
        length = 0
        packet = bytearray([START_BYTE, cmd, length])
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def send_packet_with_payload(self, cmd, payload_bytes):
        length = len(payload_bytes)
        packet = bytearray([START_BYTE, cmd, length]) + payload_bytes
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def read_packet1(self):
        payload = self.ser.read(4)
        a = struct.unpack('<f', payload)[0]  # little-endian float
        return a
    
    def read_packet3(self):
        payload = self.ser.read(12)
        a, b, c = struct.unpack('<fff', payload)  # little-endian float
        return a, b, c

    def read_packet4(self):
        payload = self.ser.read(16)
        a, b, c, d = struct.unpack('<ffff', payload)  # little-endian float
        return a, b, c, d
    
    def read_packet6(self):
        payload = self.ser.read(24)
        a, b, c, d, e, f = struct.unpack('<ffffff', payload)  # little-endian float
        return a, b, c, d, e, f
    
    def read_packet8(self):
        payload = self.ser.read(32)
        a, b, c, d, e, f, g, h = struct.unpack('<ffffffff', payload)  # little-endian float
        return a, b, c, d, e, f, g, h
    
    #---------------------------------------------------------------------

    def write_data1(self, cmd, pos, val):
        payload = struct.pack('<Bf', pos, val)
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data1(self, cmd, pos):
        payload = struct.pack('<Bf', pos, 0.0)  # big-endian
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val
    
    def write_data3(self, cmd, a, b, c):
        payload = struct.pack('<fff', a,b,c) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data3(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c = self.read_packet3()
        return a, b, c

    def write_data4(self, cmd, a, b, c, d):
        payload = struct.pack('<ffff', a,b,c,d) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data4(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d = self.read_packet4()
        return a, b, c, d
    
    def read_data6(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f = self.read_packet6()
        return a, b, c, d, e, f
    
    def read_data8(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f, g, h = self.read_packet8()
        return a, b, c, d, e, f, g, h
        
    #---------------------------------------------------------------------

    def writeSpeed(self, v0, v1, v2, v3):
        res = self.write_data4(WRITE_VEL, v0, v1, v2, v3)
        return int(res)
    
    def writePWM(self, v0, v1, v2, v3):
        res = self.write_data4(WRITE_PWM, v0, v1, v2, v3)
        return int(res)
    
    def readPos(self):
        pos0, pos1, pos2, pos3 = self.read_data4(READ_POS)
        return round(pos0,4), round(pos1,4), round(pos2,4), round(pos3,4)
    
    def readVel(self):
        v0, v1, v2, v3 = self.read_data4(READ_VEL)
        return round(v0,6), round(v1,6), round(v2,6), round(v3,6)
    
    def readUVel(self):
        v0, v1, v2, v3 = self.read_data4(READ_UVEL)
        return round(v0,6), round(v1,6), round(v2,6), round(v3,6)
    
    def readTVel(self):
        v0, v1, v2, v3 = self.read_data4(READ_TVEL)
        return round(v0,6), round(v1,6), round(v2,6), round(v3,6)
    
    def setCmdTimeout(self, timeout):
        res = self.write_data1(SET_CMD_TIMEOUT, 0, timeout)
        return int(res)
    
    def getCmdTimeout(self):
        timeout = self.read_data1(GET_CMD_TIMEOUT, 0)
        return int(timeout)
    
    def setPidMode(self, motor_no, mode):
        res = self.write_data1(SET_PID_MODE, motor_no, mode)
        return int(res)
    
    def getPidMode(self, motor_no):
        mode = self.read_data1(GET_CMD_TIMEOUT, motor_no)
        return int(mode)
    
    #---------------------------------------------------------------------

    def setPPR(self, motor_no, ppr):
        res = self.write_data1(SET_PPR, motor_no, ppr)
        return int(res)
    
    def getPPR(self, motor_no):
        ppr = self.read_data1(GET_PPR, motor_no)
        return ppr
    
    def setKp(self, motor_no, kp):
        res = self.write_data1(SET_KP, motor_no, kp)
        return int(res)
    
    def getKp(self, motor_no):
        kp = self.read_data1(GET_KP, motor_no)
        return kp
    
    def setKi(self, motor_no, ki):
        res = self.write_data1(SET_KI, motor_no, ki)
        return int(res)
    
    def getKi(self, motor_no):
        ki = self.read_data1(GET_KI, motor_no)
        return ki
    
    def setKd(self, motor_no, kd):
        res = self.write_data1(SET_KD, motor_no, kd)
        return int(res)
    
    def getKd(self, motor_no):
        kd = self.read_data1(GET_KD, motor_no)
        return kd
    
    def setRdir(self, motor_no, rdir):
        res = self.write_data1(SET_RDIR, motor_no, rdir)
        return int(res)
    
    def getRdir(self, motor_no):
        rdir = self.read_data1(GET_RDIR, motor_no)
        return int(rdir)
    
    def setCutOffFreq(self, motor_no, cutOffFreq):
        res = self.write_data1(SET_CUT_FREQ, motor_no, cutOffFreq)
        return int(res)
    
    def getCutOffFreq(self, motor_no):
        cutOffFreq = self.read_data1(GET_CUT_FREQ, motor_no)
        return cutOffFreq
    
    def setMaxVel(self, motor_no, maxVel):
        res = self.write_data1(SET_MAX_VEL, motor_no, maxVel)
        return int(res)
    
    def getMaxVel(self, motor_no):
        maxVel = self.read_data1(GET_MAX_VEL, motor_no)
        return maxVel
    
    def setI2cAddress(self, i2cAddress):
        res = self.write_data1(SET_I2C_ADDR, 0, i2cAddress)
        return int(res)
    
    def getI2cAddress(self):
        i2cAddress = self.read_data1(GET_I2C_ADDR, 0)
        return int(i2cAddress)
    
    def resetAllParams(self):
        res = self.write_data1(RESET_PARAMS, 0, 0.0)
        return int(res)

    ###################################################

    def readMotorData(self):
        pos0, pos1, pos2, pos3, v0, v1, v2, v3 = self.read_data8(READ_MOTOR_DATA)
        return round(pos0,4), round(pos1,4), round(pos2,4), round(pos3,4), round(v0,6), round(v1,6), round(v2,6), round(v3,6)