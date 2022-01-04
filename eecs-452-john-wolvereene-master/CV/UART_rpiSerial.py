'''
EECS 452 Raspberry Pi Serial
Team 8 : John Wolvereene

   Transmitted message to Rpi is in format: 
   Start 8 BITS = IMPOSSIBLE VALUE (DE)
   First message 8 bits =  right sensor
   Second message 8 bits = Front right sensor
   end 8 BITS = IMPOSSIBLE VALUE (FF)
	
   Received message from Rpi is in format:
   START 8 BITS = IMPOSSIBLE VALUE (DE)
   First message 8 bits = Distance calculated
   end 8 BITS = IMPOSSIBLE VALUE (FF)

 '''
start_byte = 0x1400
end_byte = 0xFFFF
import serial 
import time 
import UART_Quantization
import numpy as np
ser = serial.Serial("/dev/ttyS0", 9600, parity=serial.PARITY_NONE, 
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

    
    
     
def waitForByte():
    while ser.in_waiting == 0:
        time.sleep(.01) #wait 10 ms
    return


def send_message(message):
    sendVal = np.array([message])
    byte_array = bytearray()
    byte_array.append(0x14)
    byte_array.append(0x00)
    byte_array.append(UART_Quantization.fpq(sendVal,3)[0][0])
    byte_array.append(0xFF)
    byte_array.append(0xFF)
    ser.write(byte_array)
    return

def check_format():
    waitForByte()
    inbyteMSB = ser.read(1)
    if ord(inbyteMSB) != 0x14:
        return -1
    waitForByte()
    inbyteLSB = ser.read(1)
    inbyte = (ord(inbyteMSB) <<8) + ord(inbyteLSB)
    #print(inbyteMSB)
    if inbyte != start_byte:
        return -1
    byte_array = bytearray()
    waitForByte()
    byte_array.append(ord(ser.read(1)))
    waitForByte()
    byte_array.append(ord(ser.read(1)))
    waitForByte()
    byte_array.append(ord(ser.read(1)))
    waitForByte()
    byte_array.append(ord(ser.read(1)))
    waitForByte()
    inbyteMSB = ser.read(1)
    waitForByte()
    inbyteLSB = ser.read(1)
    inbyte = (ord(inbyteMSB) <<8) + ord(inbyteLSB)
    
    if inbyte != end_byte:
        return -1
    return combine_values(byte_array)


def combine_values(byte_array): 
  sensor1 =  byte_array[0]*256 + byte_array[1]
  sensor2 = byte_array[2]*256 + byte_array[3]
  return sensor1, sensor2
    
#We will put this into a class 
'''
ser= serial.Serial(port, baud_rate)
def writeResponse(self, packet):

    while True: 
        self.waitforbyte()
        start = self.ser.read(1)
        if(start == start_byte):
            self.waitforbyte()
            return_val = self.ser.read(1)
            if(end ==end_value):
                return return_val
        data_received = ser.read()
        waitforbyte()
        data_received += ser.read(new_data)
        print(data_received)

'''