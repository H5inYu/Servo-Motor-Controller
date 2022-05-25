# Servo Motor Communication
# Author: Hsin-Yu

import sys
import serial
import time
# SerialPort setting
ser = serial.serial_for_url('COM16', baudrate = 9600, 
                            bytesize = 8, parity = 'E',
                            stopbits = 1, rtscts = False, 
                            xonxoff = False, do_not_open = True)

isClockwise = True
isDeg90 = True
DEG_90 = '00799000' # 00190000
DEG_180 = '00F32000' # 00320000
target_cw_90 = 'FF867000' # FFE70000
target_cw_180 = 'FFCE0000'
target_ccw_90 = '00190000'
isAvailable = True

def send_EOT():
    msg = '\x04'
    ser.write(msg.encode())
    print('send message: {}'.format(msg.encode()))

def error_processing():
    print('error processing')
    return "error processing"
    

def data_transmission(cmd, data_no, data, station_no = '0'):
    # 1. data make-up
    msg = '\x01' + station_no + cmd + '\x02' + data_no + data + '\x03'
    checksum = 0
    for c in msg[1:]:
        checksum += ord(c)
    # 2. concatenate checksum 
    msg += hex(checksum)[-2:].upper() # checksum using the last two hex captial character
    procedure_end = False
    wrong_data_acc = 0
    not_receive_acc = 0
    while not procedure_end:
        # 3. send data
        ser.write(msg.encode())
        print('send message: {}'.format(msg.encode()))
        # 4. data receive procedure
        # if not ser.in_waiting:
        #     time.sleep(0.1)
        if ser.in_waiting:
            raw_data = ser.read(ser.in_waiting)
            data = bytearray()
            for i in range(0, len(raw_data)):
                if raw_data[i] == 2:
                    data = bytearray()
                data.append(raw_data[i])
                if raw_data[i] == 3:
                    data.append(raw_data[i+1])
                    data.append(raw_data[i+2])
                    break
            receive_checksum = 0
            for c in data[1:]:
                receive_checksum += c
                if c == 3:
                    break

            print('ack: {}'.format(data))
            if not (data[-2:] == hex(receive_checksum)[-2:].upper().encode() and chr(data[2]) == 'A'):
                print('error from receiving wrong data')
                wrong_data_acc += 1
                if (wrong_data_acc >= 3):
                    error_processing()
                    break
            else:
                print('data transmission successful')
                procedure_end = True
                return data
        else:
            not_receive_acc += 1
            if (not_receive_acc >= 3):
                print('error from not receiving data')
                error_processing()
                break 
            else:
                send_EOT()
                time.sleep(0.1)

def Turn(target):
    # print('Initialize the servo amplifier, 500 rpm, 100 ms acc time')
    global isAvailable
    data_transmission('8B', '00', '0002') # select the test positioning operation
    data_transmission('A0', '10', '1388') # set default speed 500 rpm 01F4, (05DC:1500rpm), (0BB8:3000rpm)
    data_transmission('A0', '11', '000009C4') # set default acc time 5000 ms (00001388), (2500ms, 000009C4)
    data_transmission('A0', '20', DEG_90 if isDeg90 else DEG_180) # set distance # 90 deg = 1638400 (190000), 180 deg = 3276800 (320000)
    data_transmission('A0', '21', '0001' if isClockwise else '0000') # set pulse unit (2nd digit, command =0, encoder =1) and direction (4th digit, 0001
    data_transmission('92', '00', '00000007') # turn on SON, LSP and LSN
    data_transmission('01', '80', '') # read cumulative position
    data_transmission('81', '00', '1EA5') # reset cumulative position (it clears the last read status)
    data_transmission('A0', '40', '1EA5') # start
    counter = 0
    start = time.time()
    while counter < 50:
        pos = data_transmission('01', '80', '') # read cumulative positions
        if pos and pos[7:15] == bytearray(target.encode()):
                end = time.time()
                print("=======================================target arrived:", end-start)
                data_transmission('01', '80', '')
                data_transmission('01', '80', '')
                
                
                break
        time.sleep(0.1)
        counter += 1
    data_transmission('8B', '00', '0000') # cancel the test operation mode
    send_EOT()
    # isAvailable = True
    

def CW_90():
    global isClockwise
    global isDeg90
    global isAvailable
    isClockwise = True
    isDeg90 = True
    isAvailable = False
    Turn(target_cw_90)
    # return True

def CCW_90():
    global isClockwise
    global isDeg90
    global isAvailable
    isClockwise = False
    isDeg90 = True
    isAvailable = False
    Turn(target_ccw_90)
    # return True

def CW_180():
    global isClockwise
    global isDeg90
    global isAvailable
    isClockwise = True
    isDeg90 = False
    isAvailable = False
    Turn(target_cw_180)
    # return True

def welcome_back():
    try:
        ser.open()
    except serial.SerialException as e:
        return "Could not open Servo Motor Port"
    return "Hello from Servo Motor"
    
def closeSerialPort():
    ser.close()
    return "Close Servo Motor SerialPort"
if __name__ == '__main__':
    
    try:
        ser.open()
    except serial.SerialException as e:
        ser.close()
        # ser.open()
        print("======")
    CW_90()
    ser.close()
    # (b'\x020A0000FFE70000\x03FC') clockwise 90
    # (b'\x020A0000FFCE0000\x0308') clockwise 180
    # (b'\x020A000000190000\x03BE') counter-clockwise 90