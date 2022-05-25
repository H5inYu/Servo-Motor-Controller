"""

This is a controller for MITSUBISHI ELECTRIC MR-JE series servo motor.
For more detail about the communication, please refer to the manual:
MR-JE-_A SERVO AMPLIFIER INSTRUCTION MANUAL

"""
import serial
import time

class ServoMotorController:

    def __init__(self, setting):
        self.com = setting['COM']
        self.ser = serial.serial_for_url(self.com, baudrate = 9600, bytesize = 8, 
                                        parity = 'E', stopbits = 1, rtscts = False, xonxoff = False, do_not_open = True)
        self.timeout = setting['timeout']
        self.speed = setting['speed']
        self.acc_time = setting['acc_time']

    def __enter__(self):
        self.ser.open()
        return self

    def __exit__(self, type, value, traceback):
        self.ser.close()

    def send_EOT(self):
        msg = '\x04'
        self.ser.write(msg.encode())
        print('send message: {}'.format(msg.encode()))

    def error_processing(self):
        '''
        You may implement your error processing method
        '''
        pass
        
    def data_transmission(self, cmd, data_no, data, station_no = '0'):
        '''
        The detail communication process please refer to the manual, 
        Chapter 12-12. COMMUNICATION FUNCTION (MITSUBISHI ELECTRIC GENERAL-PURPOSE AC SERVO PROTOCOL) p. 313
        '''
        # data make-up
        msg = '\x01' + station_no + cmd + '\x02' + data_no + data + '\x03'
        checksum = 0

        # Checksum calculation
        for c in msg[1:]:
            checksum += ord(c)

        # concatenate checksum, using the last two hex captial character
        msg += hex(checksum)[-2:].upper()

        # start communication process
        procedure_end = False
        wrong_data_acc = 0
        not_receive_acc = 0
        while not procedure_end:
            self.ser.write(msg.encode())
            print('send message: {}'.format(msg.encode()))

            # data receive procedure
            if self.ser.in_waiting:
                raw_data = self.ser.read(self.ser.in_waiting)
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
                    print('[Error] Receive wrong data')
                    wrong_data_acc += 1
                    if (wrong_data_acc >= 3):
                        self.error_processing()
                        break
                else:
                    print('data transmission successful')
                    procedure_end = True
                    return data
            else:
                not_receive_acc += 1
                if (not_receive_acc >= 3):
                    print('[Error] Not receive data')
                    self.error_processing()
                    break 
                else:
                    self.send_EOT()
                    time.sleep(0.1)

    def turn(self, target, clockwise):
        '''
        This is the data transmission flow for positioning operation.
        The detail communication process please refer to the manual, 
        Chapter 12-12. COMMUNICATION FUNCTION (MITSUBISHI ELECTRIC GENERAL-PURPOSE AC SERVO PROTOCOL) p.334
        '''
        distance = int(target/360*2**17)
        target_pos = distance if not clockwise else -distance
        self.data_transmission('8B', '00', '0002') # select the test positioning operation
        self.data_transmission('A0', '10', f'{self.speed:04x}'.upper()) # set default speed 500 rpm 01F4, (05DC:1500rpm), (0BB8:3000rpm)
        self.data_transmission('A0', '11', f'{self.acc_time:08x}'.upper()) # set default acc time 5000 ms (00001388), (2500ms, 000009C4)
        self.data_transmission('A0', '20', f'{distance:08x}'.upper()) # set distance # 90 deg = 1638400 (190000), 180 deg = 3276800 (320000)
        self.data_transmission('A0', '21', '0001' if clockwise else '0000') # set pulse unit (2nd digit, command =0, encoder =1) and direction (4th digit, 0001
        self.data_transmission('92', '00', '00000007') # turn on SON, LSP and LSN
        self.data_transmission('01', '80', '') # read cumulative position
        self.data_transmission('81', '00', '1EA5') # reset cumulative position (it clears the last read status)
        self.data_transmission('A0', '40', '1EA5') # start
        target_arrived = False
        counter = 0
        while not target_arrived and counter < self.timeout:
            pos = self.data_transmission('01', '80', '') # read cumulative positions
            if pos and pos[7:15] == f'{target_pos & (2**32-1):08x}'.upper():
                target_arrived = True
                print("Target arrived successfully.")
            time.sleep(0.1)
            counter += 1

        self.data_transmission('8B', '00', '0000') # cancel the test operation mode
        self.send_EOT()


if __name__ == '__main__':
    # setting
    setting = {
        'COM': 'COM2', # modify it to your serial port com number
        'speed': 500, # motor speed: [rpm]
        'acc_time': 5000, # acceleration time: [ms]
        'timeout': 50, # maximum waiting time for motor arrived the target position
    }
    with ServoMotorController(setting) as controller:
        while True:
            command = input("Enter command \"degree clockwise\" or type \"quit\" to close.\ndegree: 0~360, clockwise=1, counterclocksise=0")
            if command == 'quit':
                break
            degree, clockwise = map(int, command.split())
            controller.turn(degree, clockwise)