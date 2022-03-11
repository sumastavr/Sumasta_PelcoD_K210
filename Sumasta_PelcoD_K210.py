import sensor, image, time, lcd
from fpioa_manager import fm
from machine import UART
from board import board_info
from fpioa_manager import fm

fm.register(board_info.PIN15, fm.fpioa.UART1_TX, force=True)
fm.register(board_info.PIN17, fm.fpioa.UART1_RX, force=True)

uart_PT = UART(UART.UART1, 19200, 8, 0, 0, timeout=1000, read_buf_len=16)

lcd.init(freq=15000000)

'''
sensor.reset()                      # Reset and initialize the sensor. It will                               # run automatically, call sensor.run(0) to stop
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
'''

clock = time.clock()                # Create a clock object to track the FPS.

PAN_CW_CODE=0x02
PAN_CCW_CODE=0x04
TILT_UP_CODE=0x08
TILT_DOWN_CODE=0x10

prevPan=0;
prevTilt=0;

CW=True
CCW=False

def getPanAngle(rawFrames):

    # frame format 0x 01 00 59 01 fe 59 ff 01 00 5b 8c 7c 64
    # 13 bytes, the first 0xff is omitted.

    if rawFrames[2] == 0x59:
        if rawFrames[3] < 0xFF and rawFrames[4] < 0xFF:
            pan=rawFrames[4]|rawFrames[3]<<8  # bit wise shift 2 bytes to 1 int
            #print ("Pan Angle : ",pan)
            if pan==0:
                return prevPan
            else:
                return pan
        else:
            return prevPan
    elif rawFrames [9] == 0x59:
        if rawFrames[10] < 0xFF and rawFrames[11] < 0xFF:
            pan=rawFrames[11]|rawFrames[10]<<8 # bit wise shift 2 bytes to 1 int
            #print ("Pan Angle : ",pan)
            if pan==0:
                return prevPan
            else:
                return pan
        else:
            return prevPan

def getTiltAngle(rawFrames):

    if rawFrames[2] == 0x5B:
        if rawFrames[3] < 0xFF and rawFrames[4] < 0xFF:
            tilt=rawFrames[4]|rawFrames[3]<<8 # bit wise shift 2 bytes to 1 int
            #print ("Tilt Angle : ",tilt)
            if tilt==0:
                return prevTilt
            else:
                return tilt
        else:
            return prevTilt

    elif rawFrames [9] == 0x5B:
        if rawFrames[10] < 0xFF and rawFrames[11] < 0xFF:
            tilt=rawFrames[11]|rawFrames[10]<<8 # bit wise shift 2 bytes to 1 int
            #print ("Tilt Angle : ",tilt)
            if tilt==0:
                return prevTilt
            else:
                return tilt
        else:
            return prevTilt

def movePan(direction,speed):

    # standard format
    # FF 01 00 04 01 00 06

    if speed > 0x40:
        speed=0x40

    if direction == True: # CW
        CRC=0x01+0x00+PAN_CW_CODE+speed
        uart_PT.write(bytes([0xFF,0x01,0x00,PAN_CW_CODE,speed,0x00,CRC]))

    elif direction == False: # CCW
        CRC=0x01+0x00+PAN_CCW_CODE+speed
        uart_PT.write(bytes([0xFF,0x01,0x00,PAN_CCW_CODE,speed,0x00,CRC]))


def moveTilt(direction,speed):

    # standard format
    # FF 01 00 08 00 20 29

    if speed > 0x40:
        speed=0x40

    if direction == True: # CW
        CRC=0x01+0x00+TILT_UP_CODE+speed
        uart_PT.write(bytes([0xFF,0x01,0x00,TILT_UP_CODE,0x00,speed,CRC]))

    elif direction == False: # CCW
        CRC=0x01+0x00+TILT_DOWN_CODE+speed
        uart_PT.write(bytes([0xFF,0x01,0x00,TILT_DOWN_CODE,0x00,speed,CRC]))

def movePanByAngle(direction,speed,angle):
    currentAngle=readCurrentPanAngle()
    targetAngle=0
    if direction:
        if currentAngle+angle>360:
            targetAngle=currentAngle+angle-360
            print("Target",targetAngle)
        else:
            targetAngle=currentAngle+angle
            print("Target",targetAngle)
        movePan(direction,speed)
    else:
        if currentAngle-angle<0:
            targetAngle=(currentAngle-angle)*-1
        else:
            targetAngle=currentAngle-angle
        movePan(direction,speed)

    time.sleep_ms(10)

    panning=True

    while(panning):

        if uart_PT.any():
            while uart_PT.any():
              read_data = uart_PT.read(16)
              int_val = int.from_bytes(read_data, "big")
              byte_list = []
              prefix=False
              counterFrame=0

              for i in range (len(read_data)):
                byte_now=(read_data[0+i:i+1])
                int_byte_now=int.from_bytes(byte_now, "big")

                if prefix == True:
                    byte_list.append(int_byte_now)
                    counterFrame+=1

                    if counterFrame == 13:
                        #print(byte_list)
                        panNow=getPanAngle(byte_list)
                        tiltNow=getTiltAngle(byte_list)

                        if validAngle(panNow) and validAngle(tiltNow):
                            panNow/=100
                            tiltNow/=100
                            print("Pan Angle: ",panNow,"   Tilt Angle: ",tiltNow)
                            prevPan=panNow
                            prevTilt=tiltNow

                            if direction:
                                if panNow>=targetAngle:
                                    stopMoving()
                                    panning=False;
                                    print("Exit True")
                            else:
                                if panNow<=targetAngle:
                                    stopMoving()
                                    panning=False;
                                    print("Exit False")

                            break;
                        else:
                            break;

                if int_byte_now == 0xff and prefix == False:

                    prefix=True

              break

    currentAngle=readCurrentPanAngle()
    print(currentAngle)

def stopMoving():
    uart_PT.write(bytes([0xFF,0x01,0x00,0x00,0x00,0x00,0x01]))

def readCurrentPanAngle():
    uart_PT.write(bytes([0xFF,0x01,0x00,0x51,0x00,0x00,0x52]))
    time.sleep_ms(10)
    if uart_PT.any():
        while uart_PT.any():
            read_data = uart_PT.read(16)
            byte_list = []
            for i in range (len(read_data)):
                byte_now=(read_data[0+i:i+1])
                int_byte_now=int.from_bytes(byte_now, "big")
                byte_list.append(int_byte_now)

            if byte_list[3] == 0x59:
                pan=(byte_list[5]|byte_list[4]<<8)/100  # bit wise shift 2 bytes to 1 int
                print ("Pan Angle Single : ",pan)
                return pan

def validAngle(axis):
    return isinstance(axis,int) and axis != 0


speeds=1;

movePanByAngle(CW,10,1)

#movePan(False,5)
#moveTilt(False,5)
stopMoving()


while(True):

    clock.tick()
    #img = sensor.snapshot()         # Take a picture and return the image.
    #3lcd.display(img)                # Display on LCD
    #print(clock.fps())              # Note: MaixPy's Cam runs about half as fast when connected
    #uart_PT.write('hello world')
    #uart_PT.write('\n')

    if uart_PT.any():
        while uart_PT.any():
          read_data = uart_PT.read(16)

          #print("recv = ", read_data)
          int_val = int.from_bytes(read_data, "big")
          #print(int_val)
          #print("recv hex = ", hex(int_val))
          #print("recv length = ", len(read_data))

          #uart_PT.write(read_data)

          byte_list = []
          prefix=False
          counterFrame=0

          for i in range (len(read_data)):
            byte_now=(read_data[0+i:i+1])
            int_byte_now=int.from_bytes(byte_now, "big")
            #print(i)
            #print(hex(int_byte_now))

            if prefix == True:
                byte_list.append(int_byte_now)
                counterFrame+=1
                if counterFrame == 13:
                    #print(byte_list)
                    panNow=getPanAngle(byte_list)
                    tiltNow=getTiltAngle(byte_list)

                    if validAngle(panNow) and validAngle(tiltNow):
                        print("Pan Angle: ",panNow/100,"   Tilt Angle: ",tiltNow/100)
                        prevPan=panNow
                        prevTilt=tiltNow
                        break;
                    else:
                        break;

            if int_byte_now == 0xff and prefix == False:
                #print("BINGGO")
                prefix=True

          #print(byte_list)
          break

    #time.sleep_ms(3000) # other event
    speeds=1
    if speeds>64:
        speeds=0
    #movePan(True,speeds)
    #stopMoving()
    #moveTilt(True,speeds)
    #readCurrentPanAngle()
