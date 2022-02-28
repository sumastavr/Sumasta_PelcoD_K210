import sensor, image, time, lcd
from fpioa_manager import fm
from machine import UART
from board import board_info
from fpioa_manager import fm

fm.register(board_info.PIN15, fm.fpioa.UART1_TX, force=True)
fm.register(board_info.PIN17, fm.fpioa.UART1_RX, force=True)

uart_PT = UART(UART.UART1, 19200, 8, 0, 0, timeout=1000, read_buf_len=128)

lcd.init(freq=15000000)

'''
sensor.reset()                      # Reset and initialize the sensor. It will                               # run automatically, call sensor.run(0) to stop
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
'''

clock = time.clock()                # Create a clock object to track the FPS.

PAN_CW_CODE=0x04
PAN_CCW_CODE=0x02
TILT_UP_CODE=0x08
TILT_DOWN_CODE=0x10


def getPanAngle(rawFrames):

    # frame format 0x 01 00 59 01 fe 59 ff 01 00 5b 8c 7c 64
    # 13 bytes, the first 0xff is omitted.

    if rawFrames[2] == 0x59:
        pan=rawFrames[4]|rawFrames[3]<<8  # bit wise shift 2 bytes to 1 int
        #print ("Pan Angle : ",pan)
        return pan
    elif rawFrames [9] == 0x59:
        pan=rawFrames[11]|rawFrames[10]<<8 # bit wise shift 2 bytes to 1 int
        #print ("Pan Angle : ",pan)
        return pan

def getTiltAngle(rawFrames):

    if rawFrames[2] == 0x5B:
        tilt=rawFrames[4]|rawFrames[3]<<8 # bit wise shift 2 bytes to 1 int
        #print ("Tilt Angle : ",tilt)
        return tilt
    elif rawFrames [9] == 0x5B:
        tilt=rawFrames[11]|rawFrames[10]<<8 # bit wise shift 2 bytes to 1 int
        #print ("Tilt Angle : ",tilt)
        return tilt

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
        uart_PT.write(bytes([0xFF,0x01,0x00,TILT_UP_CODE,0x00,speed,CRC]))

def stopMoving():
    uart_PT.write(bytes([0xFF,0x01,0x00,0x00,0x00,0x00,0x01]))

def readCurrentPanAngle():
    uart_PT.write(bytes([0xFF,0x01,0x00,0x51,0x00,0x00,0x52]))
    time.sleep_ms(1000)
    if uart_PT.any():
        while uart_PT.any():
            read_data = uart_PT.readline()
            byte_list = []
            for i in range (len(read_data)):
                byte_now=(read_data[0+i:i+1])
                int_byte_now=int.from_bytes(byte_now, "big")
                byte_list.append(int_byte_now)
            if byte_list[3] == 0x59:
                pan=byte_list[5]|byte_list[4]<<8  # bit wise shift 2 bytes to 1 int
                print ("Pan Angle Single : ",pan)
                return pan


speeds=0;

while(True):
    clock.tick()                    # Update the FPS clock.
    #img = sensor.snapshot()         # Take a picture and return the image.
    #3lcd.display(img)                # Display on LCD
    #print(clock.fps())              # Note: MaixPy's Cam runs about half as fast when connected
    #uart_PT.write('hello world')
    #uart_PT.write('\n')

    if uart_PT.any():
        while uart_PT.any():
          read_data = uart_PT.readline()

          #print("recv = ", read_data)
          int_val = int.from_bytes(read_data, "big")
          #print(int_val)
          #print("recv hex = ", hex(int_val))
          #print("recv length = ", len(read_data))

          uart_PT.write(read_data)

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
                    print("Pan Angle: ",getPanAngle(byte_list)/100,"   Tilt Angle: ",getTiltAngle(byte_list)/100)

                    break;

            if int_byte_now == 0xff and prefix == False:
                #print("BINGGO")
                prefix=True


          #print(byte_list)
          break

    time.sleep_ms(3000) # other event
    speeds+=1
    if speeds>64:
        speeds=0
    movePan(True,speeds)
    moveTilt(True,speeds)
    #readCurrentPanAngle()
