import serial
import threading
import matplotlib.pyplot as plt
import numpy as np
import struct

buffer = bytearray(256*4)
bufferReg = bytearray(256*4)

uartUpdateEvent = threading.Event()

def readUart(port, baudrate):
    global buffer
    try:
        ser = serial.Serial(port, baudrate)

        while True:
            if ser.in_waiting >= 1:
                if (ser.read(1)[0] == 255 and ser.read(1)[0] == 255):
                    buffer = ser.read(2)
                    frameLen = (buffer[1] << 8) + buffer[0]
                    buffer = ser.read(frameLen)
                    if (sum(buffer)%256 == ser.read(1)[0]):
                        dataPts = np.zeros((256,1))
                        for i in range(0, 256):
                            dataPt = struct.unpack('f', buffer[i*4 : (i+1)*4])[0]
                        #dataPt = bufferReg[i*2] + (bufferReg[i*2+1]<<8)
                            dataPts[i] = (dataPt * 2) / 512
                        plt.clf()
                        plt.plot(dataPts)
                        plt.ylim((0,0.5))
                        plt.pause(0.001)

    except serial.SerialException:
        print("Error: {}".format(e))

def printBuffer():
    global buffer
    while True:
        uartUpdateEvent.wait()
        bufferReg = buffer

        dataPts = np.zeros((256,1))
        for i in range(0, 256):
            dataPt = struct.unpack('f', bufferReg[i*4 : (i+1)*4])[0]
           #dataPt = bufferReg[i*2] + (bufferReg[i*2+1]<<8)
            dataPts[i] = (dataPt * 2) / 512
        plt.clf()
        plt.plot(dataPts)
        plt.ylim((0,3.3))
        plt.pause(0.001)

plt.ion()

uart_port = 'COM14'
baudrate = 115200

uart_thread = threading.Thread(target = readUart, args=(uart_port, baudrate))

uart_thread.start()

uart_thread.join()