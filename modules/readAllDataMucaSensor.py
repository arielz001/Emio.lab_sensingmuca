#!/usr/bin/env python

from threading import Thread
import time
import collections
from matplotlib import patches
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import numpy as np
import serial
from matplotlib.widgets import Button

# pip install scipy matplotlib pyserial
class serialData:
    def __init__(self, port="/dev/proxception", numValues=1, bytesPerValue=2, valueArrayDepth=1):
        self.port = port
        self.numValues = numValues
        self.bytesPerValue = bytesPerValue
        self.valueArrayDepth = valueArrayDepth
        self.rawData = bytearray(numValues * bytesPerValue)
        self.dataOffset = [0]*numValues
        self.dataOffsetWindow = 10
        self.dataType = None
        if bytesPerValue == 2:
            self.dataType = '<h'
        elif bytesPerValue == 4:
            self.dataType = '<f'
        self.data = []
        for i in range(numValues):
            self.data.append(collections.deque([0] * valueArrayDepth, maxlen=valueArrayDepth))
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.anim = None

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialDataOffest(self):
        dataOffsetSum = [0]*self.numValues
        for n in range(self.dataOffsetWindow):
            time.sleep(0.2)
            privateData = copy.deepcopy(self.rawData[:])
            for i in range(self.numValues):
                data = privateData[(i*self.bytesPerValue):(self.bytesPerValue + i*self.bytesPerValue)]
                try:
                    value,  = struct.unpack(self.dataType, data)
                    dataOffsetSum[i] += value
                except Exception:
                    pass
        for i in range(self.numValues):
            self.dataOffset[i] = dataOffsetSum[i]/self.dataOffsetWindow

    def getSerialData(self, frame, lines, lineValueText, lineLabel, im_heatmap):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)
        self.previousTimer = currentTimer
        privateData = copy.deepcopy(self.rawData[:])
        heatmapValues = [0] * numValues
        for i in range(self.numValues):
            data = privateData[(i*self.bytesPerValue):(self.bytesPerValue + i*self.bytesPerValue)]
            try:
                value,  = struct.unpack(self.dataType, data)
            except:
                continue
            self.data[i].append(value - self.dataOffset[i])
            lines[i].set_data(range(self.valueArrayDepth), self.data[i])
            heatmapValues[i] = (value - self.dataOffset[i])

        Matrix = np.reshape(heatmapValues[0:numValues-1], (tx_num, rx_num))
        Matrix = Matrix[::-1, :]

        im_heatmap.set_array(Matrix)
        ax1.set_ylabel('y Taxel')
        ax1.set_xlabel('x Taxel')
        print(f"matrix: {Matrix}")
        
        return [im_heatmap] + lines

    def backgroundThread(self):
        """Reads serial and updates raw data directly in memory."""
        try:
            ser = serial.Serial(port=self.port, baudrate=921600, timeout=3)
        except Exception as e:
            print(f"Failed to open muca port: {self.port}")
            print(e)
            self.isRun = False
            return

        ser.write(b"START\r\n")
        buffer = bytearray()
        expected_size = 506 

        while self.isRun:
            data = ser.read(1)
            if not data:
                continue

            buffer.extend(data)

            if len(buffer) >= 2 and buffer[-2:] == b"\r\n":
                if len(buffer) <= 2:
                    buffer.clear()
                    continue


                if len(buffer) == expected_size:
                    self.rawData = buffer[:-2]
                    self.isReceiving = True

                buffer.clear()

        ser.close()

    def close(self):
        self.isRun = False
        if self.thread:
            self.thread.join()


tx_num = 21
rx_num = 12
tx_num_ = 21
rx_num_ = 12

fig = plt.figure()
fig.set_figheight(5)
fig.set_figwidth(10)

ax1 = fig.add_subplot(1, 2, 1)

im1 = ax1.imshow(np.zeros((tx_num_, rx_num_)), cmap='jet', vmin=0, vmax=30, animated=True)

ax2 = fig.add_subplot(1, 2, 2)

numValues = 253

def reset(event):
    global skin
    skin.getSerialDataOffest()
    print("Reset done")


def main():
    global skin
    bytesPerValue = 2
    valueArrayDepth = 100
    
    skin = serialData(port="/dev/proxception", numValues=numValues, bytesPerValue=bytesPerValue, valueArrayDepth=valueArrayDepth)

    skin.readSerialStart()
    
    skin.getSerialDataOffest()
    pltInterval = 100

    ymin = -50
    ymax = 100

    ax2.set_xlim([0, valueArrayDepth])
    ax2.set_ylim([ymin, ymax])
    ax2.set_title('Skin Data')
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Sensor Value')

    lineLabel = [str(i) for i in range(numValues)]
    lines = []
    lineValueText = []

    for i in range(numValues):
        lines.append(ax2.plot([], [], [], [])[0])
        lineValueText.append(ax2.text(0.70, 0.90-i*0.05, '', transform=ax2.transAxes))
        
    button_ax = plt.axes([0.7, 0.1, 0.2, 0.075])

    reset_button = Button(button_ax, 'Reset')
    reset_button.on_clicked(reset)

    anim = animation.FuncAnimation(fig, skin.getSerialData, fargs=(lines, lineValueText, lineLabel, im1), interval=pltInterval, blit=True, cache_frame_data=False)
    
    plt.legend(loc="upper left")
    plt.show()
        
    skin.close()

if __name__ == '__main__':
    main()