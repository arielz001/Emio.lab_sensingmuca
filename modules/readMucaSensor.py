# -*- coding: utf-8 -*-
#!/usr/bin/env python

import serial
import threading
import time
import collections
import struct
import copy
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from scipy.signal import find_peaks

np.set_printoptions(suppress=True, precision=3)
os.makedirs(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData", exist_ok=True)
class serialData:
    def __init__(self, port="/dev/proxception", numValues=1, bytesPerValue=2, valueArrayDepth=1):
        self.port = port
        self.numValues = numValues
        self.bytesPerValue = bytesPerValue
        self.valueArrayDepth = valueArrayDepth
        self.rawData = bytearray(numValues * bytesPerValue)
        self.dataOffset = [0] * numValues
        self.dataOffsetWindow = 10
        self.dataType = '<h' if bytesPerValue == 2 else '<f'
        
        self.data = []
        for i in range(numValues):
            self.data.append(collections.deque([0] * valueArrayDepth, maxlen=valueArrayDepth))
            
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.Matrix = []

    def readSerialStart(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self.backgroundThread)
            self.thread.start()
            while not self.isReceiving:
                time.sleep(0.1)

    def getSerialDataOffset(self):
        dataOffsetSum = [0] * self.numValues
        for n in range(self.dataOffsetWindow):
            privateData = copy.deepcopy(self.rawData[:])
            for i in range(self.numValues):
                data = privateData[(i * self.bytesPerValue):(self.bytesPerValue + i * self.bytesPerValue)]
                try:
                    value, = struct.unpack(self.dataType, data)
                    dataOffsetSum[i] += value
                except Exception:
                    pass
        for i in range(self.numValues):
            self.dataOffset[i] = dataOffsetSum[i] / self.dataOffsetWindow

    def getSerialData(self):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)
        self.previousTimer = currentTimer
        
        privateData = copy.deepcopy(self.rawData[:])
        heatmapValues = [0] * self.numValues
        
        for i in range(self.numValues):
            data = privateData[(i * self.bytesPerValue):(self.bytesPerValue + i * self.bytesPerValue)]
            try:
                value, = struct.unpack(self.dataType, data)
            except Exception:
                continue
            self.data[i].append(value - self.dataOffset[i])
            heatmapValues[i] = (value - self.dataOffset[i])

        self.Matrix = np.reshape(heatmapValues[0:self.numValues], (tx_num, rx_num))

    def backgroundThread(self):
        """Reads directly from the serial port without intermediate files"""
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=921600,
                timeout=3
            )
        except Exception as e:
            print(f"Failed to open muca port: {self.port}")
            print(e)
            self.isRun = False
            return

        ser.write(b"START\r\n")
        buffer = bytearray()
        expected_size = (self.numValues * self.bytesPerValue) + 2

        while self.isRun:
            data = ser.read(1)
            if not data:
                continue

            buffer.extend(data)

            # Check for \r\n packet termination
            if len(buffer) >= 2 and buffer[-2:] == b"\r\n":
                if len(buffer) == expected_size:
                    self.rawData = buffer[:-2]  # Remove trailing \r\n
                    self.isReceiving = True
                
                buffer.clear()
        
        ser.close()

    def close(self):
        self.isRun = False
        if self.thread:
            self.thread.join()


def getIntensities(IdxList, Matrix):
    IntensitiesList = []
    for x, y in IdxList:
        if 0 <= y < len(Matrix):
            Intensity = Matrix[y, 0]
        else:
            Intensity = 0

        if Intensity < 0:
            Intensity = 0
        IntensitiesList.append(Intensity)

    IntensitiesList = np.array(IntensitiesList)
    if np.sum(IntensitiesList) == 0:
        return np.array([0.0, 0.0])

    IntensitiesList = IntensitiesList #/ np.sum(IntensitiesList)
    for i in range(len(IdxList)):
        IdxList[i][0], IdxList[i][1] = IdxList[i][1], IdxList[i][0]

    np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/IntensitiesList.txt", IntensitiesList)
    np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/IdxList.txt", IdxList, fmt='%i')
    
    IntensitiesCoords = np.sum(IdxList * IntensitiesList[:, None], axis=0)
    return IntensitiesCoords


def getNeighborIdxs(IdxY, TouchResolution): 
    IdxList = []
    i = 0
    for j in range(IdxY - 1, IdxY + 2):
        if 0 <= j < TouchResolution:
            IdxList.append([i, j])
    return IdxList


def getTouchCoords(Matrix):
    if Matrix.ndim == 1:
        LinearIdxMax = np.argmax(Matrix)
        Value = Matrix[LinearIdxMax]
        IdxMaxY = LinearIdxMax
    else:
        IdxMaxY, IdxMaxX = np.unravel_index(np.argmax(Matrix), Matrix.shape)
        Value = Matrix[IdxMaxY, IdxMaxX]
    if Value < TOUCHTHRESHOLD / 2:
        return None, None
    return IdxMaxY, Value


# --- GLOBALS & CONFIGURATION ---
last_valid_idx = None
COLUMNA = 1
FILAS = 12
TOUCHTHRESHOLD = 7
counter_without_touch = 0

tx_num, rx_num = 21, 12
numValues = 252

MucaCols, MucaRows = 12, 21
ImgRows, ImgCols = 12, 2

indices_sensor = np.arange(FILAS)

# --- SENSOR INITIALIZATION ---
skin = serialData(port="/dev/proxception", numValues=numValues, bytesPerValue=2, valueArrayDepth=100)
skin.readSerialStart()

for _ in range(10):
    skin.getSerialDataOffset()

# --- GUI SETUP ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 6))
fig.canvas.manager.set_window_title('MuCa Sensing')
fig.suptitle('Capacitive Sensor Data & Peaks Profile', fontsize=16)

im = ax1.imshow(np.empty((ImgRows, ImgCols)),
                cmap='gray',
                vmin=0,
                vmax=TOUCHTHRESHOLD,
                animated=True)
ax1.set_xlim(-0.5, ImgCols - 0.5)
ax1.set_ylim(-0.5, ImgRows - 0.5)
ax1.set_title("Sensor States")

linea_peaks, = ax2.plot(indices_sensor, np.zeros(FILAS), color='blue', linewidth=2, marker='o')
ax2.set_xlim(-0.5, FILAS - 0.5)
ax2.set_ylim(-5, 160)
ax2.set_xticks(indices_sensor)
ax2.set_title("Signal")
ax2.set_xlabel("Row")
ax2.set_ylabel("Intensity")
ax2.grid(True)

Back = np.zeros((MucaCols, MucaRows))
Iteration = 0
WaitFrames = 20


# --- MAIN ANIMATION LOOP ---
def updatefig(*args):
    global Back, Iteration, skin, counter_without_touch, last_valid_idx

    skin.getSerialData()
    data = skin.Matrix

    try:
        Line = data.transpose()
        Floats = Line.astype(float)
        Matrix = Floats - Back
        
        if Iteration == WaitFrames:
            Back = Matrix
        
        # Calculate mean row values across selected columns
        CutMatrixFull = Matrix[:FILAS, :COLUMNA + 1]
        CutMatrixFull = np.mean(CutMatrixFull, axis=1, keepdims=True)
        senal_1d = CutMatrixFull.flatten()

        # Scipy peak detection
        peaks, propiedades = find_peaks(senal_1d, height=TOUCHTHRESHOLD)
        max_index = None

        if len(peaks) > 0:
            max_index = peaks[-1] 
            idx_inferior = max(0, max_index - 1)
            idx_superior = min(FILAS - 1, max_index + 1)
            matriz_aislada = np.zeros_like(CutMatrixFull)
            matriz_aislada[idx_inferior : idx_superior + 1] = CutMatrixFull[idx_inferior : idx_superior + 1]
            CutMatrixFull = matriz_aislada
            last_valid_idx = max_index
        else:
            CutMatrixFull = np.zeros_like(CutMatrixFull)
            last_valid_idx = None

        IdxMaxY, Value = getTouchCoords(CutMatrixFull)

        try:
            if max_index is None:
                raise ValueError("No peaks found")

            NeighborIdxList = getNeighborIdxs(IdxMaxY, TouchResolution=FILAS)
            TouchThreshold = TOUCHTHRESHOLD

            if Value > TouchThreshold:
                IntensitiesCoords = getIntensities(NeighborIdxList, CutMatrixFull)
            else:
                IntensitiesCoords = [-1]
                last_valid_idx = None 

            np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/Coord.txt", IntensitiesCoords)
            
            # Update GUI plots
            im.set_array(CutMatrixFull)
            datos_linea = CutMatrixFull.flatten()
            linea_peaks.set_ydata(datos_linea)
            
        except Exception: 
            # Clear UI data if touch is lost
            last_valid_idx = None  
            np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/Coord.txt", [])
            np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/IdxList.txt", [])
            np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/IntensitiesList.txt", [])
            im.set_array(np.zeros((ImgRows, ImgCols)))
            linea_peaks.set_ydata(np.zeros(FILAS)) 
            counter_without_touch = 0

    except Exception as ex:
        print(ex)

    Iteration += 1
    return im, linea_peaks,


# --- GUI EVENT HANDLERS & BUTTONS ---
def reset_program(event):
    print("\n--- [REINIT PROGRAM] ---\n")
    skin.close()  
    plt.close('all')
    os.execv(sys.executable, ['python'] + sys.argv)

anim_running = True

def toggle_animation(event):
    global anim_running
    if anim_running:
        ani.pause()  
        play_pause_button.label.set_text('Play')  
        anim_running = False
        print("\n--- [ANIMATION PAUSED] ---\n")
    else:
        ani.resume()  
        play_pause_button.label.set_text('Pause')  
        anim_running = True
        print("\n--- [ANIMATION RESUMED] ---\n")
    fig.canvas.draw_idle()


plt.subplots_adjust(bottom=0.25)

# Reset Button
button_ax = plt.axes([0.35, 0.05, 0.12, 0.06])
reset_button = Button(button_ax, 'Reset')
reset_button.on_clicked(reset_program)

# Play/Pause Button
pause_ax = plt.axes([0.52, 0.05, 0.12, 0.06])
play_pause_button = Button(pause_ax, 'Pause')
play_pause_button.on_clicked(toggle_animation)

# --- START APPLICATION ---
ani = animation.FuncAnimation(fig, updatefig, interval=100, blit=True)

try:
    plt.show()
finally:
    skin.close() 