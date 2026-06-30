# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# =====================================================================
# GENERAL EXPERIMENT CONFIGURATION
# =====================================================================
SENSOR_ROWS = 12
VISUAL_COLUMNS = 1
PATH_IDX = f"{os.path.dirname(os.path.abspath(__file__))}/modules/mucaData/IdxList.txt"
PATH_INTENSITIES = f"{os.path.dirname(os.path.abspath(__file__))}/modules/mucaData/IntensitiesList.txt"

# =====================================================================
# GRAPHICAL INTERFACE SETUP
# =====================================================================
fig, ax = plt.subplots(figsize=(4, 8))

ax.set_xlim(-0.5, VISUAL_COLUMNS - 0.5)
ax.set_ylim(-0.5, SENSOR_ROWS - 0.5)
ax.set_xticks(np.arange(VISUAL_COLUMNS))
ax.set_yticks(np.arange(SENSOR_ROWS))
ax.grid(True, color='gray', linestyle='--', alpha=0.6)
X, Y = np.meshgrid(np.arange(VISUAL_COLUMNS), np.arange(SENSOR_ROWS))
ax.scatter(X, Y, color='deepskyblue', s=60, alpha=0.7, edgecolors='blue')
yellowPoint, = ax.plot([], [], color='yellow', marker='o', markersize=14, 
                          markeredgecolor='orange', markeredgewidth=2, zorder=5)

# =====================================================================
# MATHEMATICAL INTERPOLATION 
# =====================================================================
def laboratorio_update(frame):
    if os.path.exists(PATH_IDX) and os.path.exists(PATH_INTENSITIES):
        try:
            if os.path.getsize(PATH_IDX) > 0 and os.path.getsize(PATH_INTENSITIES) > 0:
                idx_list = np.loadtxt(PATH_IDX, dtype=float)
                intensities = np.loadtxt(PATH_INTENSITIES)
                weightList = []
                if idx_list.size == 0 or intensities.size == 0:
                    yellowPoint.set_data([], [])
                    return yellowPoint,

                if idx_list.ndim == 1:
                    idx_list = np.array([idx_list])
                if intensities.ndim == 0:
                    intensities = np.array([intensities])

                # -----------------------------------------------------
                # TODO: Implement the center-of-mass algorithm below.
                # -----------------------------------------------------
                #
                # Hints:
                # 1. 'intensities' contains a 1D array of normalized floating intensities (Ii).
                # 2. 'idx_list' contains the corresponding coordinates.
                # 3. The specific active row index is stored inside: idx_list[i][0]
                # 
                # Target: Calculate the cumulative spatial sum into 'interpolated_coordinate'
                
                interpolated_coordinate = 0.0  
                # for i in range(len(intensities)): # <--- YOUR CODE GOES HERE (Replace this line with your loop)
                

                np.savetxt(f"{os.path.dirname(os.path.abspath(__file__))}/mucaData/WeightList.txt", weightList)
                # -----------------------------------------------------
                # END TODO
                # -----------------------------------------------------

                yellowPoint.set_data([0.0], [interpolated_coordinate])
                return yellowPoint,
                
        except Exception:
            pass 
    else:
        yellowPoint.set_data([], [])
        
    return yellowPoint,

# =====================================================================
# ASYNCHRONOUS GRAPHICS LOOP ACTIVATION
# =====================================================================
ani = animation.FuncAnimation(fig, laboratorio_update, interval=30, blit=False, cache_frame_data=False)
plt.show()