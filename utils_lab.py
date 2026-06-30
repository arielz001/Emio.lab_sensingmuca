import numpy as np
def create_path(initial_marker_center_pos, shape = 'square', size_followpath = 45, total_points = 800):
    center_followpath_x = 0.0  
    center_followpath_y = initial_marker_center_pos[1]
    center_followpath_z = 0.0

    if shape == 'square':
        followpath_half = size_followpath / 2.0
        esquinas_cuadrado = [
            [center_followpath_x - followpath_half, center_followpath_y, center_followpath_z - followpath_half], # Index 0 (Inf. Izq)
            [center_followpath_x + followpath_half, center_followpath_y, center_followpath_z - followpath_half], # Index 1 (Inf. Der)
            [center_followpath_x + followpath_half, center_followpath_y, center_followpath_z + followpath_half], # Index 2 (Sup. Der)
            [center_followpath_x - followpath_half, center_followpath_y, center_followpath_z + followpath_half]  # Index 3 (Sup. Izq)
        ]

        indice_inicio = 2  
        esquinas_cuadrado = esquinas_cuadrado[indice_inicio:] + esquinas_cuadrado[:indice_inicio]
        
        path2follow = []
        points_per_side = total_points // 4
        for i in range(4):
            p_inicio = np.array(esquinas_cuadrado[i])
            p_fin = np.array(esquinas_cuadrado[(i + 1) % 4])
            
            for t in range(points_per_side):
                factor = t / points_per_side
                punto_intermedio = p_inicio + factor * (p_fin - p_inicio)
                path2follow.append(punto_intermedio.tolist())
        path2follow.append(path2follow[0])

    elif shape == 'line':
        followpath_half = size_followpath / 2.0
        path2follow = []
        centro = np.array([center_followpath_x, center_followpath_y, center_followpath_z])
        points_per_segment = total_points // 4
        p1 = np.array([-followpath_half, 0.0, 0.0])
        p2 = np.array([followpath_half, 0.0, 0.0])
        angulo = np.radians(90) 
        c, s = np.cos(angulo), np.sin(angulo)
        R = np.array([
            [c,  0, s],
            [0,  1, 0],
            [-s, 0, c]
        ])
        
        p1_rotado = np.dot(R, p1) + centro
        p2_rotado = np.dot(R, p2) + centro
        
        # --- WAY 1: CENTER TO EXTREME 1 ---
        for t in range(points_per_segment):
            factor = t / points_per_segment
            punto = centro + factor * (p1_rotado - centro)
            path2follow.append(punto.tolist())
            
        # --- WAY 2: FROM EXTREME 1 TO CENTER---
        for t in range(points_per_segment):
            factor = t / points_per_segment
            punto = p1_rotado + factor * (centro - p1_rotado)
            path2follow.append(punto.tolist())
            
        # --- WAY 3: FROM CENTER TO EXTREME 2 ---
        for t in range(points_per_segment):
            factor = t / points_per_segment
            punto = centro + factor * (p2_rotado - centro)
            path2follow.append(punto.tolist())
            
        # --- WAY 4: FROM EXTREME 2 TO CENTER ---
        for t in range(points_per_segment):
            factor = t / points_per_segment
            punto = p2_rotado + factor * (centro - p2_rotado)
            path2follow.append(punto.tolist())

        path2follow.append(path2follow[0])

    elif shape == 'circle':
        radio = size_followpath / 2.0  
        path2follow = []
        angulo_inicio = np.pi / 4 

        for t in range(total_points):
            factor = t / total_points
            angulo = angulo_inicio + (2 * np.pi * factor)
            
            x = center_followpath_x + radio * np.cos(angulo)
            y = center_followpath_y  
            z = center_followpath_z + radio * np.sin(angulo)
            
            path2follow.append([float(x), float(y), float(z)])

        path2follow.append(path2follow[0])

    elif shape == 'point':
        punto_centro = [float(center_followpath_x), float(center_followpath_y), float(center_followpath_z)]
        path2follow = [punto_centro]

        path2follow.append(path2follow[0]) 

    return path2follow


def generate_interpolation(points, quantityinterp=10, offset=-6):
    interpolated_points = []

    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        t = np.linspace(0, 1, quantityinterp + 2)[:-1]

        for alpha in t:
            interpolated_points.append( (1 - alpha) * p1 + alpha * p2)

    interpolated_points.append(points[-1])


    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]

        t = np.linspace(0, 1, quantityinterp + 2)[:-1]

        for alpha in t:
            point2add =  (1 - alpha) * p1 + alpha * p2
            point2add[2] += offset
            interpolated_points.append(point2add)

    interpolated_points.append(points[-1])
                               
    return np.array(interpolated_points)
