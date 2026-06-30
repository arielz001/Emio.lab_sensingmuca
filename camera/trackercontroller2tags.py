import Sofa
import numpy
import numpy as np
from math import pi, cos
from scipy import signal
from operator import itemgetter
# import emioapi
# print(f"\n\n{emioapi.__file__=}\n\n")

from emioapi import EmioAPI
from emiocameraTag import EmioCamera

import Sofa.ImGui as MyGui
import parameters as params

import math

CLIP_DIST = np.array([0.4, 0.1, 0.4])


class DotTracker(Sofa.Core.Controller):

    def __init__(self, root, nb_tracker=4,
                 show_video_feed=True,
                 compute_point_cloud=False,
                 track_colors=True,
                 scale=1,
                 translation=[0, 0, 0],
                 rotation=[0, 0, 0],
                 filter_alpha=0.5,
                 *args, **kwargs):
        

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = root
        self.nb_tracker = nb_tracker
        self.compute_point_cloud = compute_point_cloud
        self.track_colors = track_colors

        # Filter
        self.filter_alpha = filter_alpha
        self.trackersf1 = np.zeros((nb_tracker,3))
        self.trackersf2 = np.zeros((nb_tracker,3))

        # --- CONFIGURACIÓN DE COLOR (NARANJA) ---
        orange_params = {
            'hue_l': 0,    'hue_h': 22,   
            'sat_l': 130,  'sat_h': 255,  
            'value_l': 160, 'value_h': 255, 
            'erosion_size': 0,
            'area': 50
        }

        # Inicializamos la cámara con tus parámetros
        self.tracker = EmioCamera(show=True, 
                                  compute_point_cloud=self.compute_point_cloud,
                                  parameter=orange_params)
        
        self.node = root.addChild("DepthCamera")

        if self.track_colors:
            coord_pt = [0.0, 0, 0] * nb_tracker

            self.mo = self.node.addObject("MechanicalObject", name="Trackers", template="Vec3d",
                                      position=coord_pt.copy(), showObject=True,
                                      showObjectScale=5, drawMode=1, showColor=[0, 1., 0, 1],
                                      rotation=rotation, translation=translation)

            self.node.addData(name="rotationy", type="float", value=rotation[1])
            self.node.addData(name="translationx", type="float", value=translation[0])
            self.node.addData(name="translationz", type="float", value=translation[2])
            self.node.addData(name="translationy", type="float", value=translation[1])
            MyGui.MyRobotWindow.addSettingInGroup("Orientation", self.node.rotationy, -200, 200, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation x", self.node.translationx, -100, 200, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation y", self.node.translationy, -100, 100, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation z", self.node.translationz, -100, 0, "Camera")

        if self.compute_point_cloud:
            coord_pt = np.clip(self.tracker.point_cloud, -CLIP_DIST, CLIP_DIST)

            self.mo_point_cloud = self.node.addObject("MechanicalObject", name="pointCloud", template="Vec3d",
                                                  position=coord_pt, showObject=True, showObjectScale=1,
                                                  showColor=[0, 255, 0, 0.1],
                                                  rotation=rotation, translation=translation,
                                                  scale=scale*1e3)
        
        # --- added: variables to wait for start ---
        self.start_delay = 100  # wait 100 steps (approx 2-4 seconds)
        self.current_step = 0
        # -----------------------------------------------------


    def onAnimateBeginEvent(self, _):
        # --- NUEVO: ESPERAR ANTES DE INICIAR CAMARA ---
        # Esto permite que los motores se conecten primero sin conflictos USB
        if self.current_step < self.start_delay:
            self.current_step += 1
            if self.current_step == 1:
                print(f"[DotTracker] Esperando {self.start_delay} pasos para iniciar cámara y proteger motores...")
            return  # Salimos de la función sin hacer nada
        # ----------------------------------------------

        # --- LOGICA DE INICIO SEGURA (MODIFICADA) ---
        if not self.tracker.is_running:
            try:
                # 1. Buscar cámaras
                cameras = EmioAPI.listCameraDevices()
                print(f"\n\n{cameras=}\n\n")
                # print(f"\n\n{cameras=}\n\n")
                if not cameras:
                    # Silenciamos el error repetitivo, solo imprimimos una vez si fuera necesario
                    return

                # 2. Decidir índice (Sincronizar con motor SI existe, si no, usar 0)
                device_index = 0
                motor_ctrl = self.root.getChild("MotorController") 
                
                if motor_ctrl is not None:
                    # Solo accedemos si existe y tiene la propiedad emiomotors conectada
                    if hasattr(motor_ctrl, 'emiomotors') and motor_ctrl.emiomotors.is_connected:
                        device_index = motor_ctrl.emiomotors.device_index
                
                # 3. Abrir cámara
                if len(cameras) > device_index:
                    self.tracker.open(cameras[device_index])
                    print(f"[DotTracker] Cámara iniciada (Índice {device_index}) tras espera.")
                else:
                    # Índice fuera de rango
                    return

            except Exception as e:
                print(f"[DotTracker] Error iniciando cámara: {e}")
                return
        # ------------------------------------------------------------

        # Lógica visual (Mover la cajita verde/roja)
        # if self.mo: 
        #     self.mo.rotation.value = [self.mo.rotation.value[0], self.node.rotationy.value, self.mo.rotation.value[2]]
        #     t = cos(pi / 4.) * self.node.translationx.value
        #     self.mo.translation.value = [-t, self.node.translationy.value, -t]

        if self.mo:
            self.mo.rotation.value = [
                self.mo.rotation.value[0],
                self.node.rotationy.value,
                self.mo.rotation.value[2]
            ]

            tx = self.node.translationx.value
            tz = self.node.translationz.value

            t1 = math.cos(math.pi/4) * tx
            t2 = math.cos(math.pi/4) * tz

            self.mo.translation.value = [
                -(t1 - t2),
                self.node.translationy.value,
                -(t1 + t2)
            ]

        # Lógica de Rastreo (Tracking)
        alpha = self.filter_alpha
        if self.track_colors and self.tracker.is_running:
            self.tracker.update()
            # Filtro de profundidad (0 a 400mm)
            trackers_pos = list(filter(lambda x: (x[2]>0 and x[2]<400), self.tracker.trackers_pos))
            trackers_pos.sort(key=itemgetter(2))
            
            coord = trackers_pos
            # print(f"\n\n{coord=}\n\n")
            if len(coord) >= self.nb_tracker:
                self.trackersf1 = alpha*self.trackersf1 + (1.-alpha)*np.array(np.array(coord[0:self.nb_tracker][0:3]))
                self.trackersf2 = alpha*self.trackersf2 + (1.-alpha)*np.array(self.trackersf1)
                self.mo.position.value = self.trackersf2.copy()
                self.mo.reinit()

        if self.compute_point_cloud and self.tracker.is_running:
            coord_pt = np.clip(self.tracker.point_cloud, -CLIP_DIST, CLIP_DIST)
            self.mo_point_cloud.position.value = coord_pt.copy()
            self.mo_point_cloud.reinit()





class ApriltagTracker(Sofa.Core.Controller):

    def __init__(self, root, nb_tracker=4,
                 show_video_feed=True,
                 compute_point_cloud=False,
                 track_tag=True,
                 scale=1,
                 translation=[0, 0, 0],
                 rotation=[0, 0, 0],
                 filter_alpha=0.5,
                 center_marker_position=[0, 0, 0],
                 leg_marker_position=[0, 0, 0],
                 show_spheres = True,
                #  initial_pose_endeffector=0,
                 *args, **kwargs):
        

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = root
        self.nb_tracker = nb_tracker # the value here is 1 
        self.compute_point_cloud = compute_point_cloud
        self.track_tag = track_tag

        # Filter
        self.filter_alpha = filter_alpha
        self.trackersf1CenterTag = np.zeros((nb_tracker,3))
        self.trackersf2CenterTag = np.zeros((nb_tracker,3))

        self.trackersf1Leg = np.zeros((nb_tracker,3))
        self.trackersf2Leg = np.zeros((nb_tracker,3))

        print(f"\n\nNB_TRACKER: {nb_tracker}\n\n")
        # Inicializamos la cámara con tus parámetros
        self.tracker = EmioCamera(show=True, 
                                  compute_point_cloud=self.compute_point_cloud)
        
        self.node = root.addChild("DepthCamera")

        # i need to read the center marker
        # translation = [0, -140, 0]
        translation_center = center_marker_position

        # leg_marker_position[0] -= 14.25

        # middle marker
        # leg_marker_position[0] -= 14
        # leg_marker_position[1] += 2
        
        # middle down marker (2nd place from down to top)
        leg_marker_position[0] -= 27.5
        leg_marker_position[1] += 8.75
        # leg_marker_position[2] -= 1
        translation_leg = leg_marker_position
        # rotation = [0, 0, 0]

        if self.track_tag:
            coord_pt_center_tag = [0.0, 0, 0] * nb_tracker 
            
            coord_pt_leg_tag = [0.0, 0, 0] * nb_tracker 

            self.mo_tracker_center = self.node.addObject("MechanicalObject", name="TrackerCenterTag", template="Vec3d",
                                      position=coord_pt_center_tag.copy(), showObject=show_spheres,
                                      showObjectScale=5, drawMode=1, showColor=[0, 1., 0, 1],
                                      rotation=rotation, translation=translation_center)

            self.mo_tracker_leg = self.node.addObject("MechanicalObject", name="TrackerLegTag", template="Vec3d",
                            position=coord_pt_leg_tag.copy(), showObject=show_spheres,
                            showObjectScale=5, drawMode=1, showColor=[0, 0., 1, 1], # blue color
                            rotation=rotation, translation=translation_leg)
        


            self.node.addData(name="rotationy", type="float", value=rotation[1])
            self.node.addData(name="translationx", type="float", value=translation_center[0])
            self.node.addData(name="translationy", type="float", value=translation_center[1])
            self.node.addData(name="translationz", type="float", value=translation_center[2])

            MyGui.MyRobotWindow.addSettingInGroup("Orientation", self.node.rotationy, -200, 200, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation x", self.node.translationx, -100, 200, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation y", self.node.translationy, -150, 150, "Camera")
            MyGui.MyRobotWindow.addSettingInGroup("Translation z", self.node.translationz, -100, 100, "Camera")

        # --- added: variables to wait for start ---
        self.start_delay = 100  # wait 100 steps (approx 2-4 seconds)
        self.current_step = 0
        # -----------------------------------------------------
        self.started_pose_camera = False
        self.initial_pose_tag = 0
        self.counter =  0

        # to avoid errors
        self.last_valid_relative_center = np.array([0.0, 0.0, 0.0])
        self.last_valid_relative_leg = np.array([0.0, 0.0, 0.0])

    def onAnimateBeginEvent(self, _):
        # --- NUEVO: ESPERAR ANTES DE INICIAR CAMARA ---
        # Esto permite que los motores se conecten primero sin conflictos USB
        if self.current_step < self.start_delay:
            self.current_step += 1
            if self.current_step == 1:
                print(f"[DotTracker] Esperando {self.start_delay} pasos para iniciar cámara y proteger motores...")
            return  # Salimos de la función sin hacer nada
        # ----------------------------------------------

        # --- LOGICA DE INICIO SEGURA (MODIFICADA) ---
        if not self.tracker.is_running:
            try:
                # 1. Buscar cámaras
                cameras = EmioAPI.listCameraDevices()
                # print(f"\n\n{cameras=}\n\n")
                if not cameras:
                    # Silenciamos el error repetitivo, solo imprimimos una vez si fuera necesario
                    return

                # 2. Decidir índice (Sincronizar con motor SI existe, si no, usar 0)
                device_index = 0
                motor_ctrl = self.root.getChild("MotorController") 
                
                if motor_ctrl is not None:
                    # Solo accedemos si existe y tiene la propiedad emiomotors conectada
                    if hasattr(motor_ctrl, 'emiomotors') and motor_ctrl.emiomotors.is_connected:
                        device_index = motor_ctrl.emiomotors.device_index
                        print(f"EMIO MOTORS INITIALIZED WITH INDEX: {device_index}")
                # 3. Abrir cámara
                if len(cameras) > device_index:
                    self.tracker.open(cameras[device_index])
                    print(f"[DotTracker] Cámara iniciada (Índice {device_index}) tras espera.")
                else:
                    # Índice fuera de rango
                    return

            except Exception as e:
                print(f"[DotTracker] Error iniciando cámara: {e}")
                return


        alpha = self.filter_alpha
        

        # ========================================
        # ===========  TAG TRACKING ==============
        # ========================================
        
        # if self.track_tag and self.tracker.is_running:

        #     self.tracker.update()

        #     trackers_pos_rigid = list(filter(lambda x: (x[2] > 0 and x[2] < 400), self.tracker.trackers_pos))
        #     trackers_pos_rigid.sort(key=itemgetter(2))


        #     trackers_pos_leg = list(filter(lambda x: (x[2] > 0 and x[2] < 400), self.tracker.trackers_leg))
        #     trackers_pos_leg.sort(key=itemgetter(2))


        #     coord_centerTag = trackers_pos_rigid
        #     coord_leg = trackers_pos_leg

        #     if len(coord_centerTag) >= self.nb_tracker:

        #         self.trackersf1CenterTag = alpha*self.trackersf1CenterTag + (1.-alpha)*np.array(np.array(coord_centerTag))
        #         self.trackersf2CenterTag = alpha*self.trackersf2CenterTag + (1.-alpha)*np.array(self.trackersf1CenterTag)
        #         current_pose_centerTag = self.trackersf2CenterTag.copy()
        #         # print(f"\n{current_pose_centerTag=}\n")


        #         self.trackersf1Leg = alpha*self.trackersf1Leg + (1.-alpha)*np.array(np.array(coord_leg))
        #         self.trackersf2Leg = alpha*self.trackersf2Leg + (1.-alpha)*np.array(self.trackersf1Leg)

        #         current_pose_leg = self.trackersf2Leg.copy()
        #         # print(f"\n{current_pose_leg=}\n")


        #         # -------- ESPERAR ESTABILIZACIÓN --------
        #         if not self.started_pose_camera:
        #             self.counter += 1
        #             if self.counter <= 10:
        #                 # print(f"Stabilizing tracker {self.counter}/10")
        #                 return   
        #             self.initial_pose_tag = current_pose_centerTag.copy()
        #             self.initial_pose_tag_leg = current_pose_leg.copy()
        #             self.started_pose_camera = True
        #             # print(f"\nInitial pose locked: {self.initial_pose_tag}\n")
        #         # ----------------------------------------

        #         relative_pose_CenterTag = current_pose_centerTag - self.initial_pose_tag 
        #         relative_pose_leg = current_pose_leg - self.initial_pose_tag_leg

                
        #         self.mo_tracker_center.position.value = relative_pose_CenterTag 
        #         self.mo_tracker_center.reinit()

        #         self.mo_tracker_leg.position.value = relative_pose_leg 
        #         self.mo_tracker_leg.reinit()



        #         # print(f"\nNew Pose Center\n{self.mo_tracker_center.position.value}\n")
        #         # print(f"\nNew Pose Leg\n{self.mo_tracker_leg.position.value}\n")

        if self.track_tag and self.tracker.is_running:

            self.tracker.update()

            trackers_pos_rigid = list(filter(lambda x: (x[2] > 0 and x[2] < 400), self.tracker.trackers_pos))
            trackers_pos_rigid.sort(key=itemgetter(2))

            trackers_pos_leg = list(filter(lambda x: (x[2] > 0 and x[2] < 400), self.tracker.trackers_leg))
            trackers_pos_leg.sort(key=itemgetter(2))

            coord_centerTag = trackers_pos_rigid
            coord_leg = trackers_pos_leg
            try:
                # --- points are detected? 
                if len(coord_centerTag) >= self.nb_tracker and len(coord_leg) >= self.nb_tracker:

                    self.trackersf1CenterTag = alpha*self.trackersf1CenterTag + (1.-alpha)*np.array(coord_centerTag)
                    self.trackersf2CenterTag = alpha*self.trackersf2CenterTag + (1.-alpha)*np.array(self.trackersf1CenterTag)
                    current_pose_centerTag = self.trackersf2CenterTag.copy()

                    self.trackersf1Leg = alpha*self.trackersf1Leg + (1.-alpha)*np.array(coord_leg)
                    self.trackersf2Leg = alpha*self.trackersf2Leg + (1.-alpha)*np.array(self.trackersf1Leg)
                    current_pose_leg = self.trackersf2Leg.copy()

                    # -------- stabilization --------
                    if not self.started_pose_camera:
                        self.counter += 1
                        if self.counter <= 10:
                            return   
                        self.initial_pose_tag = current_pose_centerTag.copy()
                        self.initial_pose_tag_leg = current_pose_leg.copy()
                        self.started_pose_camera = True
                    # ---------------------------------------

                    # --- valid shapes before applying ---
                    if current_pose_centerTag.shape == self.initial_pose_tag.shape and current_pose_leg.shape == self.initial_pose_tag_leg.shape:
                        
                        # if all is ok, apply the relative poses
                        relative_pose_CenterTag = current_pose_centerTag - self.initial_pose_tag 
                        relative_pose_leg = current_pose_leg - self.initial_pose_tag_leg
                        
                        # save the last valid relative poses
                        self.last_valid_relative_center = relative_pose_CenterTag.copy()
                        self.last_valid_relative_leg = relative_pose_leg.copy()
                    else:
                        # Si los shapes no coinciden (ej. perdimos el tag), usamos el respaldo
                        relative_pose_CenterTag = self.last_valid_relative_center
                        relative_pose_leg = self.last_valid_relative_leg
                else:
                
                    # if not enough points, use the last valid relative poses
                    relative_pose_CenterTag = self.last_valid_relative_center
                    relative_pose_leg = self.last_valid_relative_leg

                # assign
                self.mo_tracker_center.position.value = relative_pose_CenterTag 
                self.mo_tracker_center.reinit()

                self.mo_tracker_leg.position.value = relative_pose_leg 
                self.mo_tracker_leg.reinit()
                
            except: 
                pass 


        if self.compute_point_cloud and self.tracker.is_running:
            coord_centerTag_pt = np.clip(self.tracker.point_cloud, -CLIP_DIST, CLIP_DIST)
            self.mo_tracker_center_point_cloud.position.value = coord_centerTag_pt.copy()
            self.mo_tracker_center_point_cloud.reinit()
