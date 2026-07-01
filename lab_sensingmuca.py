import Sofa
import sys
import os
import numpy as np
import parts.controllers.motorcontroller as mc

current_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = os.path.abspath(os.path.join(current_dir, '../../'))
camera_dir = os.path.abspath(os.path.join(current_dir, './camera/'))

if assets_dir not in sys.path:
    sys.path.append(assets_dir)
if camera_dir not in sys.path:
    sys.path.append(camera_dir)

from camera._camera import Camera
from camera.trackercontroller2tags import ApriltagTracker

import Sofa.ImGui as MyGui
import time
from utils_lab import create_path, generate_interpolation

TouchSim_dir = f"{os.path.dirname(os.path.abspath(__file__))}/modules/mucaData/"



class Controller(Sofa.Core.Controller):   
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        self.SphereROI = kwargs['SphereROI']
        self.FPA = kwargs['FPA']
        
        self.PositionEffectorMarker = kwargs['PositionEffectorMarker']
        self.SecondPositionEffectorMarker = kwargs['SecondPositionEffectorMarker']
        self.POSNodeMO = kwargs['POSNodeMO']

        self.markers = kwargs.get('markers') 
        self.markerLeg = kwargs.get('markerLeg')

        self.LegTag = kwargs.get('LegTag')
        self.counter = 0

        self.followpathMO = kwargs.get('followpathMO')
        self.path2follow = kwargs.get('path2follow')
        print('Finished Init')
        self.following_active = False
        self.indice_ruta_actual = 0

        self.LegTagDeformable = kwargs.get('LegTagDeformable')

        self.last_FPA_direction = None
        self.reinitTags = False
        self.followedWayMO = kwargs.get('followedWayMO')

        self.drawTrajectory = False
        self.tagWayMO = kwargs.get('tagWayMO')
        self.deletePoints = False
        self.centerFollowedPoints = []
        self.centerTagPoints = []

        self.fpa_active = True

        self.TouchPointsMO = kwargs.get('TouchPointsMO')


    def onKeypressedEvent(self, c):
        key = c['key']
        print(f"\n\nKey pressed: {key}\n\n")
        
        if key == 'K':
            self.fpa_active = not self.fpa_active
            print(f"FPA Active: {self.fpa_active}")
        if key == 'D':
            self.drawTrajectory = not self.drawTrajectory
            print(f"Draw Trajectory: {self.drawTrajectory}")
        if key == '7':
            self.following_active = not self.following_active
            print(f"Following Active: {self.following_active}")
        if key == '9':
            self.reinitTags = True




    def onAnimateBeginEvent(self, eventType):

        
        if self.RootNode.getChild("DepthCamera") is None or self.markers is None:
            return
        

        emio_is_connected = MyGui.getRobotConnectionToggle()

        # ================================================================
        # for smoothing
        # ================================================================
        if not hasattr(self, 'frames_estabilizacion'):
            self.frames_estabilizacion = 0
            self.target_centro_smooth = None
            self.target_leg_smooth = None

        # ================================================================
        # THIS IS FOR STABILIZE EMIO (when the motors are active)
        # ================================================================
        if emio_is_connected and self.frames_estabilizacion < 60:
            self.frames_estabilizacion += 1
            self.reinitTags = True
            
            
        # offset inizialization
        if not hasattr(self, 'offset_center'):
            self.offset_center = np.array([0.0, 0.0, 0.0])
            self.offset_leg = np.array([0.0, 0.0, 0.0])
            self.last_FPA_direction = None

        # ================================================
        #         Tags Following
        # ================================================
        trackercenter = self.RootNode.DepthCamera.TrackerCenterTag
        trackerleg = self.RootNode.DepthCamera.TrackerLegTag
        
        TrackerCenterTag = trackercenter.position.value 
        TrackerLegTag = trackerleg.position.value

        # Data from sofa 
        markers_pos = self.markers.getMechanicalState().position.value
        secondMarker_pos = self.markerLeg.getMechanicalState().position.value

        # ------------ 3. Getting data ----------
        center_target_real = np.array(TrackerCenterTag[0][0:3])
        leg_target_real = np.array(TrackerLegTag[0][0:3]) 

        center_current = np.array(markers_pos[0][0:3])
        leg_current = np.array(secondMarker_pos[0][0:3])

        # ================================================================
        # REINIT TAGS (This is for stabilization)
        # ================================================================
        if self.reinitTags:
            self.offset_center = center_target_real - center_current
            self.offset_leg = leg_target_real - leg_current
            
            self.target_centro_smooth = center_current
            self.target_leg_smooth = leg_current
            
            self.reinitTags = False

        # ================================================================
        # RELATIVE TAGS
        # ================================================================
        # correctedp osition 
        center_target_corrected = center_target_real - self.offset_center
        leg_target_corrected = leg_target_real - self.offset_leg
        
        # updating tags directly in SOFA without reinit()
        trackercenter.position.value = [center_target_corrected.tolist()]
        trackerleg.position.value = [leg_target_corrected.tolist()]




        # ================================================================
        # FOR SMOOTHING
        # ================================================================
        if self.target_centro_smooth is None:
            self.target_centro_smooth = center_current
            self.target_leg_smooth = leg_current

        
        beta = 0.5
        self.target_leg_smooth = self.target_leg_smooth + beta * (leg_target_corrected - self.target_leg_smooth)

        # ====================================
        #          CENTER TAG FOLLOWING
        # ====================================
        puntos_ruta = self.path2follow

        if self.following_active:
            if not hasattr(self, 'ultimo_tiempo_cambio'):
                self.ultimo_tiempo_cambio = time.time()
                self.tiempo_por_punto = 0.01

            tiempo_actual = time.time()
            tiempo_transcurrido = tiempo_actual - self.ultimo_tiempo_cambio

            if tiempo_transcurrido >= self.tiempo_por_punto:
                self.indice_ruta_actual = (self.indice_ruta_actual + 1) % len(puntos_ruta)
                self.ultimo_tiempo_cambio = tiempo_actual 
            
            center_target = np.array(puntos_ruta[self.indice_ruta_actual][0:3])
            self.followpathMO.position.value = [center_target.tolist()] 
            self.PositionEffectorMarker.effectorGoal.value = self.followpathMO.position.value  

        else:
            self.target_centro_smooth = self.target_centro_smooth + beta * (center_target_corrected - self.target_centro_smooth)
            
            distance_center = np.linalg.norm(self.target_centro_smooth - center_current)
            self.markers.error1.value = distance_center
            
            if distance_center > 3:
                new_center = center_current + 0.001 * (self.target_centro_smooth - center_current)
            else:
                new_center = center_current
                
            new_center_full = markers_pos.copy()
            new_center_full[0][0:3] = new_center
            self.PositionEffectorMarker.effectorGoal.value = new_center_full


        # ====================================
        #  LEG TAG FOLLOWING 
        # ====================================
        distance_leg = np.linalg.norm(self.target_leg_smooth - leg_current)
        threshold_leg = 0.1  

        if distance_leg > threshold_leg:
            new_leg = leg_current + 0.5 * (self.target_leg_smooth - leg_current)
        else:
            new_leg = leg_current

        new_leg_full = secondMarker_pos.copy()
        new_leg_full[0][0:3] = new_leg
        self.SecondPositionEffectorMarker.effectorGoal.value = new_leg_full

        # ================================================
        #                DRAW PATH
        # ================================================

        if self.following_active and self.drawTrajectory and not self.deletePoints:
            self.centerFollowedPoints.append(center_current)
            self.centerTagPoints.append(center_target_corrected)


            with self.followedWayMO.position.writeable() as p:
                for i in range(len(self.centerFollowedPoints)):
                    p[i] = self.centerFollowedPoints[i]

            with self.tagWayMO.position.writeable() as p:
                for i in range(len(self.centerTagPoints)):
                    p[i] = self.centerTagPoints[i]

            if len(self.centerFollowedPoints) > 10000:
                self.centerFollowedPoints = []
                self.centerTagPoints = []

            self.tagWayMO.reinit()
            self.followedWayMO.reinit()


        # ================================================
        #                   FPA
        # ================================================
        if self.fpa_active:
            try:
                IdxList = np.loadtxt(TouchSim_dir + "IdxList.txt")
                WeightList = np.loadtxt(TouchSim_dir + "WeightList.txt")
            except:
                IdxList = np.array([])
                WeightList = np.array([])


            detected_idxs = self.SphereROI.indices.value
            if len(detected_idxs) > 0:
                indice_selected = detected_idxs[int(len(detected_idxs)//2)]
                self.FPA.indices.value = [indice_selected]

            self.counter += 1


            # =======================================================
            # TODO: Calculate the weighted position of the detected points
            # HINT: To access to 3d position of the touching points in the scene,
            #       use the following code:
            #       self.POSNodeMO.position.value
            #       note that is an array, so you should select the idx
            # =======================================================

            if len(IdxList) > 0 and WeightList.size > 0 and np.sum(WeightList) > 0:
                
                InterpolatedPosition = 0.0  


                for (i, Idx) in enumerate(IdxList):
                    LinearIdx = int(Idx)
                      
                    Wi = None #(wij)
                    PointPosition3D = None  #(g_w)
                    InterpolatedPosition = None

                self.SphereROI.centers.value = [InterpolatedPosition.tolist()] 

            else:             
                self.SphereROI.centers.value = [[0.0, 0.0, 0.0]]



            Points = self.SphereROI.pointsInROI.value
            idxs = self.SphereROI.indices.value
            
            all_positions = np.array(self.TouchPointsMO.position.value)
            halfTotalPoints = int(all_positions.shape[0] / 2)

            if len(Points) >= 1: 
                self.lost_steps_counter = 0
                self.FPA.maxForceVariation.value = 10.0
                self.FPA.maxForce.value = 5000
                self.FPA.minForce.value = 0
                
                directions_list = []
                    
                for i, idx in enumerate(idxs):
                    if idx < halfTotalPoints:
                        pt_A = np.array(Points[i])
                        # searching sencond point
                        pair_idx = idx + halfTotalPoints
                        pt_B = all_positions[pair_idx]
                        # direction
                        dir_vector = pt_B - pt_A
                        # normalization
                        norm = np.linalg.norm(dir_vector)
                        if norm > 0:
                            dir_vector = dir_vector / norm
                        
                        directions_list.append(dir_vector)

                # --- mean calculation ---
                if len(directions_list) > 0:
                    avg_direction = np.mean(directions_list, axis=0)
                    self.FPA.direction.value = avg_direction.tolist()
                else:
                    print("No points met the condition idx < halfTotalPoints to calculate direction.")


            else:
                if not hasattr(self, 'lost_steps_counter'):
                    self.lost_steps_counter = 0
                    
                self.lost_steps_counter += 1
                
                if self.lost_steps_counter <= 4:
                    self.FPA.maxForceVariation.value = 10.0
                    self.FPA.maxForce.value = 100
                    self.FPA.minForce.value = 0
                else:
                    self.SphereROI.indices.value = []
                    self.FPA.indices.value = []
                    self.FPA.maxForceVariation.value =  1000000000
                    self.FPA.maxForce.value = 0
                    self.FPA.minForce.value = 0
                    self.FPA.direction.value = [-0.7, 0.5, 0.0]
                    
        else:
                self.FPA.indices.value = []
                self.FPA.maxForceVariation.value =  1000000000
                self.FPA.maxForce.value = 0
                self.FPA.minForce.value = 0




def createScene(rootnode):
    rootnode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') 


    try:
        IdxList = np.loadtxt(TouchSim_dir + "IdxList.txt")
        WeightList = np.loadtxt(TouchSim_dir + "WeightList.txt")
    except:
        IdxList = np.array([])
        WeightList = np.array([])

    # void idx 
    idxlist = []
    WeightList = []
    np.savetxt(TouchSim_dir + "IdxList.txt", idxlist, fmt='%i')
    np.savetxt(TouchSim_dir + "WeightList.txt", WeightList)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.abspath(os.path.join(current_dir, '../../'))
    if assets_dir not in sys.path:
        sys.path.append(assets_dir)

    from utils.header import addHeader, addSolvers
    from parts.controllers.assemblycontroller import AssemblyController
    from parts.emio import Emio
    

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual", "showBehaviorModels", "showInteractionForceFields","showForceFields"]
    # rootnode.SceneGraph.drawMode = 1
    rootnode.dt = 0.01 
    rootnode.gravity = [0., -9810., 0.]

    addSolvers(simulation)

    camera = modelling.addChild(Camera())
    

    # =============================================
    #               EMIO INITIALIZATION
    # =============================================

    emio = Emio(name="Emio",
                legsName=["blueleg"],
                # legsName=["bluelegMuca"],
                legsModel=["tetra"],
                legsPositionOnMotor=["counterclockwisedown", "clockwisedown", "counterclockwisedown", "clockwisedown"],
                centerPartName="bluepart",
                centerPartType="rigid",
                extended=True)
    

    rootnode.addObject('BackgroundSetting', color='255 255 255')

    if not emio.isValid():
        return
    
    # ASSEMBLY

    sim = simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)



    # =============================================
    # CENTERPART MARKER
    # =============================================
    centerpart = emio.getChild("CenterPart")
    markers_pos = [[0.0, 0.0, 0]]
    markers = centerpart.addChild("Markers")

    MarkerMO = markers.addObject("MechanicalObject",name = "MarkerMO",
                      position=markers_pos + [[0, 0,0]],
                      showObject=True, showObjectScale=5, drawMode=1, showColor=[0, 0.0, 1, 1])
    
    # TODO: INSTANCE THE POSITIONEFFECTOR MARKER (CENTERPART) 
    PositionEffectorMarker = None



    # THIS IS FOR THE WAY 
    initial_marker_center_pos =  centerpart.getMechanicalState().position.value[0].copy()
    initial_marker_center_pos[1] += 10



    targetNode = modelling.addChild('Target')
    targetNode.addObject('EulerImplicitSolver', firstOrder=True)
    targetNode.addObject('CGLinearSolver', iterations=20, tolerance=1e-5, threshold=1e-10)
    
    
    emio.addInverseComponentAndGUI(targetMechaLink=MarkerMO.position.linkpath, withGUI=False)
    # rootnode.addObject(LabGUIExerciseIK(rootnode, emio))

    markers.addObject("RigidMapping") # this is rigid because the centerpart is rigid

    #is only for view the error 
    markers.addData(name="error1", type="float", value=0)
    
    # GUI TO VIEW ERROR (IS NOT NECESSARY HERE)
    group = "Error Markers (mm)"
    MyGui.MyRobotWindow.addInformationInGroup("Marker 1", markers.error1, group)


    # =============================================
    #   LEG MARKER 
    # =============================================

    leg1 = emio.getChild("Leg1")
    leg1Deformable = leg1.getChild("Leg1DeformablePart")
    
    LegTag = leg1Deformable.getChild("Leg")
    
    # middle down marker (2nd place from down to top)
    translation_leg = [
        97.5 + 18,
        -89.0 + -16,
        0 ]
    
    # ADD SPHERE ROI IN CENTER OF LEG
    markerLeg = LegTag.addChild("MarkerLeg")
    markerLegMO = markerLeg.addObject("MechanicalObject", name="markerLegMO", 
                                    template="Vec3d", position=translation_leg, showObject=True, 
                                    showObjectScale=5, drawMode=1, showColor=[1, 0.0, 0, 1])
    markerLeg.addObject("BarycentricMapping")

    # TODO: INSTANCE THE POSITIONEFFECTOR MARKER (LEGPART)
    SecondPositionEffectorMarker = None

    # ===========================================
    #            Tactile Sensing
    # ===========================================
   
    # TODO: CREATE A CHILDNODE FOR THETACTILE SENSING AS CHILD OF LEGTAG
    
    PointsOnSurfaceNode = None

    POS = np.loadtxt(TouchSim_dir + "PointsOnSurface.txt")

    # this is a little correction for the points
    POS = POS[::-1]
    POS[:,2] += 2
    print("POS:", POS)

    # TODO: CREATE THE MECHANICAL OBJECTS FOR THE POINTS IN A POSNodeMO ON THE PointsOnSurfaceNode 
    # you should use the following translation and rotation
    traslation_posnode = [97.5+2.5,0,0]
    rotation_posnode = [0,-90,-180]

    POSNodeMO = None

    # TODO: CREATE A BARYCENTRIC MAPPING FOR THE POSNodeMO
    
    # ==========================================
    #   Generating points to get direction of fpa
    # ==========================================

    interpolated_positions = generate_interpolation(POS, quantityinterp=10)

    TouchPointsNode = LegTag.addChild("TouchPoints")
    TouchPointsMO = TouchPointsNode.addObject("MechanicalObject", 
                              position=interpolated_positions, 
                              showObject=True, 
                              showObjectScale=10,
                              translation = [97.5+2.5,0,0], 
                              rotation = [0,-90,-180],
                              showColor=[0, 1, 0, 1])
    TouchPointsNode.addObject("BarycentricMapping")



    ## ==================================================
    #       LEG FPA APPLIED TO TOUCH POINTS
    ## ==================================================
    offset_sphere = 10
    fpa_sphere_position = translation_leg.copy()
    fpa_sphere_position[0]+= offset_sphere 

    FPANode = TouchPointsNode.addChild("FPANode")


    # =======================================================
    #                   TODO TODO TODO                       
    # =======================================================

    # TODO: CREATE A SPHEREROI FOR A FPA ASSOCIATED TO THE FPANODE MECHANICAL OBJECT POSITION 
    # (position="@MechanicalObject.position"))
    # HINT: THE CENTERS SHOULD BE THE fpa_sphere_position

    SphereROI = None

 
    # TODO: CREATE A FPA FOR THE SPHEREROI
    # HINT: indices='@SphereROI.indices'
    # HINT: you should be set the maxForce and minForce, also the maxForceVariation (recommended 100)

    FPA = None
   
    
    SphereROI.init()
    FPA.init()

    try:
        tracker = ApriltagTracker(name="ApriltagTracker",
                                root=rootnode,
                                nb_tracker=1,
                                show_video_feed=True, # True to see the camera
                                compute_point_cloud=False,
                                scale=1,
                                rotation=camera.torealrotation,
                                translation=camera.torealtranslation,
                                center_marker_position=initial_marker_center_pos,
                                leg_marker_position=translation_leg,
                                show_spheres=True
                                ) 
        
        rootnode.addObject(tracker)
        print(f"\n\ntracker initialized\n\n")


    
    except Exception as e:
        print("Warning: No se pudo iniciar el Tracker de camara:", e)



    # path2follow = create_path(initial_marker_center_pos, shape='line', total_points=300, size_followpath=60)
    path2follow = create_path(initial_marker_center_pos, shape='point', total_points=1, size_followpath=40)




    # ===========================================
    #            Effectors !! 
    # ===========================================

    followPathVisualizationNode = rootnode.addChild('followPathVisualizationNode')
    followpathMO = followPathVisualizationNode.addObject('MechanicalObject', 
                                        name='followpathEffectorMO',
                                        template='Vec3d', 
                                        position=path2follow,  
                                        showObject=True, 
                                        showObjectScale=1, 
                                        drawMode=1, 
                                        showColor=[0.5, 0.0, 0.5, 1])


    followedWay = modelling.addChild('MarkerWay')
    tagWay = modelling.addChild('TagWay')


    followedWayMO = followedWay.addObject('MechanicalObject', name=f'followedWayMO', 
                                template='Vec3d', 
                                position=[[0,0,0]]*10000,  
                                showObject=True, 
                                showObjectScale=1.0, 
                                drawMode=1, 
                                showColor=[0, 0, 1, 1])

    tagWayMO = tagWay.addObject('MechanicalObject', name=f'tagWayMO', 
                                template='Vec3d', 
                                position=[[0,0,0]]*10000,  
                                showObject=True, 
                                showObjectScale=1.0, 
                                drawMode=1,
                                showColor=[0, 1, 0, 1])

    # -------------------------------------------------------------------



    # TODO: LAB 2 --> Set Motors to 0 maxanglevariation

    motor_0 = emio.getChild("Motor0")
    motor_1 = None
    motor_2 = None
    motor_3 = None
    

    # motor_0.JointActuator...
    # motor_1.JointActuator...
    # motor_2.JointActuator...
    # motor_3.JointActuator...



    
    # main controller
    rootnode.addObject(Controller(name="ActuationController", 
                                  RootNode=rootnode,
                                  SphereROI=SphereROI,
                                  POSNodeMO=POSNodeMO,
                                  PositionEffectorMarker=PositionEffectorMarker,
                                  SecondPositionEffectorMarker=SecondPositionEffectorMarker,
                                  FPA=FPA,
                                  TouchPointsMO=TouchPointsMO,
                                  LegTag=LegTag,
                                  markers=markers,
                                  markerLeg=markerLeg, 
                                  followpathMO=followpathMO,
                                  path2follow=path2follow,
                                  LegTagDeformable=leg1Deformable,
                                  followedWayMO=followedWayMO,
                                  tagWayMO=tagWayMO,
                                  ))
    

    try:
        print("Trying to connect the robot...")
        emio.addConnectionComponents()
        print("Communication driver loaded.")
    except Exception as e:
        print(f"Error: {e}")

    return rootnode