::::: collapse Leg adapted with MuCa

## Leg adapted with MuCa

For the following scene, you could acces to the official documentation [here](https://sofa-framework.github.io/doc/components/statecontainer/mechanicalobject/).

This laboratory establishes a real-time touching sensing loop adapted to Emio. It bridges a physical deformable leg tracking environment with an interactive **SOFA (Simulation Open Framework Architecture)** simulation.

The pipeline integrates two distinct sensory tracking systems:

1. **Local Tactile Matrix (MuCa Sensor):** A MuCa matrix wrapped around the leg captures local deformation forces. Continuous sub-pixel coordinates are reconstructed from discrete taxel activations using a centroid interpolation model (now in 3D).

The modified Leg can be appreciated in the following image:

![](assets/labs/lab_sensingmuca/data/images/ModifiedLeg.png){width=60% .center}


2. **Global Optical Tracking (AprilTags / Markers):** A camera array tracks rigid coordinate frames (Markers) placed on the physical setup. 



---

###  Sensorization

Before modifying the controller, it is vital to understand how spatial reference frames interact within the `createScene` hierarchy:


* **Surface Point Projection (`PointsOnSurface`):** A point cloud representing the MuCa sensor geometry (`PointsOnSurface.txt`) is loaded and permanently bound to the leg using **Barycentric Mapping**. If the leg deforms, these virtual surface points track the underlying finite element mesh.



* **Rigid Transformations via Markers:** The `Markers` and `markerLeg` nodes receive real-time optical tracking frames from external AprilTag detectors. In the simulation, these markers act as **Effectors**, moving  the virtual constraints dynamically according to the motion of the leg in the physical setup.

---

:::: exercise

**Exercise 1: Scene Architecture & Tactile Workspace (`createScene`)**

Your first task is to modify the scene topology within the `createScene` function by linking the physical tracking nodes to the virtual components.

1. **Surface Point Projection Topology**
   - Create a child node named `PointsOnSurfaceNode` directly under the parent `LegTag` node.
   - Instantiate the target mechanical object  `POSNodeMO` with the following rigid spatial calibration transforms to match the orientation of the leg in EMIO:
     - `translation = [100.0, 0.0, 0.0]`
     - `rotation = [0.0, -90.0, -180.0]`
   - Append a `BarycentricMapping` component to project these coordinate arrays onto the active deformable leg surface mesh.


2. **Sphere Tactile Sensing**
   - Inside the predefined `FPANode` loop, append a `SphereROI` topological constraint component.
   - Set the initial center position with the `fpa_sphere_position` vector variable.
   - Point its target mechanical link property directly to the active mechanical object state using `@MechanicalObject.position`.

#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

You cannot run yet, because you need to implement the center-of-mass algorithm in the following excercise.
::::

---

:::: exercise

**Exercise 2: Tactile Sensing (`onAnimateBeginEvent`)**

Locate the `onAnimateBeginEvent` routine inside the controller class. You must replace the blank expressions (`None` structures) with functional statement loops that process the incoming matrices.

#### Step-by-Step Task Specifications:
1. **Array Validation:** Extract data length attributes from the parallel arrays: `IntensityList` (containing $i_i$ values) and `IdxList` (containing spatial data indices).

2. **:** 
$I_{sum}$:
   $$I_{sum} = \sum_{i=0}^{n-1} \text{IntensityList}[i]$$

3. **Centroid Weight Vector Processing (3D Mapping):** Loop through your active neighbor matrices ($N$). Unlike the 2D workspace, you must calculate a 3D target coordinate vector ($\mathbf{InterpolatedPosition}$). For every iteration, retrieve the physical 3D vertex coordinate ($\mathbf{P}_{\text{linearIdx}}$) from the simulation mesh using the specific node index, normalize its scalar weight component, and update your cumulative accumulator:
   
   * **Mathematical Formulation:**
     $$w_i = \frac{\text{IntensityList}[i]}{I_{sum}}$$
     $$\mathbf{InterpolatedPosition} = \sum_{i=0}^{n-1} \left( \mathbf{P}_{\text{linearIdx}} \cdot w_i \right)$$

   * **Implementation Guidelines:**
     Remember that `self.POSNodeMO.position.value` holds the full 3D coordinate space array. To retrieve the specific location vector $\mathbf{P}_{\text{linearIdx}}$ for your calculation, index the array directly using the unique tracking identifier `LinearIdx`:
     $$\mathbf{P}_{\text{linearIdx}} = \text{self.POSNodeMO.position.value}[\text{LinearIdx}]$$
     Multiply this spatial array by your calculated normalized weight factor $w_i$ and continuously increment your running vector total:
     $$\mathbf{InterpolatedPosition} \leftarrow \mathbf{InterpolatedPosition} + (\mathbf{P}_{\text{linearIdx}} \cdot w_i)$$


#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

#runsofa-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

::::

---

:::: exercise

**Exercise 3: Force Estimation**

Now, you must to measure the forces applied to the robot's legs. We use the MuCa sensors to get the location of tactile sensing and the markers for estimate the magnitude of the forces.

1. **Position Effectors (Markers)**
   - Instantiate a `PositionEffector` object and attach it as a component to the core center-part marker node (`Markers`).
   - Instantiate a secondary `PositionEffector` object and attach it to the tracked leg marker node (`markerLeg`).
 
2. **Force Point Actuator**
   - Create a `ForcePointActuator` instance tied explicitly to the computed ROI subset indices using `@SphereROI.indices`. Set operational limits by configuring `maxForceVariation = 100` alongside your calculated safety margins for `maxForce` and `minForce`.



### SIMULATION

To simulate you should:

1. Initialize the touching sensing:
<!-- #open-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py") -->
#python-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py")
#python-button("assets/labs/lab_sensingmuca/lab_mucaSensor.py")

2. Launch the real-time interactive graphic simulation workspace to evaluate positional tracking performance:


#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

#runsofa-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

---

:::::