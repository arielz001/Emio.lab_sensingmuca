::::: collapse Leg adapted with MuCa

## Leg adapted with MuCa

This laboratory establishes a real-time touching sensing loop adapted to Emio. It bridges a physical deformable leg tracking environment with an interactive **SOFA (Simulation Open Framework Architecture)** simulation.

The pipeline integrates two distinct sensory tracking systems:

1. **Global Optical Tracking (AprilTags / Markers):** A camera array tracks rigid coordinate frames (Markers) placed on the physical setup. This defines the spatial transformation matrices ($T_{\text{world} \to \text{leg}}$) required to align the virtual asset's rest frame with the real environment.

2. **Local Tactile Matrix (MuCa Sensor):** A MuCa matrix wrapped around the leg captures local deformation forces. Continuous sub-pixel coordinates are reconstructed from discrete taxel activations using a centroid interpolation model.

---

###  The Sensory Infrastructure: Understanding Markers & Projectors

Before modifying the controller, it is vital to understand how spatial reference frames interact within the `createScene` hierarchy:

* **Rigid Transformations via Markers:** The `Markers` and `markerLeg` nodes receive real-time optical tracking frames from external AprilTag detectors. In the simulation, these markers act as **Effectors**, moving the physical object transforms the virtual constraints dynamically.

* **Surface Point Projection (`PointsOnSurface`):** A point cloud representing the MuCa sensor geometry (`PointsOnSurface.txt`) is loaded and permanently bound to the leg using **Barycentric Mapping**. If the leg bends or deforms, these virtual surface points track the underlying finite element mesh.

---

### Mathematical Model: Continuous Centroid Mapping

To compute the interactive contact point without spatial pixelation, the discrete localized activations are converted into a single weighted contact center vector ($\mathbf{InterpolatedPosition}$):

$$
I_{sum} = \sum_{i \in N} i_{i}
$$

$$
w_{i} = \frac{i_{i}}{I_{sum}}
$$

$$
\mathbf{InterpolatedPosition} = \sum_{i \in N} \left( w_{i} \cdot \mathbf{P}_{\text{Node}}[i] \right)
$$

Where:
- $i_i$ represents the intensity stored in `IntensityList[i]`.
- $w_i$ is the normalized scalar weight contribution of the neighbor node ($i$).
- $\mathbf{P}_{\text{Node}}[i]$ is the position vector extracted using the index mappings inside `IdxList`.

The computed coordinate $\mathbf{InterpolatedPosition}$ acts as the dynamic origin center of a volumetric `SphereROI` (Region of Interest). A `ForcePointActuator` (FPA) queries this ROI to compute local surface deformation normal vectors and inject corresponding mechanical forces into the system.

Open the workspace script to begin configuration:
#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

---

:::: exercise

**Exercise 1: Scene Architecture & Tactile Workspace (`createScene`)**

Your first task is to construct the scene topology within the `createScene` function by linking the physical tracking nodes to the virtual components.

1. **Position Effectors (Optical Frame Binding)**
   - Instantiate a `PositionEffector` object and attach it as a component to the core center-part marker node (`Markers`).
   - Instantiate a secondary `PositionEffector` object and attach it to the tracked leg marker node (`markerLeg`).

2. **Surface Point Projection Topology**
   - Create a child node named `PointsOnSurfaceNode` directly under the parent `LegTag` structural container.
   - Configure a data loader component to read coordinates from `PointsOnSurface.txt`.
   - Instantiate the target mechanical object container `POSNodeMO` with the following rigid spatial calibration transforms to match the physical camera origin:
     - `translation = [100.0, 0.0, 0.0]`
     - `rotation = [0.0, -90.0, -180.0]`
   - Append a `BarycentricMapping` component to project these coordinate arrays onto the active deformable leg surface mesh.

3. **Force Point Actuator Bounds**
   - Inside the predefined `FPANode` loop, append a `SphereROI` topological constraint component.
   - Wire the initial center position field to consume the `fpa_sphere_position` vector variable.
   - Point its target mechanical link property directly to the active mechanical object state using the `@MechanicalObject.position` reference path.
   - Create a `ForcePointActuator` instance tied explicitly to the computed ROI subset indices using `@SphereROI.indices`. Set operational limits by configuring `maxForceVariation = 100` alongside your calculated safety margins for `maxForce` and `minForce`.

::::

---

:::: exercise

**Exercise 2: Tactile Sensing (`onAnimateBeginEvent`)**

Locate the `onAnimateBeginEvent` routine inside the controller class. You must replace the blank expressions (`None` structures) with functional statement loops that process the incoming matrices.

#### Step-by-Step Task Specifications:
1. **Array Validation:** Extract data length attributes from the parallel arrays: `IntensityList` (containing $i_i$ values) and `IdxList` (containing spatial data indices).

2. **Mass Distribution Accumulation:** Compute the absolute cumulative mass denominator $V_{sum}$:
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
::::

---

### Verification & System Validation

Verify the cross-process data streams by running the physical acquisition modules concurrently with the numerical SOFA solver workspace engine:

1. Initialize the background serial communication loop handler to stream raw input indices from the hardware connection:
<!-- #open-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py") -->
#python-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py")

2. Launch the real-time interactive graphic simulation workspace to evaluate positional tracking performance:

#python-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

<!-- #runsofa-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py") -->
---

:::::