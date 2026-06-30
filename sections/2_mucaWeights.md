:::::: collapse Weight Interpolation
###  Weight Interpolation

In this workshop, you will implement a **Discrete Center-of-Mass (Centroid)** algorithm to achieve sub-pixel spatial resolution. Instead of just identifying which raw sensor node has the highest signal, you will dynamically calculate the exact position of a touch by interpolating adjacent taxel node weights in a single collapsed column vector.

$$\text{Interpolated Position} = \sum_{i=0}^{n} w_i \cdot i_i  $$

Where $w_i$ is the dynamically normalized weight of taxel $i$, derived from the intensities values $i_i$.

---

:::: exercise

**Exercise: Interpolation Engine Implementation**

Your objective is to finalize the real-time processing loop within the visualizer framework. Open your script and locate the update thread routine. You must replace the placeholder with a functional iteration loop that dynamically computes the cumulative mass and normalizes the incoming streaming vectors.

#### Task Specifications:

1. **Array Parsing:** Access the parallel input arrays: `intensities` (containing the raw $i_i$ activation values) and `idx_list` (containing spatial matrix data indices).

2. **Axis Mapping:** Extract the spatial index values representing the rows. In this environment, the target row data position $y_i$ is mapped as:
   $$y_i = \text{idx\_list}[i]$$

3. **Centroid Equation with Dynamic Normalization:** Compute the absolute cumulative mass denominator $I_{sum}$ first, then calculate the normalized scalar weight factor $w_i$ inside your loop to accurately update the continuous tracking coordinate:
   $$I_{sum} = \sum_{j=0}^{n-1} \text{Intensities}[j]$$
   $$w_i = \frac{\text{Intensities}[i]}{I_{sum}}$$
   $$\text{InterpolatedPosition} = \sum_{i=0}^{n-1} w_i \cdot y_i $$
   Where each step in the iteration loop executes this exact cumulative sequence:
   $$\text{InterpolatedPosition} \leftarrow \text{InterpolatedPosition} + (w_i \cdot y_i)$$

#open-button("assets/labs/lab_sensingmuca/lab_mucaSensor.py")

::::
::::::