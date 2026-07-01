::::: collapse Force Measurement

## Force Measurement

**Measuring Forces Applied to the Robot.**  
In this lab, we will use the MuCa sensors to measure forces applied to the robot's legs.

To obtain meaningful force measurements, the robot must remain passive during the experiment. If the motors are allowed to move, their actuation may influence the contact forces and alter the sensor readings.

For this reason, the first step is to disable motor motion before performing any force interaction.

In SOFA, each motor is controlled through a `JointActuator`. One of its parameters is:

$$
\Delta\theta_{\max}
$$

which represents the maximum angular variation allowed during a simulation step.

By setting:

$$
\Delta\theta_{\max} = 0
$$

the motor is prevented from changing its angle, ensuring that the measured forces originate only from the external interaction with the robot.

Open the main controller script:

#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

---

:::: exercise

**Exercise 1: Preparing the Robot for Force Measurement**

Before measuring forces with the MuCa sensors, configure the motors so that they remain fixed during the experiment.

1. Retrieve the missing motor nodes:
   - `Motor1`
   - `Motor2`
   - `Motor3`

2. Access the corresponding `JointActuator` of each motor.

3. Modify the actuator parameter that limits the angular variation so that all motors remain locked during the simulation. (motor_number.JointActuator.maxAngleVariation.value = 0)

#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")


4. Also, you should print the force measurements in the terminal.
---

::::
### Verification

You can validate your implementation by launching the SOFA simulation environment.

1. Open the lab scene.
#open-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

2. Run the simulation.
#runsofa-button("assets/labs/lab_sensingmuca/lab_sensingmuca.py")

3. Interact with the robot and observe the force measurements reported by the MuCa sensors.

4. Verify that the motors remain fixed while external forces are applied to the robot.


::::

:::::