:::::: collapse Discover MUCA
###  Discover MUCA

**Mutual Capacitance (MuCa)** is a sensing technology that measures the electrical capacitance between on a grid of $n\times m$ crossing points (max. $21\times 12$). It works on the same principle as touch screens in consumer electronics and is used to develop touch sensitive surfaces. Each crossing point on the grid is sensitive to the change in capacitance related to the presence of the human finger. Thus, it is possible to locate the interaction on the surface. However, the technology by itself is not sensitive to force or pressure.

![](assets/labs/lab_sensingmuca/data/images/muca.png){width=35% .center}

---

###  Prerequisites & Installation

To run the data acquisition scripts, ensure you install the required dependencies first. Open your terminal and run the following command:


#python-button("-m pip install --target 'assets/labs/lab_sensingmuca/modules/site-packages' -r 'assets/labs/lab_sensingmuca/requirements.txt'")

If this command fails, try to install the dependencies manually running your local python:
```bash
pip install scipy matplotlib pyserial
```

---

###  Data Acquisition Scripts

#### 1. Full Matrix Resolution
To capture and stream the complete sensor grid matrix ($21 \times 12$), use this module:

#python-button("assets/labs/lab_sensingmuca/modules/readAllDataMucaSensor.py")

<!-- #open-button("assets/labs/lab_sensingmuca/modules/readAllDataMucaSensor.py") -->

#### 2. Subset Stream (Sensors 1 & 2)
To filter the stream and isolate data from only the relevant sensor lines, use this module:

<!-- #open-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py") -->

#python-button("assets/labs/lab_sensingmuca/modules/readMucaSensor.py")

(we actually use redundant lines/traces to improve sensing robustness)

::::::

