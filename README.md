## Real-Time Multi-Sensor Filtering Framework Project by : Atheer Ibraheem, Manar Taha, Mahmoud Diab 
  
## Details about the project

This project involves developing a data collection and analysis system using two sensors connected to an **ESP32 microcontroller** on a breadboard setup:  

1. **HC-SR04 Ultrasonic Distance Sensor** – This sensor was used to measure the distance between a human subject and the sensor at various fixed points.  
Experiments were conducted with the subject standing still at different distances ranging from **20 cm to 300 cm**, in order to evaluate the accuracy, stability, and noise characteristics of the sensor under controlled conditions. 
2. **MPU-6500 IMU Sensor** – used to record 6-axis motion data (3-axis accelerometer and 3-axis gyroscope) for analyzing movement and orientation patterns such as during walking upstairs or downstairs..

The ESP32 continuously collects readings from both sensors and transmits them over serial communication for logging and further analysis.  
A dedicated **Python-based analysis pipeline** processes the collected data, applying different filters (Exponential Moving Average, Low-Pass, Median, Outlier) to evaluate their performance across various recorded activities, including walking, sitting, standing, and stair movement.

This setup demonstrates a complete workflow—from data acquisition on embedded hardware to performance evaluation and visualization—using low-cost, reliable sensors for motion and distance measurement.

## HC-SR04 Wiring:

<img width="471" height="332" alt="image" src="https://github.com/user-attachments/assets/5c26541c-d68c-48af-a778-6c502433d281" />
<img width="467" height="349" alt="image" src="https://github.com/user-attachments/assets/33025d76-9577-4461-9fa8-31d708d67976" />


## MPU6500 Wiring:

<img width="492" height="376" alt="image" src="https://github.com/user-attachments/assets/03c17186-efcf-4335-966f-9ca15a58c636" />
<img width="494" height="369" alt="image" src="https://github.com/user-attachments/assets/8126dbff-c3ef-4af7-bf57-4ed423e201c1" />

## Folder description :
* **ESP32:**  
  Contains the source code for the ESP32 microcontroller responsible for real-time sensor logging and filter application.  
  Each sensor (HC-SR04 and MPU-6500) has its own dedicated code implementation, enabling independent data collection and filtering directly on the device.

* **Documentation:**  
  Includes the user manual, wiring diagram, and a detailed explanation of the system pipeline.  
  This section describes how the hardware and software components interact and provides guidance for setup and operation.

* **Data:**  
  Stores all collected sensor data from the ESP32, along with the generated analysis results, performance metrics, and visualization plots.  
  Separate directories are organized for each sensor and filter type to allow structured comparison across experiments.

* **Tools:**  
  Contains Python-based tools and scripts used for data collection, post-processing, and analysis.  
  These scripts connect to the ESP32 via serial communication to log real-time data and perform analysis for both sensors using the implemented filtering methods.


## ESP32 SDK version used in this project: 
The project was developed using the **Arduino Core for ESP32 (version 2.0.17)**  
by Espressif Systems, based on **ESP-IDF v4.4.6**.

## Arduino/ESP32 libraries used in this project:
* **Wire** – version 1.0.1  
  Core Arduino I²C communication library used to interface with sensors over the I²C bus.

* **MPU9250_WE** – version 1.2.15  
  Library for interfacing with the MPU9250/MPU6500 IMU sensor, providing easy access to accelerometer, gyroscope, and magnetometer data.


## Connection diagram:
<img width="963" height="602" alt="pipeline_flowchart" src="https://github.com/user-attachments/assets/7994be59-2418-4da2-b785-72b5e8a0ab55" />

## Project Poster:

 ![IOT_GROUP1_POSTER](https://github.com/user-attachments/assets/b7cec5ff-77d4-4d3e-bd07-881a8ff4e644)

This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
