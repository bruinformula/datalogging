# Datalogging
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1H7nEAA-eNDy9veOrQS1f3jlEyWO0lqVX", width="50%"/>
</p>
This repository contains all on vehicle data acquistion & data logging code. Archictecturally this is broken into a primary PCB (dash-pcb) which handles logging all CAN messages and it's inputs (IMU, shifting on Mk.8 car) to SD as well as outputting some amount of that data for real time driver feedback. This primary PCB also aggregates input from multiple secondary PCBs (corner-pcb) as well as outputs signals for real time telemetry (telemetry)
