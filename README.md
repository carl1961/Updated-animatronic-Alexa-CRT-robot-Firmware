# Updated-animatronic-Alexa-CRT-robot-Firmware
Updated the animatronic Alexa CRT robot Firmware, for FM225 Facial Recognition Module HLK-FM225   https://www.hackster.io/tmburns/breathing-life-into-an-amazon-echo-device-ed9ea3

<img width="606" height="453" alt="image" src="https://github.com/user-attachments/assets/f821d32b-8d04-40df-9729-d5db5a3b966d" />



Updating the obsolete, not available, Person Sensor by Useful Sensors SKU: SEN-21231 with a FM225 Facial Recognition Module HLK-FM225

<img width="406" height="423" alt="image" src="https://github.com/user-attachments/assets/fdab700f-ec23-44e2-a4c6-927a8ffa11cd" />

# Work in Progress............


<img width="1500" height="1247" alt="image" src="https://github.com/user-attachments/assets/9d8631c5-2581-40d3-8046-0eac71f5ec9a" />

# Code

The Mega 2560 Pro combined with the HLK-FM225 is a phenomenal pairing for upgrading the animatronic Alexa CRT robot. 
Moving away from the long-discontinued Person Sensor lets you drop the finicky I2C addressing entirely and build a 
much more robust setup.Since the original project used an Arduino-compatible microcontroller, keeping an ATmega2560 
means your core servo-driving logic, timing, and structural code stay exactly the same. You just need to redirect how 
it captures tracking data.Here is how you lay out the upgraded hardware and configure your serial connection:

1. Wiring the HardwareThe Mega 2560 Pro gives you an immediate advantage over standard microcontrollers: Multiple Hardware UARTS (Serial Ports).
  Instead of fighting with slow, fragile SoftwareSerial setups, you can leave Serial dedicated to debugging via USB and route the FM225 straight
  to Serial1.FM225 TX , Mega 2560 Pro Pin 19 (RX1)FM225 RX  Mega 2560 Pro Pin 18 (TX1)GND  Common Ground
  (Connect the external power supply ground, the Mega ground, and the FM225 ground together).⚠️ Critical Reminder on Power: The HLK-FM225 relies
  on dual binocular cameras and active Infrared (IR) floodlights to calculate depth mapping. It draws up to  1A  at a minimum voltage of 5.5V  up to 9V.
  Do not power the sensor from the Mega's 5V pin it will cause the board to brown out instantly. Use a stable external 6V to 9V  regulator or step-up
  converter for the FM225, making sure its ground tie-in links cleanly back to the Mega.

2. Code Paradigm Shift: Parsing the New Data StreamThe old sensor handed you tidy data on command via I2C. The HLK-FM225, by default, streams its
  analytical frames automatically out of its TX line at a default baud rate of 115,200bps.  As it scans, it passes an active serial data structure
  containing internal status flags, face tracking states, and spatial dimensions:Bounding Box: left, top, right, bottomRotation Vectors: yaw, pitch,
  roll To transition the robot's mechanical tracking into your existing loop, you can extract the target position by listening to Serial1 for those coordinate frames.C++


[Animatronic_Alexa_CRT_Robot_HLK-FM225_Eyes.ino] (https://raw.githubusercontent.com/carl1961/Updated-animatronic-Alexa-CRT-robot-Firmware/refs/heads/main/Firmware/Animatronic_Alexa_CRT_Robot_HLK-FM225_Eyes/Animatronic_Alexa_CRT_Robot_HLK-FM225_Eyes.ino)
