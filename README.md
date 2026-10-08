# DIY ESP32 Hotplate Mod

This is my DIY ESP32 controller/mod for a cheap generic ~$50 hotplate from Amazon.  
Nothing fancy, just a straightforward project with two boards: a control PCB and a power PCB.

![PCB preview (control + power boards)](https://github.com/user-attachments/assets/a2db4533-79bb-41f7-a2c2-90eab3b6dba0)

## What’s included

- EasyEDA project file:  
  [PCB/EasyEDA_Diy_Hotplate.epro2](PCB/EasyEDA_Diy_Hotplate.epro2)
- Gerber files:
  - [PCB/Gerber/Control_PCB.zip](PCB/Gerber/Control_PCB.zip)
  - [PCB/Gerber/Power_PCB.zip](PCB/Gerber/Power_PCB.zip)
- BOM files:
  - [PCB/BOM/BOM_Control_PCB.csv](PCB/BOM/BOM_Control_PCB.csv)
  - [PCB/BOM/BOM_Power_PCB.csv](PCB/BOM/BOM_Power_PCB.csv)

The ESP32 code is in the `Code` folder.

## Safety note

This is a DIY mains-power project, so please double-check your wiring and power safety before building or testing.
