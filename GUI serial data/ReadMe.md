========================================================================
README - Serial Port GUI, Data Logger & Visualizer Tool
========================================================================

Overview
--------
This folder contains various Python-based Graphical User Interface (GUI) 
scripts designed to read, log, visualize, and plot serial data (such as 
UWB, microcontrollers, Arduino, etc.) from COM / serial ports.

While several older prototype versions and experimental scripts are included 
in this folder for historical reference, the main, fully functional 
production version to use is:

    -> Serial_Lab.py


Folder Contents
---------------
- Serial_Lab.py                    [RECOMMENDED] Main working GUI tool
- uwb_serial_plotter_logger.py     Prototype script for UWB plotting/logging
- uwb_serial_plotter_logger_v2.py  Updated prototype for UWB serial plotting
- PUTTY_1.py                       Early PuTTY-style serial viewer prototype
- my_own putty_v1.py               PuTTY alternative prototype v1
- my_Putty_v2.py                   PuTTY alternative prototype v2
- ReadMe.md                        Markdown documentation
- README.txt                       This file


Getting Started
---------------

1. Prerequisites & Dependencies:
   Before running the script, ensure you have Python 3 installed. You will 
   need to install the required dependencies using pip.

   Open your terminal/command prompt and run:

       pip install pyserial

   Note: Built-in Python modules used by the script include:
   - tkinter (GUI framework)
   - threading (Multi-threading support)
   - queue (Thread-safe data queues)
   - re (Regular expressions)
   - datetime (Timestamping)
   - collections (Deque data structure)
   - statistics (Data calculations)
   - os (Operating system interfaces)

2. Running the Application:
   To start the main application, launch 'Serial_Lab.py' using Python:

       python Serial_Lab.py


Features of Serial_Lab.py
--------------------------
- Auto-detects available serial/COM ports.
- Configurable baud rate and connection settings.
- Real-time data streaming and auto-scrolling log output.
- Data logging to local files with timestamping.
- Plotting and statistical summary tools.
========================================================================