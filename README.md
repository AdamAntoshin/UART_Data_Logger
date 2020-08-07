# UART Data Logger

A basic Feather M0-based UART data logger. It can record 2 lines of UART communication, which lets you log any bi-directional serial communication line between two devices. Logs are saved in an SD card in the form of an indexed text file.

Each log is formatted in a clear and readable fashion — each line is seperated by a carriage-return and starts with a line identifies (RX/TX) and a basic time stamp (seconds after file creation). A new file in the form of "LOG_XXXX.TXT" is created after every boot.

Code tested on a 9600 BPS UART channel between a micro-controller and a modem module (industrial product). 
