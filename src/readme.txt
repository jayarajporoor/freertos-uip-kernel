Jayaraj Poroor: Modified FreeRTOS/UIP code that integrates both (uip/socket.c, uip/uip_task.c) and adds support for our ARM LPC board (see platform/). Adds socket-layer interface to UIP accessible from FreeRTOS tasks.

Note: Contains FreeRTOS and UIP code!

Each real time kernel port consists of three files that contain the core kernel
components and are common to every port, and one or more files that are 
specific to a particular microcontroller and or compiler.

+ The FreeRTOS/Source directory contains the three files that are common to 
every port.  The kernel is contained within these three files.

+ The FreeRTOS/Source/Portable directory contains the files that are specific to 
a particular microcontroller and or compiler.

+ The FreeRTOS/Source/include directory contains the real time kernel header 
files.

See the readme file in the FreeRTOS/Source/Portable directory for more 
information.