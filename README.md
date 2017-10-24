Arduino code for controlling the BotBoarduino board and AL5D robot arm.

Uses 'Bare-Arduino-Project' repo as a starting point for the file structure
and Arduino-Makefile functionality.


Instructions for Metal Scanning:
The main.cpp file is set up for using either of the scanning algorithms.
The ShapeMappingScanner is set up by default, but a DefinedAreaScanner is
defined and commented out (switch comments for use of this).
Both scanners require the HardwareSetup to be called initially.  This will
set up all motors and sensor connection.
The reference inductance must be set using the SetReferenceInductance
function, ensure that the sensor is not near a conductive object initially,
as this should be the "normal" sensor state.
The scanner then must use the SetHomePosition to start at home, and a scan
origin must be provided using the SetScanOrigin function.
The scan origin must be above the metal sample for a scan to work.
If using the DefinedAreaScanner, the scan area must also be set.
All of this is set up already in the main.cpp file, but origin and scan
area parameters should be updated as necessary.


Individual Module Use:
The 'al5d' and 'ldc1614' objects are defined as low level base modules, and
may have use in any project using these devices as they are not application
specific here.  See the object functionality if desired.
