DUNE: Unified Navigation Environment
======================================

DUNE: Unified Navigation Environment is a runtime environment for unmanned systems on-board software. It is used to write generic embedded software at the heart of the system, e.g. code or control, navigation, communication, sensor and actuator access, etc. It provides an operating-system and architecture independent platform abstraction layer, written in C++, enhancing portability among different CPU architectures and operating systems.

[![Build Status](https://travis-ci.org/LSTS/dune.svg?branch=master)](https://travis-ci.org/LSTS/dune)
[![Build status](https://ci.appveyor.com/api/projects/status/tdcdgyf408u4y0ng?svg=true)](https://ci.appveyor.com/project/zepinto/dune) 

======================================
## TTK22 project:

### Compilation instructions
The compilation instructions follow the lecture notes.
1. Requirements:
- gdb, make, git
2. Create working directory. Recommended folder structure:  
<pre>
|dune  
|  |-source
|  |-build
</pre>  
2. Clone repository into dune/source folder
3. Build the project by running the following lines of code
   ```
   cd dune/build
   cmake ../source
   make -j4
   ```

### Run simulation
To run a simulation, write the following commands in the terminal
```
cd dune/build
./dune -p Simulation -c lauv-xplore-1
```
