# INFOMR Project
This repository contains the Multimedia Retrieval system made by Iwan Boksebeld and Alysha Bogaers. Made for the course [Multimedia Retrieval(INFOMR)](http://www.staff.science.uu.nl/~telea001/MR/MR) from the [Department of Information and Computing Sciences](https://www.uu.nl/en/organisation/department-of-information-and-computing-sciences) at [Utrecht University](https://www.uu.nl/).

## Compiling the project
This project is made in C++, thus you will need a C++ compiler. You will also need [Cmake](https://www.cmake.org/) to build the project with correct library binding and include directories.

### Windows
In windows it is possible to use the CMake GUI, setting the source to the root of this repository(the directory containing this file) and setting the build directory to any desired location.
Then click configure and select your compiler and extra options needed for that compiler(like x64/Win32 for Visual studio) and confirm. After configuration is done click generate and open the project.
Then compile the project, remember release mode for a faster resulting program.

### Linux
Run the following commands will compile the project.
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```

## Running the program
The program can be run with the following command line options:
```
-h: The help menu
-p: Preprocess Database
-n: Normalize Meshes
-f: Calculate database feautures
-w: Optimize the weights, not recommended as this is partially done with manual code edits and thus will not work out of the box
-a: Calculate database retrieval performance statistics with ANN. Mutually exclusive with -c
-c: Calculate database retrieval performance statistics with Custom distance metrics. Mutually exclusive with -a
-t: Perform T-SNE calculation
```

## Changing the distance metrics
This application supports several distance metrics in its execution, these are however determined in compile time. Thus to change the distance metrics (un)comment the following lines in [main.cpp](src/main.cpp):

- Line 5 `#define L2` if uncommented L2 is used by the custom distance metric, unless `#define EMD` is uncommented and ANN. Otherwise L1 is used.
- Line 6 `#define EMD` if uncommented EMD is used by the custom distance metric, overriding L2. ANN still uses either L2 or L1.