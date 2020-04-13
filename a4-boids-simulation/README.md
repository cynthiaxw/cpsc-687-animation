CPSC687 - A4
-------------
Name: Xi Wang
UCID: 30057535


Compilation
-----------

## Linux
### How to Install Dependencies (Ubuntu)

    sudo apt install cmake build-essential

### How to Build

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
    cmake --build build

### How to Run

    build/simple

## Windows
### Make a build folder

    mkdir build

### How to Build

    cd build
    cmake ..

### Open the project file in Visual Studio, build and run.


References
----------
GIVR API: https://lakin.ca/givr/.
Boilerplate code: from tutorial resources (via D2L).
Compilation instructions: https://github.com/giv-lab/givr/tree/v0.0.11. 
Model source: https://free3d.com/3d-model/rainbow-trout-v1--202714.html