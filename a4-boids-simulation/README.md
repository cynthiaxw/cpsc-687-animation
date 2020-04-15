CPSC687 - A4
-------------
Name: Xi Wang
UCID: 30057535

Program description
-----------
This program has accomplished the basic part of the assignment and bonus part 1 (grid-based optimization). It simulates basic boid behaviours as well as some advanced behaviours (obstacle avoidance and bait ball).

The grid-based optimization uses the technique discussed in the tutorial. Buckets were implemented by <code>std::set</code>. The program is capable of handling over 1000 boids for around 20fps with the help of 12-by-10-by-12 cells. However, when the boids are too clustered (e.g., when simulating the bait ball), the program becomes slow. This is because the complexity is approaching the worst case (n^2) when there are too many boids in the same cell. I think this can be improved by further subdividing the grid (since the grid I am using is too coarse). 

Input text file
-----------
The input text file contains three parts. 
1. Tunable parameters include the number of boids and three coefficients for separation, alignment, and cohesion. 
2. Seeding position, radius and speed for randomly generating the boids. E.g., given a seeding position __p__, a radius __r__, and speed __s__, the boids are randomly created within the ball that centered at __p__ with raius __r__. And the velocity of the boid is also in random direction with the magnitude __s__. 
3. Infinite height obstacle cylinders that are described by three float numbers: the x-z coordinates and radius. 

Controls
-----------
### TAB - swtich between 4 different behaviours, including normal, w/ obstacle, bait ball, bait ball w/ obstacles.

### Mouse - click and drag to change the view.

### Shift + Mouse - zoom.

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