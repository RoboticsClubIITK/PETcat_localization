Introduction
=============
Object Avoidance using the method of AFPFM(Advanced Fuzzy potential Field Method).
This repo is derived from [here](https://github.com/banuprathap/PotentialFieldPathPlanning).The potential of the planner function is modified to suite our need.

Overview
=========

The project 's objective is to implement a simple path planner that interfaces with the Robot through the code API provided by the simulator and reaches the goal using AFPFM. The main advantage of this technique can be attributed to the fact that this can be extended to any robot by simply changing  the simulator.


Algorithm:
- AFPFM

Potential Fields
=============
We use AFPFM for assigning potential to the goal position and various obstacles using fuzzy rules.
The exact AFPFM algorithm can be accessed from [here](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwi95oK4gunqAhUG63MBHdivCCoQFjACegQIARAB&url=https%3A%2F%2Fwww.researchgate.net%2Fpublication%2F299523529_Advanced_Fuzzy_Potential_Field_Method_for_Mobile_Robot_Obstacle_Avoidance&usg=AOvVaw3ZdAeVdwBSI7_KstFHIpWu)



Future Work
------------
1.Implement the algorithm for a qudraple in gazebo.



License
===

This program is under MIT License. A copy of the license can be obtained from [here](https://github.com/banuprathap/PotentialFieldPathPlanning/blob/master/LICENSE)

Copyright (c) 2017 Banuprathap Anandan
```bash
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```


 Dependencies
=======

Run the following code to install the dependencies for this project
```bash
sudo apt-get update 
sudo apt-get install build-essential
sudo apt-get install freeglut3-dev
```

Build Instructions
========

- Checkout the repo (and submodules)
```bash
$ git clone --recursive https://github.com/banuprathap/PotentialFieldPathPlanning.git
cd PotentialFieldPathPlanning
mkdir -p build && cd build
cmake ..
make
```

Running the demo
=============


- To start the program, in your build directory

```bash
./app/demo
```




Running the unit tests
=============

- In your build directory

```bash
./test/cpp-test
```


Generating doxygen documentation
====

- In your git home directory

```bash
doxygen Doxygen
```

- Doxygen files will be generated to /docs folder
