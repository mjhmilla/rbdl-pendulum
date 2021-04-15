RBDL - Rigid Body Dynamics Library: Pendulum example
Copyright (c) 2017 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>

Licensed under the zlib license. See LICENSE for more details.

0. Pre-requisites

  a. C++ Multibody dynamics code:
        RBDL: https://github.com/ORB-HD/rbdl-toolkit

  b. C++ animation tool:
        rbdl-toolkit: https://github.com/ORB-HD/rbdl-toolkit
  c. C++ Numerical library:
        Boost: http://www.boost.org/
        (On Ubuntu this should already be installed)
  d. A C++ compiler

  e. CMake: https://cmake.org/
  


1. To build:
  a. Make a folder called 'build' in this directory.
  b. From within the build folder run CMake on the CMakeLists.txt file 
  c. You will need to explicitly set the path to the RBDL installation folder. If you have not manually installed RBDL to a specific directory then it may be in usr/local (in Linux).

2. Run the executable from the command terminal by running "./pendulum". You should see output printed to the terminal (below) and these files written in : "data/pendulumDynamicsBoostEnergyError.csv" and "data/pendulumDynamicsBoostMeshupFile.csv"

3.  Open "pendulumDynamics_IntegrationWithBoost.cc" to learn more. Specifically pay attention to:
   a. Reading the model in from "pendulum_luamodel.lua" using command "Addons::LuaModelReadFromFile"
   b. Read the "pendulum_luamodel.lua" to see how a model is specified. Note that in all 6 dimensional vectors the first 3 entries are in the rotational domain (orientations/angular velocities/angular accelerations & torques) and the next 3 are in the linear domain (distances/velocities/accelerations & forces).
   c. Read the code that performs the inverse dynamics analysis. Now using RBDL's doxygen, how would you add an external force to the pendulum?

   d. Read the code that performs the forward dynamics simulation. How would you modify this code to add damping to the rotational joint? For this you will need to look at the wrapper class "rbdlToBoost" that surrounds the model so that boost can numerically integrate it.

   e. Have a look at the typedef statements in the beginning: what type of integrator is being used?

===================================================

Example output from the terminal

===================================================
Constructing 1 dof pendulum
Body mass and geometry properties
Putting bodies in a table
Making a table of joints
Making the meshes
Making the model
DoF: 1
==============================
0. Inverse Dynamics 
==============================
   ts,        q,        qd,       qdd,      tau
0.000000, -1.570796, 0.000000, 0.000000, -9.810000
0.010101, -1.569610, 0.234857, 0.000000, -9.809993


...

==============================
1. Forward Dynamics 
==============================
   t,        q,        qd,       ke,       keRBDL,   keErr,    pe,      peRBDL,   peErr
0.000000, -1.570796, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000
0.010000, -1.570551, 0.049050, 0.002406, 0.002406, 0.000000, -0.002406, -0.002406, 0.000000
0.020000, -1.569815, 0.098100, 0.009624, 0.009624, -0.000000, -0.009624, -0.009624, 0.000000
0.030000, -1.568589, 0.147150, 0.021653, 0.021653, 0.000000, -0.021653, -0.021653, 0.000000
0.040000, -1.566872, 0.196200, 0.038494, 0.038494, 0.000000, -0.038494, -0.038494, 0.000000
0.050000, -1.564665, 0.245249, 0.060147, 0.060147, 0.000000, -0.060147, -0.060147, 0.000000
0.060000, -1.561967, 0.294298, 0.086611, 0.086611, 0.000000, -0.086611, -0.086611, 0.000000
0.070000, -1.558779, 0.343345, 0.117886, 0.117886, 0.000000, -0.117886, -0.117886, 0.000000
0.080000, -1.555100, 0.392390, 0.153970, 0.153970, 0.000000, -0.153970, -0.153970, 0.000000
0.090000, -1.550931, 0.441433, 0.194863, 0.194863, -0.000000, -0.194863, -0.194863, 0.000000
0.100000, -1.546272, 0.490470, 0.240561, 0.240561, 0.000000, -0.240561, -0.240561, 0.000000

....
===================================================


