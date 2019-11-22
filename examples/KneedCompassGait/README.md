# the kneed compass gait low level control part  

## KneedCompassGait.urdf
    the description file of the kneed compass gait

## rigid_body_tree_construction.cc
    this file is used to construct the terrain of the simulation, rather than construct the rigid body tree of kneed compass gait.

## test.cpp
this is the simulation file to generate the trajectory.  
it links with **linear_system.h** and **util.h**.
### linear_system.h
    This file is used to define the *dynamics* of the inverted pendulum model, then basing on this model, iterate to *generate the trajectory*.

### util.h
    defines some date types which can be used in **linear_system.h** 
### How to put disred trajectories into txt file?
    Basing on the linear_system.h file, we can only output one txt file at a time.  
    - uncomment the corresponding lines of outputing trajectory in linear_system.h file
    - run the test.cpp simulation but in this way: bazel run //examples/KneedCompassGait:test >> data.txt
    - move the data.txt file to the KneedCompassGait folder and replace the former file
    **Attention**: using pipe will not overwrite the original file, which means you need to delete the last-time-output file before run this simulation

## qp_multi_step.cpp
this is the final QP controller simulation file.  
This binary file mainly links with **KCG_common.h** and **qpController2.h**
### KCG_common.h
    this header file connect with **KCG_common.cpp** source file, they together provide *initia state* of *kneed compass gait* and some *contact parameters* setting.  
### qpController2.h
    this header file connect with **qpController2.h** source file, they togther consist the *QP controller*.

## rdSimulate.cpp
This is a passive simulation binary file.  
this is always used for *visualize the initial* state or some other things.  

## doIK.cpp, multibodyKCG.cpp and multibodyKCG.h  
not useful for now, just some tutorial files I wrote.  
these files are wrote for compute the initial state of kneed compass gait at the beginning but depreacated afterwards, but there are some details that can be referenced, if you want to try to use inverseKinematics in drake.  
