# the kneed compass gait low level control part  
## doIK.cpp, multibodyKCG.cpp and multibodyKCG.h  
these files are wrote for compute the initial state of kneed compass gait at the beginning but depreacated afterwards, but there are some details that can be referenced, if you want to try to use inverseKinematics in drake.  
## qp_multi_step.cpp
this is the final QP controller simulation file.  
## rdSimulate.cpp
this is always used for testing the initial state or some other things.  
## test.cpp
this is the simulation file to generate the trajectory. it links with linear_system.h and util.h.  
