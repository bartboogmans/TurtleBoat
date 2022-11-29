# Nausbot
Software-in-the-loop-simulator emulating a robotic ship on a ROS network

Initial developer: Bart Boogmans (bartboogmans@hotmail.com)
 
Default dynamics represent a Tito Neri model scale vessel. Initial parameters of the Tito-Neri vessel line are obtained though the publication:
 "Model predictive maneuvering control and energy management for all-electric autonomous ships" https://www.sciencedirect.com/science/article/pii/S0306261919309705
 
Missing parameters (such as dampening coefficients & non diagonal components of the inertial matrices) were estimated using linearization in characteristic working points or free-body-diagram supported estimates. The values are assumed to be quite reasonable, although a difference between simulation and reality is unavoidable. The signs and general behaviour of the ship dynamics which are herein presented should however give a good feel of how these vessels respond, allowing coarse tests of control algorythms and learning to interact with vessel control systems over ROS.

Notice:
This work can be copied, changed and redistributed as long as documentation remains clear on contributions of origonal contributors. 
This work can be used to generate content (such as: datasets, figures, model-parameters) for publications (including education deliverables such as msc thesis) given that explicit recognition has been given how this tool and the developers of this work contributed to the resulting work. 

## Use:
After installing ROS, cloning this repo & setting ROS environment variables:
```shell
cd repo_location/Nausbot
source devel/setup.bash
rosrun nausbot main.py <vesselname> <optional parameters>
rosrun nausbot main.py titoneri1 --velocity0 0.3 0 0 0 0 0.05
```

## Dynamics
A 6DOF state space model has been used to simulate motion, although for the example Tito-Neri vessel line only surface level dynamics are taken into account, although the framework supports full motion.

### Coriolis & centripetal forces
Parameterization of C matrices is done according to eq3.46 from "Handbook of marine craft hydrodynamics and motion control" 2011 by Fossen.

![image](https://user-images.githubusercontent.com/5917472/204147704-3c106978-ce6e-48eb-8dc7-0a3f0bdb95ef.png)

### Simulation rates
Rates of publishing position & heading and doing simulation steps can be individually configured. Increased simulation rate positively affects accuracy of the model, where significant errors were observed at 2hz and neglectible at 200hz, where the latter did not prove computationally challenging for a common pc, and is thus recommended.
Evaluating kinetic energy (Kirchhoff's equations of energy, 1869, or Fossen's handbook of marine craft hydrodynamics eq 6.34 ) (T = 1/2 v'Mv) in the simulator allows checking whether the undampened system is conservative. With dampening and external forces set to zero, this should remain constant. If it is not, errors expected due to: non infinitedecimal timestep (set simfrequency higher), rounding errors. 

$$ T = {1 \over 2} {v^\intercal Mv } $$

with a 2hz timestep this occasionally resulted in volatile (unstable due to positive feedback) simulation results (within 15 simulation seconds). Raising to 200hz solved this (increase of potential was still there but orders of magnitude lower than dampening effects). 
