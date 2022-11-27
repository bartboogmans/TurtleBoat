# Nausbot
Ship simulator / software-in-the-loop-system that listens to actuation and outputs simulated ship response.
Initial developer: Bart Boogmans (bartboogmans@hotmail.com)
 
Initial parameters of the Tito-Neri vessel line are obtained though the publication:
 "Model predictive maneuvering control and energy management for all-electric autonomous ships" https://www.sciencedirect.com/science/article/pii/S0306261919309705
 
Missing parameters (such as dampening coefficients) were estimated using rules of thumb, linearization in characteristic working points or free-body-diagram supported estimates. The values are assumed to be quite reasonable, although a difference between simulation and reality is unavoidable. The signs and general behaviour of the ship dynamics which are herein presented should however give a good feel of how these vessels respond, allowing coarse tests of control algorythms and learning to interact with vessel control systems over ROS.

Notice:
This work can be copied, changed and redistributed as long as documentation remains clear on contributions of origonal contributors. 
This work can be used to generate content (such as: datasets, figures, model-parameters) for publications (including education deliverables such as msc thesis) given that explicit recognition has been given how this tool and by that the developers contributed to the content. 

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
Parameterization of C matrices is done according to eq3.46 from "HANDBOOK OF MARINE CRAFT HYDRODYNAMICS AND MOTION CONTROL" 2011 by Fossen.
![image](https://user-images.githubusercontent.com/5917472/203874227-343b1f4f-487d-4a58-9a69-7624b5a4dfad.png)

Evaluating kinetic energy in the simulator allows checking whether the undampened system is conservative (which it should be)

With dampening and external forces set to zero, this should remain constant. If it is not, errors expected due to: non infinitedecimal timestep (set simfrequency higher), rounding errors. 
with a 2hz timestep this occasionally resulted in volatile simulation results (within 15 simulation seconds). Raising to 200hz solved this (almost no energy gain). 

Kinetic energy is found as follows: (fossen2011)
![image](https://user-images.githubusercontent.com/5917472/203874598-a0b453d4-6fc5-474b-8f4d-48af5066ca89.png)
