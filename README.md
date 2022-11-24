# Nausbot
A simple discrete vessel simulator over ROS


## Dynamics

### Coriolis centripetal matrix
Parameterization is done according to eq3.46 from "HANDBOOK OF MARINE CRAFT HYDRODYNAMICS AND MOTION CONTROL" 2011 by Fossen.
![image](https://user-images.githubusercontent.com/5917472/203874227-343b1f4f-487d-4a58-9a69-7624b5a4dfad.png)

Evaluating kinetic energy in the simulator allows checking whether the undampened system is conservative (which it should be)

With dampening and external forces set to zero, this should remain constant. If it is not, errors expected due to: non infinitedecimal timestep (set simfrequency higher), rounding errors. 
with a 2hz timestep this occasionally resulted in volatile simulation results (within 15 simulation seconds). Raising to 200hz solved this (almost no energy gain). 

![image](https://user-images.githubusercontent.com/5917472/203874598-a0b453d4-6fc5-474b-8f4d-48af5066ca89.png)
