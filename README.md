# TurtleBoat
Software-in-the-loop-simulator emulating a robotic ship on a ROS 1 network

Developed on ROS Noetic

Initial developer: Bart Boogmans (bartboogmans@hotmail.com)
 
![image](https://user-images.githubusercontent.com/5917472/204604860-5a0f899e-1df0-4577-9d4f-759c835b8c75.png)

Default dynamics represent a Tito Neri model scale vessel. Initial parameters of the Tito-Neri vessel line are obtained though the publication:
 "Model predictive maneuvering control and energy management for all-electric autonomous ships" https://www.sciencedirect.com/science/article/pii/S0306261919309705
 
Missing parameters (such as dampening coefficients & non diagonal components of the inertial matrices) were estimated using linearization in characteristic working points or free-body-diagram supported estimates. The values are assumed to be quite reasonable, although a difference between simulation and reality is unavoidable. The signs and general behaviour of the ship dynamics which are herein presented should however give a good feel of how these vessels respond, allowing coarse tests of control algorythms and learning to interact with vessel control systems over ROS.

Notice:
This work can be copied, changed and redistributed as long as documentation remains clear on contributions of origonal contributors. 
This work can be used to generate content (such as: datasets, figures, model-parameters) for publications (including education deliverables such as msc thesis) given that explicit recognition has been given how this tool and the developers of this work contributed to the resulting work. 

## Use:
After installing ROS, cloning this repo & setting ROS environment variables:
```shell
cd repo_location/TurtleBoat
source devel/setup.bash
rosrun turtleboat main.py boat1 --velocity0 0.3 0 0 0 0 0.05
```

This should yield periodic information of the rates of core functionality of the simulator and related topics. 

Now that the simulation runs we can look at active topics that we can interact with:
```
bart@bart-P5820T:~$ rostopic list
/boat1/diagnostics/sim_state/actuation
/boat1/diagnostics/sim_state/actuationReference
/boat1/diagnostics/sim_state/f_actuator
/boat1/diagnostics/sim_state/f_corioliscentripetal
/boat1/diagnostics/sim_state/f_drag
/boat1/diagnostics/sim_state/f_res
/boat1/diagnostics/sim_state/velocity
/boat1/reference/actuation
/boat1/reference/actuation_prio
/boat1/state/geopos
/boat1/state/yaw
```

The main topics of interest are the reference and state groups. The simulator executes any actuator reference published on the reference topics, giving priority to actuation_prio. The resultant pose is published on in the state topics as geographical coordinate (/geopos) and heading (/yaw)

This gives us an input-output model that now interacts with other nodes that attempt to control this virtual ship through the shown topics. 

The diagnostics section gives various information of current simulation status. These are internal states of the simulator which would be 'ground truth' with a real ship, and normally we cannot access them. They are included for diagnostics purposes, and only need to be looked at if one is particularly interested in the internal states. 

## Dynamics
A 6DOF state space model has been used to simulate motion, although for the example Tito-Neri vessel line only surface level dynamics are taken into account, although the framework supports full motion.

### Coriolis & centripetal forces
Parameterization of C matrices is done according to eq3.46 from "Handbook of marine craft hydrodynamics and motion control" 2011 by Fossen.

```math
C(ν) = \begin{bmatrix} 0_{3x3} & -S(M_{11}ν_1 + M_{12}ν_2) \\ -S(M_{11}ν_1 + M_{12}ν_2) & -S(M_{21}ν_1 + M_{22}ν_2) \end{bmatrix} 
```
with similar shape for added mass contributions

### Simulation rates
Rates of publishing position & heading and doing simulation steps can be individually configured. Increased simulation rate positively affects accuracy of the model, where significant errors were observed at 2hz and neglectible at 200hz, where the latter did not prove computationally challenging for a common pc, and is thus recommended.
Evaluating kinetic energy (Kirchhoff's equations of energy, 1869, or Fossen's handbook of marine craft hydrodynamics eq 6.34 - shown below) in the simulator allows checking whether the undampened system is conservative. With dampening and external forces set to zero, this should remain constant. If it is not, errors expected due to: non infinitedecimal timestep (set simfrequency higher), rounding errors. 

$$ T = {1 \over 2} {v^\intercal Mv } $$

with a 2hz timestep this occasionally resulted in volatile (unstable due to positive feedback) simulation results (within 15 simulation seconds). Raising to 200hz solved this (increase of potential was still there but orders of magnitude lower than dampening effects). 

### Thruster model
Actuators follow simplistic responses modeled with an absolute rate limiter. The image below shows as an example the propeller velocity reference (green) followed slightly delayed by the modelled behaviour (red)
![image](https://user-images.githubusercontent.com/5917472/230133639-ea5e6c15-79bb-46af-9b8a-8fed1e0bf2d8.png)
The magnitude of the latency is configurable. Current values are based upon earlier studies of the Tito Neri low level control system. 


