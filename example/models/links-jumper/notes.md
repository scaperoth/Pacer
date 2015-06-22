Notes for the Jumper
--------------------

### NEW APPROACH
use arbitrary values based on time such as t^20 for x, y, z values. Once the exponent reaches 1, switch
to desired velocites from trajectory calculation

#### Update
use jacobian calculation for the base to get the desired velocity of the foot. The desired foot velocity will come from Jb_transpose * (desired velocity - velocity of the base) = desired velocity of foot. then this is multiplied by
-alpha to and then multiplied again by joint jacobian, Jq, to get the tau, or desired torque of the joints. This force
is then applied to the joint generalized value. 

#### RESULT 
also decent performance acheived visually, but not a principled approach physically. Too many variables need to be changed to update the system

### NEW APPROACH
use spline trajectory planning for desired base velocity and position instead of polynomial calculation from before

#### Update
using three nodes for spline calculation: start, lowest point, and finish. The values placed at these points are experimental at best at this point

### NEW APPROACH
use phases for different parts of the movement to form a state machine. Phases used are LOADED, STARTED, LIFTOFF, and LANDED. Each phase is controlled differently. In LOADED, stand-trajectory -- or, in this project, jumper-planner -- sets the standing position. STARTED is activated when # of contaces, NC, is greater than zero. In STARTED, base velocity and position are determined by spline code. If the current phase is STARTET and the NC is 0, the phase changes to LIFTOFF. In LIFTOFF, the joint-PID controller is started which gets its desired values from the jumper-planner code. If NC > 0 and the phase is LIFTOFF, the phase changes to LANDED. 

#### Update
LIFTOFF gains cause the robot to flip and/or turn violently because it is trying to reach it's desired pose too quickly. A new joint-PID controller class is created for the LIFTOFF phase and the gains for that phase are lowered to slow the movement of the end effectors in the air and reduce the resulting angular momentum of in-air movement.

#### Update
Experiencing problems with spline calculation. The spline starts off correctly then in the middle of the calculation "bumps" up a little bit at what is supposed to be the lowest point of the spline, or the global minimum of the trajectory. This causes undesirable movements in the robot base when trying to perform a jump. 

#### Update
Using three nodes for spline proves to be difficult and unnecessary. switching to two nodes for spline calculation leads to smother movement between start and end points in all directions.

#### Update
code was added to track actual movement and spline calculation

#### RESULT
The robot still has problems during the jumping phase. The comparison of the xyz desired vs actual revealed that the robot was not following the spline trajectory. This is potentially because the robot is only basing its desired movement off of velocity error and gains instead of using the full PID control mechanism. 

### NEW APPROACH
added eef-PID-control to stabilize the robot during the jumping phase

#### Update
experiencing some drift in the x and y directions, but z direction tracks pretty close to spline

#### Update
increased base velocity gains in vars.xml from 1e1 to 2e2 and increase mu-coulumb in model.xml to 100 to decrease slippage. 
Side note: 100 mu-coulumb is full no-slip model in moby enviornment

#### Update
experiencing stiffness in the movement. spline time is set to .5 and the end spline position is set to .4  0 .5. The stiffness is due to there 
being so little "dip" (curve below zero) in the z dimension that the robot does not get go through the proper movement to get the desired jump motion.
Decreasing the z value in the final spline calculation should increase the "dip" motion. 

#### Update 
set the range of the spline to 1 meter and the end of the spline to .4  0 .2 meters for x y z respectively. causes decent performance in overall 
spline movement 

#### Update
Trying to debug the problem with the robot, the body will roll slightly towards its left side causing it to jump in a non-desirable pattern. 
This problem started when adding eef-PID-controller. 

#### RESULT
this approach resulted in an overly complicated system for the current state of discovery. Need to reduce the problem space to include only jumping. 

### NEW APPROACH
Remove phases. Rely on jumping only in jumping controller with eef-PID-control. 

TODO: Simplify problem by "deactivating" liftoff and landing phases. Trying to completely control with end-effector for jump phase
TODO: add xd and xdd to goal.x and goal.xdd if there are contacts
TODO: when not in jumping phase, set goal.x to init.x and xd,xdd to zero
TOD: remove phases for now...

### Update
because i turned off the trajectory planner, I am using the jumper.cpp file to set the goal.x, goal.xd, and goal.xdd for the eef pid controller to use. But since there needs to be number of contacts for the code to calculate the Jacobian of the base, we have set the code to check the number of contacts to make sure there are 4 active contacts. 

### New Approach
was not correctly using the Jacobian base with the velocity of the foot. I was using a more generalized movement from base -> foot -> joints. I did not fully understand the code base that was being used to calculate Jb. 
code needs to be something more like: 
Vft will equal [(N_1...N_nc), (S_1...S_nc), (T_1...T_nc)] s.t.
== > Xd_1 = (N_1, S_1, T_1) 
== > Xd = (Vft[i], Vft[i+NC], Vft[i + NC*2])

### Update
after using this new method, movement is more stable, but the gains need to be updated. 

They are currently set to:

```
Kp =   1e3 1e3 1e3     1e3 1e3 1e3     1e3 1e3 1e3     1e3 1e3 1e3
Kv =   3e1 3e1 3e1     3e1 3e1 3e1     3e1 3e1 3e1     3e1 3e1 3e1   
Ki = 1e-3 1e-3 1e-3   1e-3 1e-3 1e-3  1e-3 1e-3 1e-3  1e-3 1e-3 1e-3
```