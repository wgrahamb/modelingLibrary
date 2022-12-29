
Use:
- This repository is intended to be run from a linux terminal. If you wish
to use a project on Windows, I suggest isolating it and tailoring it
from there. Visual Studio should support the CMake for all of these
projects with minimal effort. Any input path nested within the code
will need to be modified.
- Anything that can run is designed to run from the parent directory.
- All PY scripts require a python interpreter and a few pip installs.
Start with numpy, pandas, and matplotlib.
- All CPP projects are configured by CMake and require a compiler.
Build scripts written in python are included.

To Do:
- Might pair down the repository and find a better way of presenting it. I think
that it is bloated as is. I want CPP_6DOF_SRAAM_V2 and PY_5DOF_MOCK_HELLFIRE to 
be the main modeling presentations and then I want to start showcasing my rocket
propulsion projects. As far as presenting it, will have to think on how best to
present the work I have done.
- CPP_6DOF_SRAAM_V2
     - Update the readme to point out how my
     programming skills have changed from V1 to V2.
     - Add a rotating elliptical earth.
     - Add a getState method.
     - Add guidance modes to CPP_6DOF_SRAAM_V2. Initial turn, midcourse, terminal.
     This bit will include some basic fire control.
     - Using Zarchan method, write a rate controller. Needs work. If have trouble
     try again with Zipfel's method.
- CPP_6DOF_SAM
     - Start project. Reuse as much Zipfel code as possible.
     - Maybe? I'm leaning toward porting ROCKET6G instead.
- PY_5DOF_MOCK_HELLFIRE
     - Turn enu to ecef into a method to put in earthTransforms.py.
     - Split dynamics and components in to seperate folders.
     - Make aerodynamics its own class within dynamics. Use the found paper to 
     estimate pitch damping.
     - Axial Forces: (Make aerodynamics its own class within dynamics.)
          - Use bookmarked article to model the drag.
          - Using final project from MAE540, design a real rocket motor. Then the
          mass and motor properties will have to be split.
     - Navigator:
          - INS (ADS6) and GPS (not sure which project) from Zipfel.
          - Kalman Filter (from Zipfel) to pair with it after finished
          working through examples. See filters.py in pythonFunctions.
- PY_6DOF_70MM_ROCKET
     - The pitch and yaw aerodynamics do not yet line up with the flight data.
     - Check alpha and beta calculations.
- pythonFunctions
     - Finish working through filters.py
     - Need to derive a class from rocketPropulsionFinal.py called
     solidRocketBooster.py
     - Port rocketPropulsionFinal.py to CPP (C?) and make it a main project.

Folder Structure:

     3DOFS

     Includes multiple three degree of freedom models. Some 3DOF
     translational and some 3DOF side scrollers. Has a base CPP model
     and a base python model intended for development and rapid
     prototype test beds.


     CADAC_SIMS

     REF: Modeling and Simulation of Aerospace Vehicle Dynamics,
     Second Edition, Peter H. Zipfel.

     This is the source code provided with the textbook listed above. It is
     written by Peter H. Zipfel. It is a very good resource, written to
     help students such as myself, learn the complexities of modeling.


     CPP_6DOF_SAM

     This will be a partial port from CADAC_SIMULATIONS ADS6. This is
     a surface-to-air missile. It will start as a Dynamics engine, to
     determine performance of the missile. I have not used quaternions before
     and I will integrate them into this project. It may stay that way. I may
     also write a simple shell around it to use in modeling environments.
     I have not started this project.


     CPP_6DOF_SRAAM_V1

     This is a port from CADAC_SIMULATIONS SRAAM6, Seemingly based on a
     Sidewinder, this is the very first CPP six degree of freedom missile
     model I wrote. I am fond of it. So, it will live here forever. It is
     not very good code or modeling. It does however hit its target, and that
     doesn't happen on accident. It also run very fast and is good for batch
     runs or multi process applications. Included are scripts to run the model.


     CPP_6DOF_SRAAM_V2

     This is a cleaned version of CPP_6DOF_SRAAM_V1, written to be
     easy to integrate into other environments. It has a different motion
     model than V1, one that allows it to fly ballistically. The motion
     model used in V1 tends to harsh alpha rates (it could be wrong), while
     this one stabilizes. It also has a simpler and more barbaric control
     theory. The utility in V1 has been moved to a seperate file. The actuators
     have been re-written and componentized. There are three methods of
     integration. Euler, RK2, and RK4. The logging and visual contain
     higher fidelity. It runs pretty fast, but not as fast as V1. Also
     included is a way to fly the missile in a 3DOF motion model, very useful
     for predictor models. There is a use-case provided in the main function.


     cppFunctions

     Includes multiple CPP functions for modeling, some written by me,
     some written by Zipfel. It also includes any stand alone classes or
     functions that I find useful. It will build as its own project.


     PY_5DOF_AIM

     This is a paired down port from CADAC_SIMULATIONS. It is a bit of
     a kludgy model, but it does work and runs very fast. It would be very
     easy to integrate into other environments. I really don't know what
     real life missile this is supposed to represent, but based on the
     reference characteristics, it may be designed after the old British
     Taildog missile, also called S.R.A.A.M. or Short Range
     Air-to-Air Missile. 


     PY_5DOF_MOCK_HELLFIRE

     REF: Tactical And Strategic Missile Guidance, Third Edition, Paul Zarchan.

     Using Zarchan's method for linearizing an airframe, I made a sketch
     of a mock Hellfire and used those dimensions for calculations. Using
     those estimations for the aerodynamics of the airframe, I then
     wrote a simple script to represent the mass and motor properties
     of tbe mock Hellfire. I then wrapped these things in a Dynamics
     constructor and a Dynamics flight simulator. This yielded a fast and
     portable mock Hellfire Dynamics engine. It takes a flight duration and 
     fin deflections as an input.


     PY_6DOF_70MM_ROCKET

     I found this paper early in my career. It was released by the
     government for public use. It gives enough data of a 70MM HYDRA rocket
     to model its dynamics. The paper and its transcribed data are included.
     It is a challenging model for a couple of reasons. First, the rocket
     is not ballistically stable and suffers wobble at the end of its flight.
     Second, the rocket motor and fins provide a rolling moment that changes
     characteristics, depending on the speed of the rocket.


     PY_6DOF_SRAAM

     This is the python port of the CADAC_SIMULATIONS SRAAM6. I wrote this
     sim before the CPP version. It has gone through a fair amount of revision
     since its beginning. It uses the same motion model as CPP_6DOF_SRAAM_V1.
     It runs pretty fast considering it is a good bit of code. It would be
     very easy to integrate into other environments.


     pythonFunctions

     Includes multiple python functions for modeling. Most of them are
     written by me, but some of it comes from elsewhere.






















