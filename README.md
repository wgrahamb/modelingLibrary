
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
- Create a new repository. It will contain two main modeling projects, currently
named CPP_6DOF_SRAAM_V2 and PY_5DOF_MOCK_HELLFIRE. Their naming conventions may 
change. My solid rocket booster design project from school will also become a main
project, most likely called SRM_DESIGN. Each project will need a report to detail
its design and results. There will be nothing else, as all the other projects in
this repository are pretty much for development. This repository will become
my private sandbox.

To Do For New Repository: (It is still intended for development)
- CPP_6DOF_SRAAM_V2
     - Clean the configureAndRun script and add target velocity as an input.
     - Add a target update.
     - Re structure the code to make a plainer distinction between "truth" and
     things that are components.
     - Add a rotating elliptical earth. This will require the creation of an 
     "earthTransforms.cpp" which will be a CPP mirror of "earthTransforms.py" that
     can be found in PY_5DOF_MOCK_HELLFIRE.
     - Add a "getState" method.
     - The interface methods could probably use some cleaning.
     - Create fire control and recreate guidance and control. The scope of
     these things will contain: fire control using a three dof, guidance modes,
     waypoint scheduling, and various control theories. Whatever guidance
     is used will have to be applied to the three dof model as well.
- PY_5DOF_MOCK_HELLFIRE
     - Clean comments.
     - Five Dof Dynamics Engine:
          - Make a plain input dictionary.
          - The "construct_msl" method needs to take the classes as an input
          argument. There are a couple of options for this. One is to make the
          four classes each an individual input: atmosphere, aerodynamics,
          mass properties, and motor properties. The other is to make the 
          atmosphere an integral part of the class and only have one input class,
          called something like "mslPhysicalProperties." This class would have to
          have very plainly documented inputs and outputs and would probably use
          a base class of some kind. This way if it was desirable to create another
          missile, this would be THE template. I'm leaning toward the latter option.
          - Mock Hellfire Physical Properties:
               - Model the drag of the missile using the paper that I found.
               - Model the damping of the missile using the paper that I found. Try
               a simple estimate first.
               - Using the solid rocket booster design project, create a higher
               fidelity rocket motor.
     - Components.
          - Create a component base class. Do some research and use __variables.
          Make plain input and output dictionaries for each component.
          - Create a perfect navigator as a pass through placeholder, an analytical
          navigator to test performance, and finally port a hybrid navigator from
          Zipfel code. Finish working through the filtering examples on the website
          that I've bookmarked before porting the Zipfel hybrid navigator.
- SRM_DESIGN
     - Port the python file to C using the manual I got for Christmas.

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
     This bit will include some basic fire control. Fly the missile over various
     ranges, then average the speed to get a table lookup for TGO.
     - Using Zarchan method, write a rate controller. Needs work. If have trouble
     try again with Zipfel's method. Zipfel's rate controller could also work.
- CPP_6DOF_SAM
     - Start project. Reuse as much Zipfel code as possible.
     - Maybe? I'm leaning toward porting ROCKET6G instead.
- PY_5DOF_MOCK_HELLFIRE
     - Dynamics
          - Five Dof Engine
               - Input the classes in the "construct_msl" method.
          - Aerodynamics
               - Five Coefficient Aerodynamics:
                    - Input:
                    - Output:
                         - Aerodynamic Forces:
                              - CX (Axial Coefficient)
                              - CZ (Normal Force Coefficient)
                              - CM (Pitching Moment Coefficient)
                              - CY (Lateral Force Coefficient)
                              - CN (Yawing Moment Coefficient)
                         - Aerodynamic Derivatives:
                              - CZA (Normal Force Coefficient due to Alpha)
                              - CZD (Normal Force Coefficient due to Deflection)
                              - CYB (Lateral Force Coefficient due to Sideslip)
                              - CYD (Lateral Force Coefficient due to Deflection)
                              - CMA (Pitching Moment Coefficient due to Alpha)
                              - CMD (Pitching Moment Coefficient due to Deflection)
                              - CNB (Yawing Moment Coefficient due to Sideslip)
                              - CND (Yawing Moment Coefficient due to Deflection)
          - Mass Properties
               - Input:
               - Output:
                    - Center of Gravity
                    - Missile Mass
                    - Lateral Moment of Inertia
          - Motor
               - Input:
               - Output:
                    - Thrust
          - Atmosphere
               - Input:
                    - Altitude
                    - Speed
               - Output:
                    - Air Density
                    - Dynamic Pressure
                    - Air Pressure
                    - Speed of Sound
                    - Acceleration due to Gravity
                    - Mach
     - Components
          - Component Base Class
               - Specifically needs to define the time behavior.
               - Use a method to get next time, don't grab it directly.
          - Actuators
               - Input:
                    - Fin Command
               - Output:
                    - Fin Deflection
          - Guidance
               - Input:
                    - Missile Local to Body DCM
                    - Missile Local Position
                    - Missile Local Velocity
                    - Target Local Position
                    - Target Local Velocity
               - Output:
                    - Normal Command
                    - Lateral Command
          - Control
               - Input:
               - Output:
                    - Pitching Fin Command
                    - Yawing Fin Command
          - Navigator
     - Make a base class for the components. Learn about "private" variables in PY.
     - Output structures for components?
     - Make the five dof dynamics a clear engine, not just for a mock hellfire. It
     should be plainly generic and the inputs and outputs for its components very
     clearly labeled.
     - Turn enu to ecef into a method to put in earthTransforms.py.
     - Split dynamics and components in to seperate folders.
     - Make aerodynamics its own class within dynamics. Use the found paper to 
     estimate pitch damping (simple estimate at first).
     - Axial Forces:
          - Use bookmarked article to model the drag.
          - Using final project from MAE540, design a real rocket motor. Then the
          mass and motor properties will have to be split.
     - Navigator:
          - A perfect navigator for setting up a pass through.
          - An analytical navigator with draws.
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
     - Port rocketPropulsionFinal.py to C and make it a main project.

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






















