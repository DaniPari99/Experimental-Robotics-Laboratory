# Assignment 1

------------------------------------------

Parisi Daniele Martino 4670964

The assignment involves a robot deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.
## The environment
The environment is shown in the following figure:

![Schermata 2022-11-20 alle 12 56 25](https://user-images.githubusercontent.com/62515616/202900566-2f837b84-09f0-47f5-aca4-6c3fca1ee8fa.png)

This is the 2D environment made of 4 rooms and 3 corridors.
The robot starts in the E location and waits until it receives the information to build the topological map,
i.e., the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.

Develop a Finite State Machine based on the SMACH library which represents the robot motion in the environment by respecting the above specifications.

![Diagramma senza titolo drawio](https://user-images.githubusercontent.com/62515616/202901930-51bb1b33-9703-41ac-99ab-55f5fa0ce4e5.png)
