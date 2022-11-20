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

The purpose of the assignment is to develop a Finite State Machine based on the SMACH library which represents the robot motion in the environment by respecting the above specifications.

## Project structure
The following figure shows the software architecture of the assignment:

![Diagramma senza titolo drawio-4](https://user-images.githubusercontent.com/62515616/202909870-3cfa6d0d-1aaa-4e4e-987a-bf8973b7a9f3.png)

As we can see we have 4 nodes, but there is a fifth one: the **helper**, which is not shown, because it is simply an helper interface usefull for the **assignment_fsm** node.
* **assignment_fsm:** is the node which implements the Finite State Machine which drives the robot through the locations of the map according to the stimuli. It uses external function provided by the **helper.py** node.
* **battery_state:** is the node which simulate the battery behaviour. The battery is recharged and run out infinitely. This state publishes on the topic ```/state/battery_low``` the state of the battery: it is a boolean which is **True** if the battery is low and **False** otherwise.
* **controller:** is the server node which simulates the random motion of the robot while it is visiting a location. It only waste time waiting for doing something smarter in the future.
* **armor:** is a server already done used by the ```assignment_fsm``` through the ```helper``` node for doing manipulations or queries on the ontology.
* ****


## Finite State Machine
The following figure shows the states diagram of the Finite State Machine:

![Diagramma senza titolo drawio-3](https://user-images.githubusercontent.com/62515616/202902152-24488445-a19b-4eb3-ab98-8950915526cd.png)

As we can see we have 4 states:
* ```WAIT```: this state is just executed at the beginning and it waits the map ontology to be loaded. As soon as the ontology is loaded, the transition **loaded** is trigguered.
* ```SLEEP```: this state is executed in order to recharge the battery of the robot. As soon as the battery goes high, the transition **rested** is trigguered.
* ```DECIDE```: this state is executed in order to decide the next location to be visited. As soon as the next location is chosen, the transition **decided** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.
* ```VISIT```: this state is executed in order to visit the chosen location. As soon as the chosen location is visited, the transition **visited** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.


