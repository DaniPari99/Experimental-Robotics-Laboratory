# Assignment 1

------------------------------------------

Parisi Daniele Martino 4670964

[code documentation](https://danipari99.github.io/Experimental-Robotics-Laboratory/)

The assignment involves a robot deployed in an indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The purpose of the assignment is to develop a Finite State Machine based on the SMACH library which represents the robot motion in the environment by respecting the above specifications.

## Software architecture
The following figure shows the software architecture of the assignment:

![Diagramma senza titolo drawio-4](https://user-images.githubusercontent.com/62515616/202909870-3cfa6d0d-1aaa-4e4e-987a-bf8973b7a9f3.png)

As we can see we have 4 nodes, but there is a fifth one: the ```helper```, which is not shown, because it is simply an helper interface usefull for the ```assignment_fsm``` node.
* **assignment_fsm:** is the node which implements the Finite State Machine which drives the robot through the locations of the map according to the stimuli. It uses external functions provided by the ```helper.py``` node.
* **battery_state:** is the node which simulates the battery behaviour. The battery is recharged and run out in a random time belonging to the range [15, 40]. This node publishes on the topic ```/state/battery_low``` the state of the battery: it is a boolean which is **True** if the battery is low and **False** otherwise.
* **controller:** is the server node which simulates the random motion of the robot while it is visiting a location. It only wastes time waiting for doing something smarter in the future improvements.
* **armor:** is a server already implemented which is used by the ```assignment_fsm``` through the ```helper``` node for doing manipulations or queries on the ontology.

## Temporal diagram
For sake of completeness the following figure shows the temporal diagram of the software architecture:

![Diagramma senza titolo drawio-5](https://user-images.githubusercontent.com/62515616/202917399-4889196b-1a46-4285-86f9-b455e57d0221.png)

The diagram shows that the ```battery_state``` is always in contact with the ```assignment_fsm``` node through a pub/sub approach, indeed it makes the system going back to the ```assignment_fsm``` immediately in order to make the robot recharging.
The other software componensts, which are servers, are always active, but they work only if the ```assignment_fsm``` sends a request to them.

## States diagram
The following figure shows the states diagram of the Finite State Machine:

![Diagramma senza titolo drawio-2-3](https://user-images.githubusercontent.com/62515616/202918060-40c54de6-60bf-485f-a580-f060d253ae70.png)

As we can see we have 4 states:
* ```WAIT```: this state is just executed at the beginning and it waits until the ontology map to be loaded. As soon as the ontology is loaded, the transition **loaded** is trigguered.
* ```SLEEP```: this state is executed in order to recharge the battery of the robot. As soon as the battery goes high, the transition **rested** is trigguered.
* ```DECIDE```: this state is executed in order to decide the next location to be visited. As soon as the next location is chosen, the transition **decided** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.
* ```VISIT```: this state is executed in order to visit the chosen location. As soon as the chosen location is visited, the transition **visited** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.

For sake of completeness and robustness I also implemented the so called transitions loop: the transition which remains in the current state whenever they are trigguered.

## Installation and running

You need to have aRMOR package cloned in the SRC folder of your workspace, if you don't have follow the steps in the following link:

[aRMOR installation](https://github.com/EmaroLab/armor/issues/7)

Now you need to modify the armor_api folder by adding in the ```armor_manipulation_client``` node the following function:

```
def disj_all_inds(self,ind_list):
        try:
            res = self._client.call('DISJOINT', 'IND', '', ind_list)

        except rospy.ServiceException as e:
            raise ArmorServiceCallError("Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
```
Where ```ind_list``` is the list of all individuals of the ontology.

I did this modification, because I faced some problems with the pre-existed function ```disj_inds_of_class(self, class_name)``` of ```armor_manipulation_client.py```.

### Running with roslaunch
In order to run the application with a launch file you need to install ```x-term``` with the following steps:
```
sudo apt-get update
sudo apt-get -y install xterm
```
Finally you can run the launch file just typing on the terminal:
```
roslaunch assignment_1 assignment_1.launch
```
### Running Without roslaunch
In order to run the application without a launch file you have to follow the following commands:
```
# open a terminal and run:
roscore &
# open a new terminal and run:
rosrun armor execute it.emarolab.armor.ARMORMainService
# open a new terminal and run:
rosrun assignment_1 controller.py
# open a new terminal and run:
rosrun assignment_1 battery_state.py
# open a new terminal and run:
rosrun assignment_1 assignment_fsm.py
```
## Video

In the following gif animated video we can see 4 running terminals which show the overall assignment behaviour:

![assignment_1_completo_2](https://user-images.githubusercontent.com/62515616/204088174-198d4e54-d875-4983-88a7-5e696041c9e3.gif)


* **aRMOR server terminal:** in the top left of the video we can see the aRMOR server in running, waiting for requests coming from other nodes. As soon as a request is sent, we can see that aRMOR will accomplish it. Whenever a manipulation on the ontology is requested or whenever a query request is done, a request to aRMOR is sent.
* **Controller server terminal:** in the top right of the video we can see the controller server waiting for accomplishing the visiting routine in which basically it wastes time.
* **Finite State Machine terminal:** in the bottom left we can see the Finite State Machine node running: at the beginning the finite state machine is in the ```'WAIT'``` state where the topological map is loaded. We can see that it is loaded from the terminal, because the list of all the individuals is printed. When the ontology is loaded the FSM goes in ```'SLEEP'``` state. It remains in this state until the battery becomes fully recharged, in this case the fsm goes in ```'DECIDE'``` state where, according to the 'reachable locations' and the 'urgent' ones, the next location to be visited is decided: in the case of the video C1 is chosen. At the end the FSM goes in ```'VISIT'``` state where the robot arrives in the location chosen (C1) and visits it.
*  **Battery state terminal:** in the bottom right we can see the battery state node which publishes the state of the the battery in the topic ```'state/battery_low'```: 'True' if the battery is low and 'False' if the battery is high.

## Working hypothesis and environment

The environment is shown in the following figure:

![Schermata 2022-11-20 alle 12 56 25](https://user-images.githubusercontent.com/62515616/202900566-2f837b84-09f0-47f5-aca4-6c3fca1ee8fa.png)

This is the 2D environment made of 4 rooms and 3 corridors.

The robot starts in the E location and waits until it receives the information to build the topological map,
i.e., the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.

In order for the reasoner to always discover a consensual ontology to operate with, the environment that is used and initialized must be consistent with the real one.
Additionally, it is also assumed that E, C1, and C2 are all connected. The robot can correctly carry out its monitoring policy in this way.

The robot starts in the recharging location 'E'. When the battery is fully recharged the robot moves randomly along the locations. So it means that it checks among the reachable locations if there are urgent ones. If there are not urgent locations a corridor is chosen randomly, but if there are not corridors among the reachable locations a random location is chosen.

Because of the urgency threshold is set to the default value of 7 seconds and that the battery change behaviour is low and high randomly after an amount of seconds included in the range [15, 40], the robot will not do the movement from C1 to C2 or viceversa. But this does not mean that one of the 2 corridors will never be reached, because once the robot comes back to 'E' for recharging it can randomly chosen one of the 2 corridors. So I Think that it can be a reasonable solution, because all the locations can be visited.


### System’s limitations

* It is assumed that the movement from a location to an other one is an atomic action, so the robot can be only in a location, the precise position in that location is unknown. so it is assumed that the robot has not a particular current position in a room, but it is supposed that the robot is in that room.
* It is also assumed that the 'VISIT' action is just a waste of time.

* Robot cannot do the transition between the 2 corridors.

### Possible technical Improvements

* The system can be improved by implementing a Planner node and a controller node which work cooperatively for simulating the continuous motion of the robot along the locations.
* Also for improving the VISIT state I can implement the 2 cooperative node: planner and controller.

* The third limitation can be solved by increasing the urgency threshold.

## Authors and contacts

[Parisi Daniele Martino](https://github.com/DaniPari99) (4670964)

s4670964@studenti.unige.it

