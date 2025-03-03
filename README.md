
# LPV-MPC Controller for Autonomous Vehicle: Trajectory Tracking and Dynamic Control

The LPV-MPC Controller for Autonomous Vehicle project aims to develop a robust control system that allows an autonomous vehicle to follow predetermined trajectories with high precision. Autonomous vehicles operate in dynamic environments, where accurate trajectory tracking is crucial for safety and efficiency. This project uses a Model Predictive Controller (MPC) with a Linear Parameter-Varying (LPV) approach to handle the inherent nonlinearities of vehicle dynamics.

The LPV-MPC framework is particularly suited for autonomous vehicles because it dynamically adjusts the control strategy based on the vehicle’s current operating conditions. By approximating nonlinear behaviors with a set of linear models, the LPV-MPC effectively manages changes in vehicle dynamics, allowing for stable control even under varying conditions. The controller uses a bicycle model for simplified representation of vehicle kinematics, balancing accuracy and computational efficiency. The control inputs for the system are the steering angle and applied acceleration, which are optimized through a cost function that minimizes deviations from the desired trajectory.

The controller’s effectiveness was tested in simulations across three distinct driving scenarios, each with unique trajectory complexities to evaluate performance. In these simulations, the LPV-MPC demonstrated its ability to maintain trajectory adherence, manage dynamic vehicle responses, and respect constraints on control inputs and states. The project’s success highlights the potential of LPV-MPC to enhance the reliability of autonomous vehicle control, making it a promising foundation for further advancements in autonomous driving technologies.


## Installation

**Prequisites**

* Python 3.10.5 

* QP solvers Cvx opt


## Run Locally

Clone the project

```bash
  git clone git@github.com:shivasamkumar/Design_Optimization_Final_project.git
```

Go to the project directory

```bash
  py Main_Mpc.py
```

If it is not working try : 

open Visual studio code and select python kerenel as 3.10.5 and Run the Main file 




## Demo



**Video**

                                      This video shows the Demonstation of the project


[![Watch the video](https://github.com/shivasamkumar/Design_Optimization_Final_project/blob/main/Screenshot%20from%202024-11-07%2011-46-59.png)](https://drive.google.com/file/d/1ShP-VWfv6WIJ6ot2c8PE-dyYAamWYO5u/view?usp=sharing)


                                                       Carla Simulation Track 

![Demo Image](path/to/demo-image.png)  

                                      This video shows the Demonstation of the project in carla simulation 


[![Watch the video]()](https://drive.google.com/file/d/1bnslvg_tduYz1SL6sid344SuI9o-KWxV/view)


## Future versions 
* **Extending the controller to future projects**

    Extend controller testing to real-world autonomous vehicle scenarios for broader validation.Integrate learning-based methods to further enhance adaptability under varying dynamics.
