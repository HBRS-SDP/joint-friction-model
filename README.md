# ws21-joint-friction-model
(WS 2021)
## Contributors
### Members
* Natalia Quiroga
* Ritwik Sinha
* Muhammad Qadeer Altaf

### Coach
* Djordje Vukcevic

## Description
This repository is a submission for MAS Software Development Project. This project is about estimating a friction model for the
Kinova Gen3 Arm.
The software works with the robot in offline mode and can identify the amount of friction there is in the different joints of the arm of the robot.
The software renders the different parameters and the curve of the friction model which won't take temperature into consideration.
Depending upon specific custom applications, the user may utilise this information.

## Usage
To run the main_kinova file run these commands on terminal in the project directory.

```
$ mkdir build
$ cd build
$ cmake ..
$ make
$./main

```
## Requirement 
Kinova Api is required in order to work with this project, follow this link and complete all requirements.

```
https://github.com/Kinovarobotics/kortex
```
Install Eigen, a C++ template library for linear algebra
```
https://eigen.tuxfamily.org/index.php?title=Main_Page
```
## Acknoledgements

We would like to thank our project coach MSc. Djordje Vukcevic for introducing and explaining the problem formulation for the project.
His timely guidance and help are the pivot parameters for the motivation to reach the goal of this project.    

## References



