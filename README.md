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
The software can identify the amount of friction there is in the different joints of the arm of the robot.
The software renders the different parameters and the curve of the friction model, which won't consider temperature.
Depending upon specific custom applications, the user may utilize this information.

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

We will like to thank our project coach MSc. Djordje Vukcevic for introducing and explaining the problem formulation for the project.
His timely guidance and help are the pivot parameters for the motivation to reach the goal of this project.
We would sincerely like to express our gratitude towards him for the time and effort he has invested with us on the project.

## References

Golub, G. H., & Van Loan, C. F. (1980). An analysis of the total least squares problem. SIAM journal on numerical analysis, 17(6), 883-893
C. Canudas de Wit, H. Olsson, K. J. Astrom and P. Lischinsky, "A new model for control of systems with friction," in IEEE Transactions on Automatic Control, vol. 40, no. 3, pp. 419-425, March 1995
Sammut, C., & Webb, G. I. (Eds.). (2010). Encyclopedia of Machine Learning. Springer US. https://doi.org/10.1007/978-0-387-30164-8
Ben-Israel, A. (1966). A Newton-Raphson method for the solution of systems of equations. In Journal of Mathematical Analysis and Applications (Vol. 15, Issue 2, pp. 243–252). Elsevier BV




