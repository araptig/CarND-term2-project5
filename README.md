# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

### Overview & Implementation:

#### a) Model

- The **prediction control model** was used to generate steering angle and the throttle control signals
- The **state** vector of the car has six components as was discussed in class
  - x-position of the car, $x$
  - y-position of the car, $y$
  - orientation of the car, $\psi$
  - velocity magnitude , $v$
  - cross track error, $cte$
  - orientation error, $e\psi$ 


- Prediction & trajectory:

  - Prediction length is tuned with two independent parameters:

    - number of steps, of timestep length,  $N$
    - sample rate, or elapsed duration, $dt$

  - predicted horizon is $T= N \cdot dt$ 

  - the reference trajectory (provided by a map)

    - is transformed from map coordinates to car coordinates using the transformation:

      $$\left(\begin{array}{c} x_c \\ y_c  \end{array}\right) = \left( \begin{array}{cc} \cos(\psi) & \sin(\psi)  \\ -\sin(\psi) & \cos(\psi)  \\ \end{array}\right)\left(\begin{array}{c} x_m -x\\ y_m -y \end{array}\right)$$

      where 

      - $(x_m, y_m)$ is the position of a reference trajectory point in map coordinates
      - $(x,y)$ is the car location in map coordinates
      - $(x_c, y_c)$ is the position of a reference trajectory point in car coordinates 

    - the set of points in the reference trajectory is modeled as a third order polynomial $f(x)$

- Control signals:

  - steering angle, $\delta_k$, constrained to lie between $\pm 25$ degrees
  - acceleration $a_k$, constrained to lie between $\pm 1$ m/s2

- Model :

  - given current state $(x_k, y_k, \psi_k, v_k, cte_k, e\psi_k)^T$ & control $(\delta_k, a_k)^k$, the next state is 

  $$\begin{split} x_{k+1} &= x_k + v_k \cos(\psi_k) dt \\ y_{k+1} &= y_k + v_k \sin(\psi_k) dt \\   \psi_{k+1} &= \psi_k - \tfrac{v_k}{L_f} \delta_k dt   \\ v_{k+1} &= v_k + a_k dt \\ cte_{k+1} &= f(x_k)-y_k + v_k \sin(e\psi_k) dt  \\ e\psi_{k+1} &= \psi_k - \psi_{ref, k} + \tfrac{v_k}{L_f} \delta_k dt\end{split}$$

  where $L_f$ is the length of the car from front to center of gravity

- Cost:

  - the cost associated between a reference and predicted trajectory $k \in \{1,2,\cdots, N\}$, is given by

    $$ J = \sum_{k=1}^N w_{cte}(cte_k - cte_{ref,k})^2 + w_{e\psi}(e\psi_k-e\psi_{ref,k})^2 + w_v(v_k - v_{ref})^2$$

    where

    - the reference cte error is evaluated from reference trajectory polynomial $f(x_k)$ (which will be discussed later) 

      $$cte_{ref,k} = f(x_k)$$  	

    - the reference orientation error is evaluated as

      $$e\psi_{ref,k} = \arctan (f'(x_k)) $$

    - the reference velocity $v_{ref}$ is an input to the model

    - the weights $w_{cte}, w_{e\psi}, w_{v}$ are hyper parameters that assign different weight to different terms

  - additional cost factors include terms that discourage 

    - large control signals

      $$ \sum_{k=1}^{N-1} w_\delta \delta_k^2 + w_a a_k^2 $$


    - and sudden changes in control between consecutive samples

      $$\sum_{k=1}^{N-2} w_{d\delta}(\delta_{k+1} - \delta_k)^2 + w_{da}(a_{k+1}-a_k)^2 $$

    with hyper parameters  $w_\delta, w_a, w_{d\delta}, w_{da}$

- List of hyper-parameters & their final values:

  - polynomial order to fit reference trajectory (chosen 3 as used in class)

  - number of steps, of timestep length,  $N=10$

  - sample rate, or elapsed duration, $dt=0.15$

  - the cost function weights

       $$(w_{cte}, w_{e\psi}, w_{v},  w_\delta, w_a, w_{d\delta}, w_{da}) = (1, 7, 0.1, 0, 0, 0.3, 0)$$

    - it was important to set $w_{e\psi} \gg w_{cte}$, otherwise the car kept overshooting
    - a small $w_v=0.1$ was sufficient to keep velocity close to reference
    - i did not see optimize much the other weights

  - with these settings I could run the car with $v_{ref}=45$ comfortably around the loop

#### b) Time-step Length and Elapsed Duration

- $(N, dt)$ were defined in previous section

- Started with 

  $$(N, dt) = (25, 0.05) $$

  - however the predicted projection was very wobbly (in the absence of a better word). With such large $N$ the algorithm was having difficulty predicting such a long path and the predicted trajectory was exhibiting oscillatory  behavior. I kept reducing $N$ till I ended up at $N=10$
  - $dt$ was tuned after I put latency on, so will defer discussion to that section 

#### c) MPC Preprocessing

- The reference trajectory (provided by a map) 
  - the reference trajectory, which is in map coordinates is transformed to car coordinates using the transformation:
    $$\left(\begin{array}{c} x_c \\ y_c  \end{array}\right) = \left( \begin{array}{cc} \cos(\psi) & \sin(\psi)  \\ -\sin(\psi) & \cos(\psi)  \\ \end{array}\right)\left(\begin{array}{c} x_m -x\\ y_m -y \end{array}\right)$$
    where 

    - $(x_m, y_m)$ is the position of a reference trajectory point in map coordinates
    - $(x,y)$ is the car location in map coordinates
    - $(x_c, y_c)$ is the position of a reference trajectory point in car coordinates 

  - the resulting $N$ points are modeled by as a third order polynomial $f(x)$,  using the provided function *polyfit()*

  - when all the trajectory points are in car coordinates then the first three states also can be represented in car coordinates which simplify to zero

    $$ (x_k, y_k, \psi_k) = (0,0,0)$$

    - in words, using car coordinates, the car is at the origin $(0,0)$ with zero orientation (looking ahead) 

#### d) Latency

- Increasing the elapsed duration $dt$, gave me the most improvement in the presence of latency of $l=0.1$ s 
  - the predicted trajectory  was oscillating for smaller $dt$
  - ended up using $dt=0.15$
  - since $0.15 > 0.10$, the latency becomes less of a nuisance than for smaller $dt$
  - in summary, by choosing $dt>l$, I did not have to make specific  model changes for latency since its impact became small 





## Generic Read-me Content:

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
  is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
  (not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4. Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
