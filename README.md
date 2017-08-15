# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

### Writeup

A MPC Controller was implemented to control steering as well as throttle and braking in an Unity based driving simulator achieving velocities up to 100mph with good and smooth turning behaviour around the track.

The MPC controller was implemented in the file MPC.cpp. The main components of the implementation are a defined cost function to be optimized upon and global kinematic equations including the state variables and constraints. All of these information is then passed to the Ipopt solver which returns optimal steering and throttle values (negative throttle values meaning braking/deaccelarating). We also make use of the CppAD library to compute derivatives. The solver and equations take into account a look ahead which is defined by the number of timesteps N and elapsed time between timesteps dt.

The main program main.cpp handles the communication via uWebSockets with the simulator and calls the MPC controller object to update throttle and steering values and send them back to the simulator. Since latency of actuators impose a big problem in real-life applications we apply an artifficial delay of 100ms in the communication process of actuation values back to the simulator to try to emulate this phenomena. The way I handled the latency was to make a prediction at "t + 100ms" of the values coming from the simulator before passing them to the MPC Solver.

Also at main.cpp we make a transformation from map global coordinates to car centered coordinates, before passing the state variables to the MPC Solver.

## Global kinematic model

Analog to the model proposed in the classroom we make use of the following global kinematic model with the following state variables and control inputs:

[ x_1, y_1, \psi_1, v_1, cte_1, e\psi_1]
State: [x,y,/psi, v]

X and Y being the car position in Cartesian Coordinates, Psi the orientation angle of the car and V the velocity of the car in MPH.

Control Inputs: [Delta, a]
Delta being the steering angle from [-1, 1] and a the throttle/brake value from [-1, 1] also.

We use the following kinematic update equations:

x_{t+1} = x_{t} + v_{t} * cos(psi) * dt

\begin{align}
x_{t+1} & = x_t + v_t * cos(\psi_t) * dt \nonumber \\
y_{t+1} & = y_t + v_t * sin(\psi_t) * dt \nonumber \\
\psi_{t+1} & = \psi_t + \frac{v_t}{L_f} * \delta_t * dt \nonumber \\
v_{t+1} & = v_t + a_t * dt \nonumber \\
cte_{t+1} & = f(x_t) - y_t + v_t * sin(e\psi_t) * dt \nonumber \\
e\psi_{t+1} &= \psi_t - {\psi}des_t + \frac{v_t}{L_f} * \delta_t * dt \nonumber
\end{align}
# Cost function



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

