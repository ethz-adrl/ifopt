### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/ifopt/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/ifopt/job/master/) [<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/api/ifopt_core/html/index.html)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

Ifopt is a unified [Eigen]-based interface to use Nonlinear Programming solvers, such as [Ipopt] and [Snopt]. The user defines the solver independent optimization problem by set of C++ classes resembling variables, cost and constraints. Subsequently, the problem can then be solved with either solver. An example of how this interface can be used to optimize complex motions for legged robots can be seen in [towr].

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")

-------
... also we only need [981 lines of code](https://i.imgur.com/NCPJsSw.png) [(why this matters)](https://blog.codinghorror.com/the-best-code-is-no-code-at-all/) to allow the generation of (1) solver independent problem formulations, (2) automatic ordering of independent variable and constraint sets in the overall problem, (3) Eigen sparse-matrix exploitation for fast performance, (4) implementation of interfaces to Ipopt and Snopt. 


## <img align="center" height="15" src="https://i.imgur.com/fjS3xIe.png"/> Requirements

* [CMake] >= v3.1.0
* [Eigen] >= v3.2.0  (```sudo apt-get install libeigen3-dev```)
* [Ipopt](https://www.coin-or.org/Ipopt/documentation/node10.html) and/or 
  [Snopt](http://www.sbsi-sol-optimize.com/asp/sol_snopt.htm)


## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building
Point cmake to the location of your NLP solvers by modifying the [Findipopt.cmake](cmake/Findipopt.cmake) and/or
[Findsnopt.cmake](cmake/Findsnopt.cmake)
```bash
set(solver_DIR "/path_to_solver_build_dir") 
```

### Building with cmake
* Install
  ```bash
  git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copy files in this folder to /usr/local/*
  sudo xargs rm < install_manifest.txt # in case you want to uninstall the above
  ```

* Test: Make sure everything installed correctly by running
  ```bash
  make test
  ```
  You should see `#1 ifopt_core-test....Passed` as well as one test for each installed solver.
  In case you want to see the actual iterations of the solver, run ``ctest -V``. 

 
* Use: To use in your cmake project, see this minimal *CMakeLists.txt*:
  ```cmake
  find_package(ifopt)
 
  # your function formulating and solving an optimization problem
  add_executable(main main.cpp)
 
  # only command required to pull in include directories, libraries, ... 
  # if only formulating the problem, use ifopt:ifopt_core
  # if solving with IPOPT, use ifopt::ifopt_ipopt
  # if solving with SNOPT, use ifopt::ifopt_snopt
  target_link_libraries(main PUBLIC ifopt::ifopt_ipopt) 
  ```
        
### Building with catkin
* Install: Download [catkin] or [catkin command line tools], then:
  ```bash
  cd catkin_workspace/src
  git clone https://github.com/ethz-adrl/ifopt.git
  cd ..
  catkin_make_isolated # `catkin build` if you are using catkin command-line tools 
  source ./devel/setup.bash
  ```
   
* Test
  ```bash
  rosrun ifopt ifopt_core-test # or ifopt_ipopt-example ifopt_snopt-example
  ```

* Use: Include in your catkin project by adding to your *CMakeLists.txt* 
  ```cmake
  find_package(catkin COMPONENTS ifopt) 
  include_directories(${catkin_INCLUDE_DIRS})
  target_link_libraries(foo ${catkin_LIBRARIES})
  ```
  Add the following to your *package.xml*:
  ```xml
  <package>
    <depend>ifopt</depend>
  </package>
  ```


## <img align="center" height="15" src="https://i.imgur.com/vAYeCzC.png"/> Example
The optimization problem to solve is defined as:

<img align="center" height="100" src="https://i.imgur.com/YGi4LrR.png"/>

Each set of variables, costs and constraints are formulated by one C++ object
purely through Eigen vectors and matrices and independent from any specific solver.
Although this example adds just one, multiple sets of variables or constraints 
can be added to the NLP and ifopt manages the overall variable vector and jacobian, 
so each set can be implemented independent of the others. 

<img align="center" height="300" src="https://i.imgur.com/uzt1N7O.png"/>

> The red values show the initial variables values and constraints violating the bounds. 
> A more involved problem definition with multiple sets 
> of variables and constraints can be seen [here](https://i.imgur.com/4yhohZF.png) (taken from [towr]).

If you have IPOPT installed and linked correctly, you can run this binary [example](src/ifopt_core/test/ex_test_ipopt.cc) through
```bash
./build/src/ifopt_ipopt/ifopt_ipopt-example # or `rosrun ifopt ifopt_ipopt-example `
```

```c++
#include <ifopt/problem.h>
#include <ifopt/ipopt.h>
#include <test_vars_constr_cost.h>

int main() {

  // Define the solver independent problem
  Problem nlp;
  nlp.AddVariableSet  (std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());

  // Choose solver and options
  Ipopt solver; // or Snopt
  solver.linear_solver_ = "ma27";
  solver.tol_           = 0.001;

  // Solve
  solver.Solve(nlp);

  std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
}
```
Output:
```bash
1.0 0.0
```
Take a quick look now at how easily this example problem can be formulated:

[src/ifopt_core/test/test_vars_constr_cost.h](src/ifopt_core/test/test_vars_constr_cost.h)


## <img align="center" height="15" src="https://i.imgur.com/dHQx91Q.png"/> Publications

If you use this work in an academic context, please consider citing the currently released version <a href="https://doi.org/10.5281/zenodo.1135046"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg" alt="DOI" align="center"></a> as shown [here](https://zenodo.org/record/1135085/export/hx#.Wk4NGTCGPmE)
or the project within which this code was developed:
> A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](https://awinkler.github.io/publications), IEEE Robotics and Automation Letters (RA-L), 2018:

    @article{winkler18,
      author    = {Winkler, Alexander W and Bellicoso, Dario C and 
                   Hutter, Marco and Buchli, Jonas},
      title     = {Gait and Trajectory Optimization for Legged Systems 
                   through Phase-based End-Effector Parameterization},
      journal   = {IEEE Robotics and Automation Letters (RA-L)},
      year      = {2018},
      month     = {July},
      pages     = {1560-1567},
      volume    = {3},
      doi       = {10.1109/LRA.2018.2798285},
    }




##  <img align="center" height="15" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/ifopt/issues). This can include a desired interface to another solver, build issues on your machine, or general usage questions.  

[CMake]: https://cmake.org/cmake/help/v3.0/
[Eigen]: http://eigen.tuxfamily.org
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://ampl.com/products/solvers/solvers-we-sell/snopt/
[catkin]: http://wiki.ros.org/catkin
[catkin command line tools]: http://catkin-tools.readthedocs.io/en/latest/installing.html
[towr]: https://github.com/ethz-adrl/towr
[catkin tools]: http://catkin-tools.readthedocs.org/
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Fa2png]: http://fa2png.io/r/font-awesome/link/


