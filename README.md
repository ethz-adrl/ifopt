### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/ifopt/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/ifopt/job/master/) [<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/api/ifopt_core/html/index.html)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

*A modern, light-weight, [Eigen]-based C++ interface to Nonlinear Programming solvers, such as [Ipopt] and [Snopt].*

:heavy_check_mark: Related variables and constraints are implemented (grouped) in *independent sets*. Ifopt automatically generates the overall problem from these sets. No more changing indices in your variable vector or Jacobian when adding or removing variables/constraints. See this large [problem](https://i.imgur.com/4yhohZF.png), that requires multiple variable- and constraint sets to generate the motion for legged robot (implemented in [towr]).

More Features:  
:heavy_check_mark: [Eigen] allows inuitive formulation and fast performance due to sparse matrix exploitation.  
:heavy_check_mark: exports cmake scripts to easily `find_package(ifopt)` in your project.  
:heavy_check_mark: [catkin] integration (optional).  
:heavy_check_mark: light-weight (~[1k lines of code](https://i.imgur.com/NCPJsSw.png)) makes it easy to use and extend.  


**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")



## <img align="center" height="15" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies
Name | Min. Ver. | Description | Install
--- | --- | --- | --- |
[CMake] | v3.1.0 | C++ build tool | ```sudo apt-get install cmake```
[Eigen] | v3.2.0 | Library for linear algebra | ```sudo apt-get install libeigen3-dev```
([Ipopt]) | v3.11.9 | NLP solver (Interior-Point) |```sudo apt-get install coinor-libipopt-dev```
([Snopt]) |  7.4  |  NLP solver (SQP) | non-free

####  Setting up NLP solvers
Install one or both NLP solvers to solve the optimization problem.
If [Ipopt] was installed through the the package manager or installed into the standard search paths,
you're all set! But if you want to link to a local installation of Ipopt or to 
Snopt, see this [section](#solver-install).

## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building with cmake
* Install
  ```bash
  git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
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
        
## <img align="center" height="15" src="https://i.imgur.com/x1morBF.png"/> Building with catkin
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

If you have IPOPT installed and linked correctly, you can run this binary [example](ifopt_ipopt/test/ex_test_ipopt.cc) through
```bash
./build/ifopt_ipopt/ifopt_ipopt-example # or `rosrun ifopt ifopt_ipopt-example `
```

```c++
#include <ifopt/problem.h>
#include <ifopt/ipopt.h>
#include <ifopt/test_vars_constr_cost.h>

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

[ifopt_core/test/test_vars_constr_cost.h](ifopt_core/test/test_vars_constr_cost.h)


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



## <a name="solver-install"></a> Installing and linking solvers
If you want to use a locally installed version of IPOPT add the path to your
Ipopt build folder to your `~/.bashrc`, e.g.
```bash
export IPOPT_DIR=/home/your_name/Code/Ipopt-3.12.8/build
```

In case your OS doesn't provide the precompiled binaries or the required version,
you can also easily install Ipopt from source as described [here](https://www.coin-or.org/Ipopt/documentation/node14.html). This summary might work for you:
```bash
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.10.zip
unzip Ipopt-3.11.10.zip
cd Ipopt-3.11.10/ThirdParty/Mumps
./get.Mumps  # HSL routines are faster (http://www.hsl.rl.ac.uk/ipopt/)
cd ../../
mkdir build && cd build
../configure --prefix=/usr/local
make
make test
make install
export IPOPT_TEST=`pwd`
```

If you need an interface to [Snopt], point cmake to that build folder in your `~/.bashrc` through e.g.
```bash
export SNOPT_DIR=/home/your_name/Code/Snopt
```
and run cmake as 
```bash
cmake -DBUILD_SNOPT=ON ..
```

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


