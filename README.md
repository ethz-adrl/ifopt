### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__ifopt__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__ifopt__ubuntu_xenial_amd64/)
[![Documentation](https://img.shields.io/badge/docs-generated-green.svg)](http://docs.ros.org/kinetic/api/ifopt/html/)
[![ROS integration](https://img.shields.io/badge/ROS-integration-blue.svg)](http://wiki.ros.org/ifopt)
![](https://tokei.rs/b1/github/ethz-adrl/ifopt)
[![CodeFactor](https://www.codefactor.io/repository/github/ethz-adrl/ifopt/badge)](https://www.codefactor.io/repository/github/ethz-adrl/ifopt)
[![License BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://tldrlegal.com/license/bsd-3-clause-license-%28revised%29#fulltext)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

*A modern, light-weight, [Eigen]-based C++ interface to Nonlinear Programming solvers, such as [Ipopt] and [Snopt].*

Related variables and constraints are implemented (grouped) in *independent sets*. Ifopt automatically generates the overall problem from these sets. No more changing indices in your variable vector or Jacobian when adding or removing variables/constraints. See this large [problem](https://i.imgur.com/4yhohZF.png), that requires multiple variable- and constraint sets to generate the motion for legged robot (implemented in [towr]).

More Features:  
:heavy_check_mark: [Eigen] allows inuitive formulation and fast performance due to sparse matrix exploitation.  
:heavy_check_mark: exports cmake scripts to easily <find_package(ifopt)> in your project.  
:heavy_check_mark: [catkin] integration (optional).  
:heavy_check_mark: light-weight (~[2k lines of code](https://i.imgur.com/NCPJsSw.png)) makes it easy to use and extend.  


## Dependencies
Name | Min. Ver. | Description | Install
--- | --- | --- | --- |
[CMake] | v3.1.0 | C++ build tool | ```sudo apt-get install cmake```
[Eigen] | v3.2.0 | Library for linear algebra | ```sudo apt-get install libeigen3-dev```
[Ipopt] | v3.11.9 | NLP solver (Interior-Point) |```sudo apt-get install coinor-libipopt-dev```
([Snopt]) |  7.4  |  NLP solver (SQP) | non-free

Quick Install: ``` sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev```

> If you want to link to a local installation of Ipopt or to Snopt, see this [section](#solver-install).

## Building with cmake
* Install
  ```bash
  git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in this folder to /usr/local/*
  # sudo xargs rm < install_manifest.txt # in case you want to uninstall the above
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
  # Formulate (ifopt:ifopt_core) and solve (ifopt::ifopt_ipopt) the problem
  add_executable(main main.cpp)
  # Pull in include directories, libraries, ... 
  target_link_libraries(main PUBLIC ifopt::ifopt_ipopt) 
  ```
        
## Building with catkin
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


## Example
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
  solver.linear_solver_ = "mumps";
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
See [here :page_facing_up:](ifopt_core/test/ifopt/test_vars_constr_cost.h) for how easily this example problem is formulated


## Authors 
[Alexander W. Winkler](https://awinkler.github.io/) - Initial Work/Maintainer

This was has been carried out at the following institutions:

[<img src="https://i.imgur.com/aGOnNTZ.png" height="45" />](https://www.ethz.ch/en.html "ETH Zurich") &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/uCvLs2j.png" height="45" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="45" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")


## Publications

If you use this work in an academic context, please consider citing the currently released version as shown [here](https://zenodo.org/record/1135085/export/hx#.Wk4NGTCGPmE)
or the research project within which this code was developed:
- A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](https://awinkler.github.io/publications), IEEE Robotics and Automation Letters (RA-L), 2018:


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


## Contributing
We love pull request, whether its interfaces to additional solvers, bug fixes, unit tests or updating the documentation. Please have a look at [CONTRIBUTING.md](CONTRIBUTING.md) for more information.  

See here the list of [contributors](https://github.com/ethz-adrl/ifopt/graphs/contributors) who participated in this project.


##  Bugs & Feature Requests

To report bugs, request features or ask questions, please have a look at [CONTRIBUTING.md](CONTRIBUTING.md). 



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
export IPOPT_DIR=`pwd`
```

If you need an interface to [Snopt], point cmake to that build folder in your `~/.bashrc` through e.g.
```bash
export SNOPT_DIR=/home/your_name/Code/Snopt
```
and run cmake as 
```bash
cmake -DBUILD_SNOPT=ON ..
```






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


