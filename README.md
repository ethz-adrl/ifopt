### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__ifopt__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__ifopt__ubuntu_xenial_amd64/)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](http://docs.ros.org/kinetic/api/ifopt/html/)
[![ROS integration](https://img.shields.io/badge/ROS-integration-blue.svg)](http://wiki.ros.org/ifopt)
![](https://tokei.rs/b1/github/ethz-adrl/ifopt)
[![CodeFactor](https://www.codefactor.io/repository/github/ethz-adrl/ifopt/badge)](https://www.codefactor.io/repository/github/ethz-adrl/ifopt)
[![License BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://tldrlegal.com/license/bsd-3-clause-license-%28revised%29#fulltext)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

*A modern, light-weight, [Eigen]-based C++ interface to Nonlinear Programming solvers, such as [Ipopt] and [Snopt].*

An example nonlinear optimization problem to solve is defined as:

<img align="center" height="100" src="https://i.imgur.com/YGi4LrR.png"/>

* To see how this problem is formulated, see [*test_vars_constr_cost.h*](ifopt_core/test/ifopt/test_vars_constr_cost.h).   
* Afterwards the problem can be solved using e.g. Ipopt as shown in [*ex_test_ipopt.cc*](ifopt_ipopt/test/ex_test_ipopt.cc).   

Related variables and constraints are implemented (grouped) in *independent sets*. Ifopt automatically generates the overall problem from these sets. No more changing indices in your variable vector or Jacobian when adding or removing variables/constraints. See this large [problem](https://i.imgur.com/4yhohZF.png), that requires multiple variable- and constraint sets to generate the motion for legged robot (implemented in [towr]).

More Features:  
:heavy_check_mark: [Eigen] allows inuitive formulation and fast performance due to sparse matrix exploitation.  
:heavy_check_mark: exports cmake scripts to easily `find_package(ifopt)` in your project.  
:heavy_check_mark: [catkin] integration (optional).  
:heavy_check_mark: light-weight (~[2k lines of code](https://i.imgur.com/NCPJsSw.png)) makes it easy to use and extend.  



## Dependencies
Name | Min. Ver. | Description | Install
--- | --- | --- | --- |
[CMake] | v3.1.0 | C++ build tool | ```sudo apt-get install cmake``` [(upgrade)](https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu#answer-908211)
[Eigen] | v3.2.0 | Library for linear algebra | ```sudo apt-get install libeigen3-dev```
[Ipopt] | v3.11.9 | NLP solver (Interior-Point) |```sudo apt-get install coinor-libipopt-dev```
([Snopt]) |  7.4  |  NLP solver (SQP) | non-free

Quick Install: 

``` sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev```

If you want to link to a local installation of Ipopt or to Snopt, see the [doxygen documentation](http://docs.ros.org/kinetic/api/ifopt/html/).

  
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
  cd catkin_ws/src
  git clone https://github.com/ethz-adrl/ifopt.git
  cd ..
  catkin_make_isolated # `catkin build` if you are using catkin command-line tools 
  source ./devel/setup.bash
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
  
## Test 
Navigate to your build folder in which the `Makefile` resides, which depends
on how you built the code:
```bash
cd ifopt/build  # plain cmake 
cd catkin_ws/build_isolated/ifopt/devel # catkin_make_isolated
cd catkin_ws/build/ifopt # catkin build
```
Make sure everything installed correctly by running the `test` target
```bash
make test
```
You should see `ifopt_ipopt-example....Passed` (or snopt if installed) as well as `ifopt_core-test` if
[gtest] is installed.

If you have IPOPT installed and linked correctly, you can also run the [binary example](ifopt_ipopt/test/ex_test_ipopt.cc) 
directly (again, first navigate to the build folder with the `Makefile`)
```bash
make test ARGS='-R ifopt_ipopt-example -V'
```
Output:
```bash
1.0 0.0
```

Each set of variables, costs and constraints are formulated by one C++ object
purely through Eigen vectors and matrices and independent from any specific solver.
Although the above [example](ifopt_core/test/ifopt/test_vars_constr_cost.h) adds just one, 
multiple sets of variables or constraints can be added to the NLP and ifopt manages 
the overall variable vector and jacobian, so each set can be implemented independent of 
the others. A more involved problem definition with multiple sets 
of variables and constraints, taken from [towr] can be seen in the following: 

<img align="center" height="500" src="https://i.imgur.com/4yhohZF.png"/>

## Authors 
[Alexander W. Winkler](https://awinkler.github.io/) - Initial Work/Maintainer

This was has been carried out at the following institutions:

[<img src="https://i.imgur.com/aGOnNTZ.png" height="45" />](https://www.ethz.ch/en.html "ETH Zurich") &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/uCvLs2j.png" height="45" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="45" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")


## Publications
If you use this work in an academic context, please consider citing the currently released version as shown [here](https://zenodo.org/record/1135085/export/hx#.Wk4NGTCGPmE)
or the research project within which this code was developed:

A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](https://awinkler.github.io/publications), IEEE Robotics and Automation Letters (RA-L), 2018:


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



[CMake]: https://cmake.org/cmake/help/v3.0/
[Eigen]: http://eigen.tuxfamily.org
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://ampl.com/products/solvers/solvers-we-sell/snopt/
[catkin]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[catkin command line tools]: http://catkin-tools.readthedocs.io/en/latest/installing.html
[towr]: https://github.com/ethz-adrl/towr
[catkin tools]: http://catkin-tools.readthedocs.org/
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[gtest]: https://github.com/google/googletest


