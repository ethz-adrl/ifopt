### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__ifopt__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__ifopt__ubuntu_xenial_amd64/)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](http://docs.ros.org/api/ifopt/html/)
[![ROS integration](https://img.shields.io/badge/ROS-integration-blue.svg)](http://wiki.ros.org/ifopt)
![](https://tokei.rs/b1/github/ethz-adrl/ifopt?category=code)
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

</br>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#install">Install</a> •
  <a href="#examples">Examples</a> •
  <a href="#contribute">Contribute</a> •
  <a href="#publications">Publications</a> •
  <a href="#authors">Authors</a>
</p>

## Features
*Combines* the advantages of [Ipopt] / [Snopt] and [Eigen]:

[Ipopt] / [Snopt]  | [Eigen] 
----------|---------
:heavy_check_mark: high-quality solvers for nonlinear optimization  | :heavy_check_mark: modern, intuitive formulations of vectors and matrices 
:x: C++ API inconvenient and error-prone (raw pointers, index management, jacobian construction) | :heavy_check_mark: highly efficient implementations   
:x: linking and exporting difficult  | 

* Solver independent formulation of variables and constraints with Eigen (highly efficient)     
* Automatic index management by formulation of [variable- and constraint-sets](http://docs.ros.org/api/ifopt/html/group__ProblemFormulation.html)  
* Integration: pure cmake `find_package(ifopt)` or [catkin]/[ROS] (optional)       
* light-weight (~[2k lines of code](https://i.imgur.com/NCPJsSw.png)) makes it easy to use and extend    

</br>

An optimization problem consists of multiple *independent variable- and constraint-sets*. Each set represents a common concept, e.g. a set of variables might represents spline coefficients, another footstep positions. Similarly, a constraint-set groups similar constraints together.  `ifopt` allows users to define each of these sets independently in separate classes and then builds the overall problem from these sets. (No more worrying adapting indices when adding or removing sets).

</br>

```
find x0, x1                              (variable-sets 0 & 1)
s.t
  x0_lower  <= x0 <= x0_upper            (bounds on variable-set x0 \in R^2)

  {x0,x1} = arg min c0(x0,x1)+c1(x0,x1)  (cost-terms 0 and 1)

  g0_lower < g0(x0,x1) < g0_upper        (constraint-set 0 \in R^2)
  g1_lower < g1(x0,x1) < g0_upper        (constraint-set 1 \in R^1)
```

</br>

Supplying derivative information greatly increases solution speed. `ifopt` allows to define the derivative of each cost-term/constraint-set with respect to each variable-set *independently*. This ensures that when the order of variable-sets changes in the overall vector, this derivative information is still valid. These "Jacobian blocks" must be supplied through ``ConstraintSet::FillJacobianBlock()`` and are then used to build the complete Jacobian for the cost and constraints.

</br>

<img src="http://docs.ros.org/api/ifopt/html/ifopt.png" height="400" />


 A graphical overview as UML can be seen [here](http://docs.ros.org/api/ifopt/html/inherits.html).



## Install
The easiest way to install is through the [ROS binaries](http://wiki.ros.org/ifopt) and you're all set!
```
sudo apt-get install ros-<distro>-ifopt
```

#### Install dependencies
In case you don't use ROS or the binaries don't exist for your distro, you can easily build these
packages from source. For this, install the required dependencies [Cmake], [Eigen] and [Ipopt] using
```
sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
```
If you want to link to a local installation of [Ipopt] or to [Snopt], see [here](#additional-information).

#### Build with cmake
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
        
#### Build with catkin
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
  add_compile_options(-std=c++11)
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
  
## Examples 
#### Unit tests & toy problem
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

#### towr
A more involved problem, taken from [towr], with multiple sets of variables and constraints to generate motions for legged robots produces the following: 

<img align="center" height="500" src="https://i.imgur.com/4yhohZF.png"/>

## Contribute
We love pull request, whether its interfaces to additional solvers, bug fixes, unit tests or updating the documentation. Please have a look at [CONTRIBUTING.md](CONTRIBUTING.md) for more information. 
See here the list of [contributors](https://github.com/ethz-adrl/ifopt/graphs/contributors) who participated in this project.


## Publications
If you use this work, please consider citing as follows:

    @misc{ifopt,
      author       = {Alexander W Winkler},
      title        = {{Ifopt - A modern, light-weight, Eigen-based C++ interface to 
                       Nonlinear Programming solvers Ipopt and Snopt.}},
      year         = 2018,
      doi          = {10.5281/zenodo.1135046},
      url          = {https://doi.org/10.5281/zenodo.1135046}
    }

The research project within which this code was developed:
* A. W. Winkler, D. Bellicoso, M. Hutter, J. Buchli, [Gait and Trajectory Optimization for Legged Systems through Phase-based End-Effector Parameterization](http://awinkler.me), IEEE Robotics and Automation Letters (RA-L), 2018:


## Authors 
[Alexander W. Winkler](https://www.alex-winkler.com) - Initial Work/Maintainer

This was has been carried out at the following institutions:

[<img src="https://i.imgur.com/aGOnNTZ.png" height="45" />](https://www.ethz.ch/en.html "ETH Zurich") &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/uCvLs2j.png" height="45" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="45" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")



## Additional Information
#### Linking to custom Ipopt or Snopt
If you are building from source and want to use a locally installed version of [Ipopt] add the path to your
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
[catkin]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[catkin command line tools]: http://catkin-tools.readthedocs.io/en/latest/installing.html
[towr]: https://github.com/ethz-adrl/towr
[catkin tools]: http://catkin-tools.readthedocs.org/
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[gtest]: https://github.com/google/googletest


