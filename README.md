### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/ifopt/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/ifopt/job/master/) [<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/api/ifopt_core/html/index.html)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

Ifopt is a unified [Eigen]-based interface to use Nonlinear Programming solvers, such as [Ipopt] and [Snopt]. The user defines the solver independent optimization problem by set of C++ classes resembling variables, cost and constraints. Subsequently, the problem can then be solved with either solver. An example of how this interface can be used to optimize complex motions for legged robots can be seen in [towr].

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")

-------
... also we only need [981 lines of code](https://i.imgur.com/NCPJsSw.png) [(why this matters)](https://blog.codinghorror.com/the-best-code-is-no-code-at-all/) to allow the generation of (1) solver independent problem formulations, (2) automatic ordering of independent variable and constraint sets in the overall problem, (3) Eigen sparse-matrix exploitation for fast performance, (4) constraint-jacobian and cost-gradient ordering and (5) implementation of interfaces to Ipopt and Snopt. 


## Requirements

* [CMake] 3.1.0 or greater
* [Eigen] 3.2.0 (older might work as well): ```$ sudo apt-get install libeigen3-dev```
* ([Ipopt](https://www.coin-or.org/Ipopt/documentation/node10.html) and/or 
  [Snopt](http://www.sbsi-sol-optimize.com/asp/sol_snopt.htm))


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building
Point CMake to the location of your NLP solvers by modifying the root [CMakeLists.txt](CMakeLists.txt)
```bash
# if folder doesn't exist cmake just ignores that solver
set(IPOPT_DIR "/home/your_name/path_to_ipopt_dir") 
set(SNOPT_DIR "/home/your_name/path_to_snopt_dir")
```

### with CMake
**Install**:
```bash
git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt
mkdir build && cd build
cmake ..
make
# copy files in this folder to
# /usr/local/include/ifopt: headers
# /usr/local/lib: libraries
# /usr/local/shared/ifopt/cmake: find-scripts (.cmake)
sudo make install
  
# in case you want to uninstall the above
xargs rm < install_manifest.txt 
```

**Test**: Make sure everything installed correctly by running
```bash
make test
```
You should see `#1 test-ifopt....Passed` as well as one test for each installed solver.
In case you want to see the actual iterations of the solver, run ``ctest -V``. 

 
**Use**: To use in your cmake project, see this minimal *CMakeLists.txt*:
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
        
### with catkin
**Install**:
Download [catkin] (``sudo apt-get install ros-kinetic-catkin``) or [catkin command line tools] (``sudo apt-get install python-catkin-tools``), clone this repo into your catkin workspace and build:
```bash
cd catkin_workspace/src
git clone https://github.com/ethz-adrl/ifopt.git
cd ..
catkin_make # `catkin build` if you are using catkin command-line tools 
source ./devel/setup.bash
```
   
**Test**: test if solvers where correctly linked through
```bash
rosrun ifopt testifopt # or testipopt, testsnopt
```

**Use**: Included in your catkin project by adding to your *CMakeLists.txt* 
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


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Example
The optimization problem to solve is defined as:

<img align="center" height="100" src="https://i.imgur.com/YGi4LrR.png"/>

If you have IPOPT installed and linked correctly, you can run this binary example through
```bash
./build/src/ifopt_ipopt/testipopt # or `rosrun ifopt testipopt` if built with catkin
```
[src/ifopt_ipopt/src/ipopt_adapter.cc](src/ifopt_ipopt/src/ipopt_adapter.cc):
```c++
#include <ifopt/test/ex_problem.h>
#include <ifopt_ipopt/ipopt_adapter.h>

int main() 
{
  Problem nlp;

  nlp.AddVariableSet  (std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());
  
  IpoptAdapter::Solve(nlp); // or SnoptAdapter::Solve(nlp);
  std::cout << nlp.GetOptVariables()->GetValues();
}
```
Output:
```bash
1.0 0.0
```

The 3 classes representing variables, costs and constraints are defined as 
in the following. The entire following code is independent from any specific solver
and is based purely on Eigen.

[src/ifopt_core/include/ifopt/ex_problem.h](src/ifopt_core/include/ifopt/ex_problem.h):

The variables x0 and x1 with their bound -1 <= x0 <= 1 is
formulated as follows:
```c++
class ExVariables : public VariableSet {
public:
  ExVariables() : VariableSet(2, "var_set1")
  { // initial values
    x0_ = 0.5;
    x1_ = 1.5;
  }

  virtual void SetVariables(const VectorXd& x)
  {
    x0_ = x(0);
    x1_ = x(1);
  };

  virtual VectorXd GetValues() const
  {
    return Vector2d(x0_, x1_);
  };

  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = Bounds(-1.0, 1.0);
    bounds.at(1) = NoBound;
    return bounds;
  }

private:
  double x0_, x1_;
};
```

The example constraint 0 = x0^2 + x1 - 1 is formulated as follows:
```c++
class ExConstraint : public ConstraintSet {
public:
  ExConstraint() : ConstraintSet(1, "constraint1") {}

  virtual VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    g(0) = std::pow(x(0),2) + x(1);
    return g;
  };

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = Bounds(1.0, 1.0);
    return b;
  }

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
      
      jac.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
    }
  }
};
```


The example cost f(x) = -(x1-2)^2 is formulated as follows:
```c++
class ExCost: public CostTerm {
public:
  ExCost() : CostTerm("cost_term1") {}

  virtual double GetCost() const override
  {
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    return -std::pow(x(1)-2,2);
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 0.0;             // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0*(x(1)-2.0); // derivative of cost w.r.t x1
    }
  }
};
```

## <img align="center" height="20" src="https://i.imgur.com/dHQx91Q.png"/> Publications

If you use this work in an academic context, please consider citing the currently released version <a href="https://doi.org/10.5281/zenodo.1135046"><img src="https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg" alt="DOI" align="center"></a> as shown [here](https://zenodo.org/record/1135085/export/hx#.Wk4NGTCGPmE).


##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

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


