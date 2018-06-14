### <img src="https://i.imgur.com/ZOfGZwB.png" height="60" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/ifopt/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/ifopt/job/master/) [<img height="20" src="https://i.imgur.com/ZqRckbJ.png"/>](http://docs.ros.org/api/ifopt_core/html/index.html)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1135046.svg)](https://doi.org/10.5281/zenodo.1135046)
<!-- The actual jenkins documentation job can be found here -->
<!-- http://build.ros.org/view/Ldoc/job/Ldoc__ifopt__ubuntu_xenial_amd64/ -->

Ifopt is a unified [Eigen]-based interface to use Nonlinear Programming solvers, such as [Ipopt] and [Snopt]. The user defines the solver independent optimization problem by set of C++ classes resembling variables, cost and constraints. Subsequently, the problem can then be solved with either solver. This package can be dropped in your [catkin] workspace.

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)**

[<img src="https://i.imgur.com/uCvLs2j.png" height="50" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="50" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")           &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html "ETH Zurich")

-------
... also we only need [981 lines of code](https://i.imgur.com/NCPJsSw.png) [(why this matters)](https://blog.codinghorror.com/the-best-code-is-no-code-at-all/) to allow the generation of (1) solver independent problem formulations, (2) automatic ordering of independent variable and constraint sets in the overall problem, (3) [Eigen] sparse-matrix exploitation for fast performance, (4) constraint-jacobian and cost-gradient ordering and (5) implementation of interfaces to [Ipopt] and [Snopt]. 



## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

This package can be built using either [catkin] or [CMake].

### Generic
* Install [CMake]: ``$ sudo apt-get install cmake``
* Install [Eigen]: ``$ sudo apt-get install libeigen3-dev``   
* Depending on which solver you want to use, install either [Ipopt] or [Snopt]. Follow the instructions provided here:

     * https://www.coin-or.org/Ipopt/documentation/node10.html (open source)
     * http://www.sbsi-sol-optimize.com/asp/sol_snopt.htm

* To build [ifopt_snopt](ifopt_snopt) or [ifopt_ipopt](ifopt_ipopt) set the location of the shared 
libraries and header files directly in the [CMakeLists.txt](https://github.com/ethz-adrl/ifopt/blob/fbf7acda4e3e42711031f65e015f6c9f84c87fbd/ifopt_ipopt/CMakeLists.txt#L16-L17) 
of the corresponding solver.
### Catkin
* Install the cmake build tool [catkin]: ``$ sudo apt-get install ros-kinetic-catkin``.  _     
* Clone this repo into your [catkin] workspace and build

      $ cd catkin_workspace/src
      $ git clone https://github.com/ethz-adrl/ifopt.git
      $ cd ..
      $ catkin_make -DCMAKE_BUILD_TYPE=Release
      $ source ./devel/setup.bash
    

#### <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests

Make sure everything installed correctly by running the unit tests through

    $ catkin_make run_tests
     
This should also solve the [example problem](ifopt_core/include/ifopt/ex_problem.h) with your installed solvers. 
If you have [IPOPT] installed and linked correctly, this should also execute the 
binary [ifopt_ipopt-test](ifopt_ipopt/test/ex_test_ipopt.cc). 

### CMake
* Clone this repo and configure (``cd`` to desired directory before doing the following)

      $ git clone https://github.com/ethz-adrl/ifopt.git
      $ mkdir build ; cd build
      $ cmake -DCATKIN_BUILD=OFF -DCMAKE_INSTALL_PREFIX=path_to_build_dir ..
      
Set ``path_to_build_dir`` to your desired installation target.

* Build

      $ make

* Run unit tests

      $ make test
      
* Install

      $ make install
      
## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage

For an example of how to use this to efficiently generate dynamic motions for legged robots, check-out [towr].
See [test/ex_problem.h](ifopt_core/include/ifopt/ex_problem.h) for a minimal example with detailed comments and explanation
of the below code line by line. 
The optimization problem to solve is defined as:

<img align="center" height="100" src="https://i.imgur.com/YGi4LrR.png"/>

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
in the following. The variables x0 and x1 with their bound -1 <= x0 <= 1 is
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

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/ifopt/issues).

[Eigen]: http://eigen.tuxfamily.org
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://ampl.com/products/solvers/solvers-we-sell/snopt/
[catkin]: http://wiki.ros.org/catkin
[towr]: https://github.com/ethz-adrl/towr
[catkin tools]: http://catkin-tools.readthedocs.org/
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Fa2png]: http://fa2png.io/r/font-awesome/link/
[CMake]: https://cmake.org/


