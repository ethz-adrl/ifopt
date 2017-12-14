### <img src="https://i.imgur.com/ct8e7T4.png" height="80" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_leggedrobotics/xpp/master)](https://ci.leggedrobotics.com/job/github_leggedrobotics/job/xpp/job/master/)

Ifopt is a unified [Eigen]-based interface to use Nonlinear Programming solvers, such as [Ipopt] and [Snopt]. The user defines the solved independent optimization problem by set of C++ classes resembling variables, cost and constraints. Subsequently, the problem can then be solved with either solver. This package can also be used in your [catkin] workspace.

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)** 

This code was developed at the [Agile and Dexterous Robotics Lab](http://www.adrl.ethz.ch/doku.php), ETH Zurich. It is currently maintained at the [Robotics Systems Lab](http://www.rsl.ethz.ch/), ETH Zurich.

[<img src="https://i.imgur.com/uCvLs2j.png" height="60" />](http://www.adrl.ethz.ch/doku.php)  &nbsp; &nbsp; &nbsp; &nbsp;    [<img src="https://i.imgur.com/aGOnNTZ.png" height="50" />](https://www.ethz.ch/en.html)



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Installation

* First you have to install [Eigen]

      $ sudo apt-get install libeigen3-dev
    
* Now, depending on which solver you want to use, install either [Ipopt] or [Snopt]. Follow the instructions provided here:

  * https://www.coin-or.org/Ipopt/documentation/node10.html (open source)
  * http://www.sbsi-sol-optimize.com/asp/sol_snopt.htm

* In order for ifopt to know for which solvers to build the code, set the environmental variable pointing to the libraries and headers of the solver. If you have IPOPT 3.12.8 installed, add these lines to your ~/.bashrc and re-source. Alternatively you can also supply the location of the shared libraries and 
header files directly in the CMakeLists.txt.

      export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.4
     

## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

Vanilla cmake

    $ git clone https://github.com/ethz-adrl/ifopt.git
    $ cd ifopt
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make 
    
If you are using [catkin], simply clone this repo into your catkin workspace

    $ cd catkin_workspace/src
    $ git clone https://github.com/ethz-adrl/ifopt.git
    $ cd ..
    $ catkin_make
    


## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests

Make sure everything installed correctly by running the unit tests through

    $ make test
     
This should solve the example problem defined in test/ex_problem.h with your installed solver. You can also execute these manually by typing 

    ./test/ex_ipopt 
    ./test/ex_snopt

of if you are using [catkin]

    rosrun ifopt ex_ipopt
    rosrun ifopt ex_snopt
    rosrun ifopt_test 
    

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage

Checkout test/ex_problem.h to see how to specify an optimization problem. 

```c++
#include "ex_problem.h"
#include <ifopt/solvers/ipopt_adapter.h>

using namespace opt;

int main() {
  Problem nlp;

  nlp.AddVariableSet  (std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());

  IpoptAdapter::Solve(nlp); // or SnoptAdapter::Solve(nlp);
  std::cout << nlp.GetOptVariables()->GetValues();
}
```

The actual variables, cost and constraints for this problem are defined as follows.
Add image

```c++
class ExVariables : public Variable {
public:
  ExVariables() : Variable(2, "var_set1")
  {
    // initial values
    x0_ = 0.0;
    x1_ = 0.0;
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
    bounds.at(0) = NoBound;
    bounds.at(1) = Bounds(-1.0, 1.0);
    return bounds;
  }

private:
  double x0_, x1_;
};
```


```c++
class ExConstraint : public Constraint {
public:
  ExConstraint() : Constraint(1, "constraint1"){}

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
    b.at(0) = Bounds(1.0, +inf); // between 1 and inifinity
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


```c++
class ExCost: public Cost {
public:
  ExCost() : Cost("cost_term1") {}

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


##  <img align="center" height="20" src="https://i.imgur.com/H4NwgMg.png"/> Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-adrl/ifopt/issues).

[Eigen]: http://eigen.tuxfamily.org
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://ampl.com/products/solvers/solvers-we-sell/snopt/
[catkin]: http://wiki.ros.org/catkin
[catkin tools]: http://catkin-tools.readthedocs.org/
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Fa2png]: http://fa2png.io/r/font-awesome/link/


