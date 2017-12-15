### <img src="https://i.imgur.com/Y4MsIIm.png" height="80" />

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-adrl/ifopt/master)](https://ci.leggedrobotics.com/job/github_ethz-adrl/job/ifopt/job/master/)

Ifopt is a unified [Eigen]-based interface to use Nonlinear Programming solvers, such as [Ipopt] and [Snopt]. The user defines the solver independent optimization problem by set of C++ classes resembling variables, cost and constraints. Subsequently, the problem can then be solved with either solver. This package can also be dropped in your [catkin] workspace.

**Author/Maintainer: [Alexander W. Winkler](https://awinkler.github.io/)** 

[<img src="https://i.imgur.com/uCvLs2j.png" height="60" />](http://www.adrl.ethz.ch/doku.php)  &nbsp; &nbsp; &nbsp; &nbsp;    [<img src="https://i.imgur.com/aGOnNTZ.png" height="60" />](https://www.ethz.ch/en.html)

The code is currently maintained at the [Robotic Systems Lab](http://www.rsl.ethz.ch/).

-------
... also we only need __928 lines of code__ to allow the generation of (1) solver indenpendent problem formulations, (2) automatic ordering of independent variable and constraint sets in the overall problem, (3) [Eigen] sparse-matrix exploitation for fast performance, (4) constraint-jacobian and cost-gradient ordering and (5) implementation of interfaces to [Ipopt] and [Snopt]. 

<img align="center" height="150" src="https://i.imgur.com/gzLoSVU.png"/>

[Why this matters](https://work.qz.com/1154701/a-short-equation-explains-why-simplicity-is-the-best-policy/)



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Installation

* Install [Eigen]

      $ sudo apt-get install libeigen3-dev
    
* Depending on which solver you want to use, install either [Ipopt] or [Snopt]. Follow the instructions provided here:

     * https://www.coin-or.org/Ipopt/documentation/node10.html (open source)
     * http://www.sbsi-sol-optimize.com/asp/sol_snopt.htm

* In order for ifopt to know for which solvers to build the code, set the environmental variable pointing to the libraries and headers of the solver. If you have IPOPT 3.12.8 installed, add `export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.8` to your ~/.bashrc and re-source. Alternatively you can also supply the location of the shared libraries and 
header files directly in the [CMakeLists.txt](CMakeLists.txt).
     

## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building
Vanilla cmake as

    $ git clone https://github.com/ethz-adrl/ifopt.git
    $ cd ifopt
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make 
    
or if you are using [catkin], simply clone this repo into your catkin workspace

    $ cd catkin_workspace/src
    $ git clone https://github.com/ethz-adrl/ifopt.git
    $ cd ..
    $ catkin_make
    


## <img align="center" height="20" src="https://i.imgur.com/026nVBV.png"/> Unit Tests

Make sure everything installed correctly by running the unit tests through

    $ make test
     
This should solve the [example problem](test/ex_problem.h) with your installed solver. You can also execute these manually by typing 

    ./test/ex_ipopt 
    ./test/ex_snopt

of if you are using [catkin]

    rosrun ifopt ex_ipopt
    rosrun ifopt ex_snopt
    rosrun ifopt_test 
    

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Usage

Example from [test/ex_problem.h](test/ex_problem.h).
The optimization problem to solve is defined as:

<img align="center" height="100" src="https://i.imgur.com/naxXK2i.png"/>

```c++
#include <ifopt/test/ex_problem.h>
#include <ifopt/solvers/ipopt_adapter.h>

int main() {
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
0 1
```

In this simple example we only have one set of variables, constraints and cost. However, most real world problems have multiple different constraints and also different variable sets representing different quantities. This framework allows to define each set of variables or constraints absolutly independently from another and correctly stitches them together to form the final optimization problem.

The following shows how we transform this into a solver independent optimization problem.
```c++
class ExVariables : public VariableSet {
public:
  ExVariables() : VariableSet(2, "var_set1")
  { // initial values
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
    b.at(0) = Bounds(1.0, +inf); // between 1 and inifinity
    return b;
  }

  // (can also be ommited and the NLP calculates through numeric differences)
  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "var_set1". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
      
      jac.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
    }
  }
};
```



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


