/******************************************************************************
Copyright (c) 2017, Alexander W Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
 *  @file test_vars_constr_cost.h
 *
 *  @brief Example to generate a solver-independent formulation for the problem, taken
 *  from the IPOPT cpp_example.
 *
 *  The example problem to be solved is given as:
 *
 *      min_x f(x) = -(x1-2)^2
 *      s.t.
 *           0 = x0^2 + x1 - 1
 *           -1 <= x0 <= 1
 *
 * In this simple example we only use one set of variables, constraints and
 * cost. However, most real world problems have multiple different constraints
 * and also different variable sets representing different quantities. This
 * framework allows to define each set of variables or constraints absolutely
 * independently from another and correctly stitches them together to form the
 * final optimization problem.
 *
 * For a helpful graphical overview, see:
 * http://docs.ros.org/api/ifopt/html/group__ProblemFormulation.html
 */

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>

namespace ifopt {
using Eigen::Vector2d;

class ExVariables : public VariableSet {
 public:
  // Every variable set has a name, here "var_set1". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  ExVariables() : ExVariables("var_set1"){};
  ExVariables(const std::string& name) : VariableSet(2, name)
  {
    // the initial values where the NLP starts iterating from
    x0_ = 3.5;
    x1_ = 1.5;
  }

  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    x0_ = x(0);
    x1_ = x(1);
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override { return Vector2d(x0_, x1_); };

  // Each variable has an upper and lower bound set here
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

class ExConstraint : public ConstraintSet {
 public:
  ExConstraint() : ExConstraint("constraint1") {}

  // This constraint set just contains 1 constraint, however generally
  // each set can contain multiple related constraints.
  ExConstraint(const std::string& name) : ConstraintSet(1, name) {}

  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    g(0)       = std::pow(x(0), 2) + x(1);
    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = Bounds(1.0, 1.0);
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  // Attention: see the parent class function for important information on sparsity pattern.
  void FillJacobianBlock(std::string var_set,
                         Jacobian& jac_block) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "var_set1". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac_block.coeffRef(0, 0) =
          2.0 * x(0);  // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) =
          1.0;  // derivative of first constraint w.r.t x1
    }
  }
};

class ExCost : public CostTerm {
 public:
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {}

  double GetCost() const override
  {
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    return -std::pow(x(1) - 2, 2);
  };

  void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 0.0;                  // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0 * (x(1) - 2.0);  // derivative of cost w.r.t x1
    }
  }
};

}  // namespace ifopt
