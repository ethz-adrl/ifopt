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

#ifndef IFOPT_INCLUDE_OPT_PROBLEM_H_
#define IFOPT_INCLUDE_OPT_PROBLEM_H_

#include "variable_set.h"
#include "constraint_set.h"
#include "cost_term.h"

namespace ifopt {

/**
 * @brief A generic optimization problem with variables, costs and constraints.
 *
 * This class is responsible for holding all the information of an optimization
 * problem, which includes the optimization variables, their variable bounds,
 * the cost function, the constraints and their bounds and derivatives of
 * all. With this information the problem can be solved by any specific solver.
 *
 *  find x1, x2                            (Variable sets 1 & 2)
 *  s.t
 *       x1_lower  <= x1 <= x1_upper       (bounds on variable set x1 \in R^m1)
 *
 *       g1_lower < g1(x1,x2) < g1_upper   (Constraint set 1 \in R^n1)
 *       g2_lower < g2(x1,x2) < g2_upper   (Constraint set 2 \in R^n2)
 *
 *       x1,x2 = arg min c1(x1,x2)         (Cost terms 1)
 *
 *
 * See @ref solvers for currently implemented solvers.
 *
 * Notice all the quantities (variables, cost, constraint) are represented
 * by the same generic Component class.
 */
class Problem {
public:
  using VecBound = Component::VecBound;
  using Jacobian = Component::Jacobian;
  using VectorXd = Component::VectorXd;

  /**
   * @brief  Creates a optimization problem with no variables, costs or constraints.
   */
  Problem ();
  virtual ~Problem () = default;

  /**
   * @brief Add one individual set of variables to the optimization problem.
   * @param variable_set  The selection of optimization variables.
   *
   * This function can be called multiple times, with multiple sets, e.g.
   * one set that parameterizes a body trajectory, the other that resembles
   * the optimal timing values. This function correctly appends the
   * individual variables sets and ensures correct order of Jacobian columns.
   */
  void AddVariableSet(VariableSet::Ptr variable_set);

  /**
   * @brief Add a set of multiple constraints to the optimization problem.
   * @param constraint_set  This can be 1 to infinity number of constraints.
   *
   * This function can be called multiple times for different sets of
   * constraints. It makes sure the overall constraint and Jacobian correctly
   * considers all individual constraint sets.
   */
  void AddConstraintSet(ConstraintSet::Ptr constraint_set);

  /**
   * @brief Add a cost term to the optimization problem.
   * @param cost_set  The calculation of the cost from the variables.
   *
   * This function can be called multiple times if the cost function is
   * composed of different cost terms. It makes sure the overall value and
   * gradient is considering each individual cost.
   */
  void AddCostSet(CostTerm::Ptr cost_set);

  /**
   * @brief  Updates the variables with the values of the raw pointer @c x.
   */
  void SetVariables(const double* x);

  /**
   * @brief The number of optimization variables.
   */
  int GetNumberOfOptimizationVariables() const;

  /**
   * @brief True if the optimization problem includes a cost, false if
   * merely a feasibility problem is defined.
   */
  bool HasCostTerms() const;

  /**
   * @brief The maximum and minimum value each optimization variable
   * is allowed to have.
   */
  VecBound GetBoundsOnOptimizationVariables() const;

  /**
   * @brief The current value of the optimization variables.
   */
  VectorXd GetVariableValues() const;

  /**
   * @brief The scalar cost for current optimization variables @c x.
   */
  double EvaluateCostFunction(const double* x);

  /**
   * @brief The column-vector of derivatives of the cost w.r.t. each variable.
   */
  VectorXd EvaluateCostFunctionGradient(const double* x);

  /**
   * @brief The number of individual constraints.
   */
  int GetNumberOfConstraints() const;

  /**
   * @brief The upper and lower bound of each individual constraint.
   */
  VecBound GetBoundsOnConstraints() const;

  /**
   * @brief Each constraint value g(x) for current optimization variables @c x.
   */
  VectorXd EvaluateConstraints(const double* x);

  /**
   * @brief Extracts those entries from constraint Jacobian that are not zero.
   * @param [in]  x  The current values of the optimization variables.
   * @param [out] values  The nonzero derivatives ordered by Eigen::RowMajor.
   */
  void EvalNonzerosOfJacobian(const double* x, double* values);

  /**
   * @brief The sparse-matrix representation of Jacobian of the constraints.
   *
   * Each row corresponds to a constraint and each column to an optimizaton
   * variable.
   */
  Jacobian GetJacobianOfConstraints() const;

  /**
   * @brief Saves the current values of the optimization variables in x_prev.
   *
   * This is used to keep a history of the values for each NLP iterations.
   */
  void SaveCurrent();

  /**
   * @brief Read/write access to the current optimization variables.
   */
  Composite::Ptr GetOptVariables() const;

  /**
   * @brief Sets the optimization variables to those at iteration iter.
   */
  void SetOptVariables(int iter);

  /**
   * @brief Sets the optimization variables to those of the final iteration.
   */
  void SetOptVariablesFinal();

  /**
   * @brief The number of iterations it took to solve the problem.
   */
  int GetIterationCount() const { return x_prev.size(); };

  /**
   * @brief Prints the variables, costs and constraints.
   */
  void PrintCurrent() const;

private:
  Composite::Ptr variables_;
  Composite constraints_;
  Composite costs_;

  std::vector<VectorXd> x_prev; ///< the pure variables for every iteration.

  VectorXd ConvertToEigen(const double* x) const;
};

} /* namespace opt */

#endif /* IFOPT_INCLUDE_OPT_PROBLEM_H_ */
