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

#include <gtest/gtest.h>

#include <ifopt/problem.h>
#include <ifopt/test_vars_constr_cost.h>

using namespace ifopt;

TEST(Problem, GetNumberOfOptimizationVariables)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>("var_set0"));
  nlp.AddVariableSet(std::make_shared<ExVariables>("var_set1"));

  EXPECT_EQ(2 + 2, nlp.GetNumberOfOptimizationVariables());
}

TEST(Problem, GetBoundsOnOptimizationVariables)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>("var_set0"));
  nlp.AddVariableSet(std::make_shared<ExVariables>("var_set1"));

  auto bounds = nlp.GetBoundsOnOptimizationVariables();
  EXPECT_EQ(2 + 2, bounds.size());

  // var_set0
  EXPECT_DOUBLE_EQ(-1.0, bounds.at(0).lower_);
  EXPECT_DOUBLE_EQ(+1.0, bounds.at(0).upper_);
  EXPECT_DOUBLE_EQ(-inf, bounds.at(1).lower_);
  EXPECT_DOUBLE_EQ(+inf, bounds.at(1).upper_);

  // var_set1
  EXPECT_DOUBLE_EQ(-1.0, bounds.at(2).lower_);
  EXPECT_DOUBLE_EQ(+1.0, bounds.at(2).upper_);
  EXPECT_DOUBLE_EQ(-inf, bounds.at(3).lower_);
  EXPECT_DOUBLE_EQ(+inf, bounds.at(3).upper_);
}

TEST(Problem, GetVariableValues)
{
  auto var_set0 = std::make_shared<ExVariables>("var_set0");
  var_set0->SetVariables(Eigen::Vector2d(0.1, 0.2));

  auto var_set1 = std::make_shared<ExVariables>("var_set1");
  var_set1->SetVariables(Eigen::Vector2d(0.3, 0.4));

  Problem nlp;
  nlp.AddVariableSet(var_set0);
  nlp.AddVariableSet(var_set1);

  Eigen::VectorXd x = nlp.GetVariableValues();
  EXPECT_EQ(0.1, x(0));
  EXPECT_EQ(0.2, x(1));
  EXPECT_EQ(0.3, x(2));
  EXPECT_EQ(0.4, x(3));
}

TEST(Problem, GetNumberOfConstraints)
{
  Problem nlp;
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint1"));

  // add same constraints again for testing.
  // notice how the Jacobian calculation inside ExConstraint-class remains the
  //same - the full Jacobian is stitched together accordingly.
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint2"));

  EXPECT_EQ(1 + 1, nlp.GetNumberOfConstraints());
}

TEST(Problem, GetBoundsOnConstraints)
{
  Problem nlp;
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint1"));
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint2"));

  auto bounds = nlp.GetBoundsOnConstraints();
  // since it's an equality contraint, upper and lower bound are equal
  EXPECT_DOUBLE_EQ(1.0, bounds.at(0).lower_);
  EXPECT_DOUBLE_EQ(1.0, bounds.at(0).upper_);
  EXPECT_DOUBLE_EQ(1.0, bounds.at(1).lower_);
  EXPECT_DOUBLE_EQ(1.0, bounds.at(1).upper_);
}

TEST(Problem, EvaluateConstraints)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint1"));
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint2"));

  double x[2]       = {2.0, 3.0};
  Eigen::VectorXd g = nlp.EvaluateConstraints(x);
  EXPECT_DOUBLE_EQ(2 * 2.0 + 3.0, g(0));  // constant -1 moved to bounds
  EXPECT_DOUBLE_EQ(2 * 2.0 + 3.0, g(1));  // constant -1 moved to bounds
}

TEST(Problem, GetJacobianOfConstraints)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint1"));
  nlp.AddConstraintSet(std::make_shared<ExConstraint>("constraint2"));

  double x[2] = {2.0, 3.0};
  nlp.SetVariables(x);
  auto jac = nlp.GetJacobianOfConstraints();
  EXPECT_EQ(nlp.GetNumberOfConstraints(), jac.rows());
  EXPECT_EQ(nlp.GetNumberOfOptimizationVariables(), jac.cols());

  EXPECT_DOUBLE_EQ(2 * x[0], jac.coeffRef(0, 0));  // constraint 1 w.r.t x0
  EXPECT_DOUBLE_EQ(1.0, jac.coeffRef(0, 1));       // constraint 1 w.r.t x1
  EXPECT_DOUBLE_EQ(2 * x[0], jac.coeffRef(1, 0));  // constraint 2 w.r.t x0
  EXPECT_DOUBLE_EQ(1.0, jac.coeffRef(1, 1));       // constraint 2 w.r.t x1
}

TEST(Problem, EvaluateCostFunction)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>());
  nlp.AddCostSet(std::make_shared<ExCost>("cost_term1"));
  nlp.AddCostSet(std::make_shared<ExCost>("cost_term2"));

  EXPECT_TRUE(nlp.HasCostTerms());

  double x[2] = {2.0, 3.0};
  EXPECT_DOUBLE_EQ(2 * (-std::pow(x[1] - 2.0, 2)),
                   nlp.EvaluateCostFunction(x));  // constant -1 moved to bounds
}

TEST(Problem, HasCostTerms)
{
  Problem nlp;
  EXPECT_FALSE(nlp.HasCostTerms());

  nlp.AddVariableSet(std::make_shared<ExVariables>());
  EXPECT_FALSE(nlp.HasCostTerms());

  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  EXPECT_FALSE(nlp.HasCostTerms());

  nlp.AddCostSet(std::make_shared<ExCost>());
  EXPECT_TRUE(nlp.HasCostTerms());
}

TEST(Problem, EvaluateCostFunctionGradient)
{
  Problem nlp;
  nlp.AddVariableSet(std::make_shared<ExVariables>());
  nlp.AddCostSet(std::make_shared<ExCost>("cost_term1"));
  nlp.AddCostSet(std::make_shared<ExCost>("cost_term2"));

  double x[2]          = {2.0, 3.0};
  Eigen::VectorXd grad = nlp.EvaluateCostFunctionGradient(x);

  EXPECT_EQ(nlp.GetNumberOfOptimizationVariables(), grad.rows());
  EXPECT_DOUBLE_EQ(0.0, grad(0));                    // cost1+cost2 w.r.t x0
  EXPECT_DOUBLE_EQ(2 * (-2 * (x[1] - 2)), grad(1));  // cost1+cost2 w.r.t x1
}
