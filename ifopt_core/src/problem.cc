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

#include <ifopt/problem.h>
#include <iomanip>
#include <iostream>

namespace ifopt {

Problem::Problem()
    : constraints_("constraint-sets", false), costs_("cost-terms", true)
{
  variables_ = std::make_shared<Composite>("variable-sets", false);
}

void Problem::AddVariableSet(VariableSet::Ptr variable_set)
{
  variables_->AddComponent(variable_set);
}

void Problem::AddConstraintSet(ConstraintSet::Ptr constraint_set)
{
  constraint_set->LinkWithVariables(variables_);
  constraints_.AddComponent(constraint_set);
}

void Problem::AddCostSet(CostTerm::Ptr cost_set)
{
  cost_set->LinkWithVariables(variables_);
  costs_.AddComponent(cost_set);
}

int Problem::GetNumberOfOptimizationVariables() const
{
  return variables_->GetRows();
}

Problem::VecBound Problem::GetBoundsOnOptimizationVariables() const
{
  return variables_->GetBounds();
}

Problem::VectorXd Problem::GetVariableValues() const
{
  return variables_->GetValues();
}

void Problem::SetVariables(const double* x)
{
  variables_->SetVariables(ConvertToEigen(x));
}

double Problem::EvaluateCostFunction(const double* x)
{
  VectorXd g = VectorXd::Zero(1);
  if (HasCostTerms()) {
    SetVariables(x);
    g = costs_.GetValues();
  }
  return g(0);
}

Problem::VectorXd Problem::EvaluateCostFunctionGradient(
    const double* x, bool use_finite_difference_approximation, double epsilon)
{
  int n        = GetNumberOfOptimizationVariables();
  Jacobian jac = Jacobian(1, n);
  if (HasCostTerms()) {
    if (use_finite_difference_approximation) {
      double step_size = epsilon;

      // calculate forward difference by disturbing each optimization variable
      double g = EvaluateCostFunction(x);
      std::vector<double> x_new(x, x + n);
      for (int i = 0; i < n; ++i) {
        x_new[i] += step_size;  // disturb
        double g_new       = EvaluateCostFunction(x_new.data());
        jac.coeffRef(0, i) = (g_new - g) / step_size;
        x_new[i]           = x[i];  // reset for next iteration
      }
    } else {
      SetVariables(x);
      jac = costs_.GetJacobian();
    }
  }

  return jac.row(0).transpose();
}

Problem::VecBound Problem::GetBoundsOnConstraints() const
{
  return constraints_.GetBounds();
}

int Problem::GetNumberOfConstraints() const
{
  return GetBoundsOnConstraints().size();
}

Problem::VectorXd Problem::EvaluateConstraints(const double* x)
{
  SetVariables(x);
  return constraints_.GetValues();
}

bool Problem::HasCostTerms() const
{
  return costs_.GetRows() > 0;
}

void Problem::EvalNonzerosOfJacobian(const double* x, double* values)
{
  SetVariables(x);
  Jacobian jac = GetJacobianOfConstraints();

  jac.makeCompressed();  // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

Problem::Jacobian Problem::GetJacobianOfConstraints() const
{
  return constraints_.GetJacobian();
}

Problem::Jacobian Problem::GetJacobianOfCosts() const
{
  return costs_.GetJacobian();
}

void Problem::SaveCurrent()
{
  x_prev.push_back(variables_->GetValues());
}

Composite::Ptr Problem::GetOptVariables() const
{
  return variables_;
}

void Problem::SetOptVariables(int iter)
{
  variables_->SetVariables(x_prev.at(iter));
}

void Problem::SetOptVariablesFinal()
{
  variables_->SetVariables(x_prev.at(GetIterationCount() - 1));
}

void Problem::PrintCurrent() const
{
  using namespace std;
  cout << "\n"
       << "************************************************************\n"
       << "    IFOPT - Interface to Nonlinear Optimizers (v2.0)\n"
       << "                \u00a9 Alexander W. Winkler\n"
       << "           https://github.com/ethz-adrl/ifopt\n"
       << "************************************************************"
       << "\n"
       << "Legend:\n"
       << "c - number of variables, constraints or cost terms" << std::endl
       << "i - indices of this set in overall problem" << std::endl
       << "v - number of [violated variable- or constraint-bounds] or [cost "
          "term value]"
       << "\n\n"
       << std::right << std::setw(33) << "" << std::setw(5) << "c  "
       << std::setw(16) << "i    " << std::setw(11) << "v " << std::left
       << "\n";

  variables_->PrintAll();
  constraints_.PrintAll();
  costs_.PrintAll();
};

Problem::VectorXd Problem::ConvertToEigen(const double* x) const
{
  return Eigen::Map<const VectorXd>(x, GetNumberOfOptimizationVariables());
}

}  // namespace ifopt
