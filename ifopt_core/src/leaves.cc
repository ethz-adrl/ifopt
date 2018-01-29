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

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>


namespace ifopt {

VariableSet::VariableSet(int n_var, const std::string& name)
    : Component(n_var, name)
{
}

ConstraintSet::ConstraintSet (int row_count, const std::string& name)
    : Component(row_count, name)
{
}

ConstraintSet::Jacobian
ConstraintSet::GetJacobian () const
{
  Jacobian jacobian(GetRows(), variables_->GetRows());

  int col = 0;
  for (const auto& vars : variables_->GetComponents()) {

    int n = vars->GetRows();
    Jacobian jac = Jacobian(GetRows(), n);

    FillJacobianBlock(vars->GetName(), jac);

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<jac.outerSize(); ++k)
      for (Jacobian::InnerIterator it(jac,k); it; ++it)
        jacobian.coeffRef(it.row(), col+it.col()) = it.value();

    col += n;
  }

  return jacobian;
}

void
ConstraintSet::LinkWithVariables(const VariablesPtr& x)
{
  variables_ = x;
  InitVariableDependedQuantities(x);
}

CostTerm::CostTerm (const std::string& name) :ConstraintSet(1, name)
{
}

CostTerm::VectorXd
CostTerm::GetValues() const
{
  VectorXd cost(1);
  cost(0) = GetCost();
  return cost;
}

CostTerm::VecBound
CostTerm::GetBounds() const
{
  return VecBound(GetRows(), NoBound);
}


} /* namespace opt */
