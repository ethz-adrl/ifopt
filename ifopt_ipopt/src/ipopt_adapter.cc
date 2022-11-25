/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ifopt/ipopt_adapter.h>

namespace Ipopt {

IpoptAdapter::IpoptAdapter(Problem& nlp, bool finite_diff)
{
  nlp_         = &nlp;
  finite_diff_ = finite_diff;
}

bool IpoptAdapter::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  n = nlp_->GetNumberOfOptimizationVariables();
  m = nlp_->GetNumberOfConstraints();

  if (finite_diff_)
    nnz_jac_g = m * n;
  else
    nnz_jac_g = nlp_->GetJacobianOfConstraints().nonZeros();

  nnz_h_lag = n * n;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

bool IpoptAdapter::get_bounds_info(Index n, double* x_lower, double* x_upper,
                                   Index m, double* g_l, double* g_u)
{
  auto bounds_x = nlp_->GetBoundsOnOptimizationVariables();
  for (std::size_t c = 0; c < bounds_x.size(); ++c) {
    x_lower[c] = bounds_x.at(c).lower_;
    x_upper[c] = bounds_x.at(c).upper_;
  }

  // specific bounds depending on equality and inequality constraints
  auto bounds_g = nlp_->GetBoundsOnConstraints();
  for (std::size_t c = 0; c < bounds_g.size(); ++c) {
    g_l[c] = bounds_g.at(c).lower_;
    g_u[c] = bounds_g.at(c).upper_;
  }

  return true;
}

bool IpoptAdapter::get_starting_point(Index n, bool init_x, double* x,
                                      bool init_z, double* z_L, double* z_U,
                                      Index m, bool init_lambda, double* lambda)
{
  // Here, we assume we only have starting values for x
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  VectorXd x_all                            = nlp_->GetVariableValues();
  Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;

  return true;
}

bool IpoptAdapter::eval_f(Index n, const double* x, bool new_x,
                          double& obj_value)
{
  obj_value = nlp_->EvaluateCostFunction(x);
  return true;
}

bool IpoptAdapter::eval_grad_f(Index n, const double* x, bool new_x,
                               double* grad_f)
{
  Eigen::VectorXd grad = nlp_->EvaluateCostFunctionGradient(x, finite_diff_);
  Eigen::Map<Eigen::MatrixXd>(grad_f, n, 1) = grad;
  return true;
}

bool IpoptAdapter::eval_g(Index n, const double* x, bool new_x, Index m,
                          double* g)
{
  VectorXd g_eig             = nlp_->EvaluateConstraints(x);
  Eigen::Map<VectorXd>(g, m) = g_eig;
  return true;
}

bool IpoptAdapter::eval_jac_g(Index n, const double* x, bool new_x, Index m,
                              Index nele_jac, Index* iRow, Index* jCol,
                              double* values)
{
  // defines the positions of the nonzero elements of the jacobian
  if (values == NULL) {
    // If "jacobian_approximation" option is set as "finite-difference-values", the Jacobian is dense!
    Index nele = 0;
    if (finite_diff_) {  // dense jacobian
      for (Index row = 0; row < m; row++) {
        for (Index col = 0; col < n; col++) {
          iRow[nele] = row;
          jCol[nele] = col;
          nele++;
        }
      }
    } else {  // sparse jacobian
      auto jac = nlp_->GetJacobianOfConstraints();
      for (int k = 0; k < jac.outerSize(); ++k) {
        for (Jacobian::InnerIterator it(jac, k); it; ++it) {
          iRow[nele] = it.row();
          jCol[nele] = it.col();
          nele++;
        }
      }
    }

    assert(nele ==
           nele_jac);  // initial sparsity structure is never allowed to change
  } else {
    // only gets used if "jacobian_approximation finite-difference-values" is not set
    nlp_->EvalNonzerosOfJacobian(x, values);
  }

  return true;
}

bool IpoptAdapter::intermediate_callback(
    AlgorithmMode mode, Index iter, double obj_value, double inf_pr,
    double inf_du, double mu, double d_norm, double regularization_size,
    double alpha_du, double alpha_pr, Index ls_trials, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq)
{
  nlp_->SaveCurrent();
  return true;
}

void IpoptAdapter::finalize_solution(SolverReturn status, Index n,
                                     const double* x, const double* z_L,
                                     const double* z_U, Index m,
                                     const double* g, const double* lambda,
                                     double obj_value, const IpoptData* ip_data,
                                     IpoptCalculatedQuantities* ip_cq)
{
  nlp_->SetVariables(x);
  nlp_->SaveCurrent();
}

}  // namespace Ipopt
