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
#include <ifopt/ipopt_solver.h>

namespace ifopt {

IpoptSolver::IpoptSolver(bool rethrow_non_ipopt_exceptions)
{
  ipopt_app_ = std::make_shared<Ipopt::IpoptApplication>();
  status_    = Ipopt::Solve_Succeeded;

  /* Which linear solver to use. Mumps is default because it comes with the
   * precompiled ubuntu binaries. However, the coin-hsl solvers can be
   * significantly faster and are free for academic purposes. They can be
   * downloaded here: http://www.hsl.rl.ac.uk/ipopt/ and must be compiled
   * into your IPOPT libraries. Then you can use the additional strings:
   * "ma27, ma57, ma77, ma86, ma97" here.
   */
  SetOption("linear_solver", "mumps");

  /* whether to use the analytical derivatives "exact" coded in ifopt, or let
   * IPOPT approximate these through "finite difference-values". This is usually
   * significantly slower.
   */
  SetOption("jacobian_approximation", "exact");
  SetOption("hessian_approximation", "limited-memory");
  SetOption("max_cpu_time", 40.0);
  SetOption("tol", 0.001);
  SetOption("print_timing_statistics", "no");
  SetOption("print_user_options", "no");
  SetOption("print_level", 4);

  // SetOption("max_iter", 1);
  // SetOption("derivative_test", "first-order");
  // SetOption("derivative_test_tol", 1e-3);

  // Enable or Disable throwing original exceptions for catching errors
  ipopt_app_->RethrowNonIpoptException(rethrow_non_ipopt_exceptions);
}

void IpoptSolver::Solve(Problem& nlp)
{
  using namespace Ipopt;

  status_ = ipopt_app_->Initialize();
  if (status_ != Solve_Succeeded) {
    std::cout << std::endl
              << std::endl
              << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  // check the jacobian_approximation method
  std::string jac_type = "";
  ipopt_app_->Options()->GetStringValue("jacobian_approximation", jac_type, "");
  bool finite_diff = jac_type == "finite-difference-values";

  // convert the NLP problem to Ipopt
  SmartPtr<TNLP> nlp_ptr = new IpoptAdapter(nlp, finite_diff);
  status_                = ipopt_app_->OptimizeTNLP(nlp_ptr);

  if (status_ != Solve_Succeeded) {
    std::string msg = "ERROR: Ipopt failed to find a solution. Return Code: " +
                      std::to_string(status_) + "\n";
    std::cerr << msg;
  }
}

void IpoptSolver::SetOption(const std::string& name, const std::string& value)
{
  ipopt_app_->Options()->SetStringValue(name, value);
}

void IpoptSolver::SetOption(const std::string& name, int value)
{
  ipopt_app_->Options()->SetIntegerValue(name, value);
}

void IpoptSolver::SetOption(const std::string& name, double value)
{
  ipopt_app_->Options()->SetNumericValue(name, value);
}

double IpoptSolver::GetTotalWallclockTime()
{
  return ipopt_app_->Statistics()->TotalWallclockTime();
}

} /* namespace ifopt */
