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

#include <ifopt/ipopt.h>
#include <ifopt/ipopt_adapter.h>

namespace ifopt {


void
Ipopt::Solve (Problem& nlp)
{
  using namespace Ipopt;
  using IpoptPtr            = SmartPtr<TNLP>;
  using IpoptApplicationPtr = SmartPtr<IpoptApplication>;

  IpoptApplicationPtr ipopt_app_ = new IpoptApplication();

  ipopt_app_->Options()->SetStringValue("linear_solver", linear_solver_);
  ipopt_app_->Options()->SetStringValue("hessian_approximation", hessian_approximation_);
  ipopt_app_->Options()->SetNumericValue("max_cpu_time", max_cpu_time_);
  ipopt_app_->Options()->SetNumericValue("tol", tol_);
  if (use_jacobian_approximation_)
    ipopt_app_->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");

  ipopt_app_->Options()->SetStringValue("print_timing_statistics", print_timing_statistics_);
  ipopt_app_->Options()->SetStringValue("print_user_options", print_user_options_);
  ipopt_app_->Options()->SetIntegerValue("print_level", print_level_);

  //  ipopt_app_->Options()->SetIntegerValue("max_iter", 1);
  //  ipopt_app_->Options()->SetNumericValue("derivative_test_tol", 1e-3);
  //  ipopt_app_->Options()->SetStringValue("derivative_test", "first-order"); // "second-order"

  ApplicationReturnStatus status_ = ipopt_app_->Initialize();
  if (status_ != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    throw std::length_error("Ipopt could not initialize correctly");
  }

  // convert the NLP problem to Ipopt
  IpoptPtr nlp_ptr = new IpoptAdapter(nlp);
  status_ = ipopt_app_->OptimizeTNLP(nlp_ptr);

  if (status_ != Solve_Succeeded) {
    std::string msg = "ERROR: Ipopt failed to find a solution. ReturnCode: " + std::to_string(status_) + "\n";
    std::cerr << msg;
  }
}



} /* namespace ifopt */
