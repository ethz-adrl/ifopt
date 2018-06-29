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

#ifndef IFOPT_SRC_IFOPT_IPOPT_INCLUDE_IFOPT_IPOPT_H_
#define IFOPT_SRC_IFOPT_IPOPT_INCLUDE_IFOPT_IPOPT_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>

namespace ifopt {

class Ipopt : public Solver {
public:
  using Ptr = std::shared_ptr<Ipopt>;

  /** @brief  Creates an IpoptAdapter and solves the NLP.
    * @param [in/out]  nlp  The specific problem.
    *
    * This function creates the actual solver, sets the solver specific
    * options (see SetOptions()) and passes the IpoptAdapter problem to it
    * to be modified.
    */
  virtual void Solve(Problem& nlp) override;

  /** Options for the IPOPT solver. A complete list can be found here:
    * https://www.coin-or.org/Ipopt/documentation/node40.html
    */

  /** Which linear solver to use. Mumps is default because it comes with the
   *  precompiled ubuntu binaries. However, the coin-hsl solvers can be
   *  significantly faster and are free for academic purposes. They can be
   *  downloaded here: http://www.hsl.rl.ac.uk/ipopt/ and must be compiled
   *  into your IPOPT libraries. Then you can use the additional strings:
   *  "ma27, ma57, ma77, ma86, ma97" here.
   */
  std::string linear_solver_ = "mumps";

  /** whether to use the analytical derivatives coded in ifopt, or let
   *  IPOPT approximate these through finite differences. This is usually
   *  significantly slower.
   */
  bool use_jacobian_approximation_ = false;


  std::string hessian_approximation_ = "limited-memory";
  double tol_ = 0.001;
  double max_cpu_time_ = 40.0;
  int print_level_ = 3;
  std::string print_user_options_ = "no";
  std::string print_timing_statistics_ = "no";




};

} /* namespace ifopt */

#endif /* IFOPT_SRC_IFOPT_IPOPT_INCLUDE_IFOPT_IPOPT_H_ */
