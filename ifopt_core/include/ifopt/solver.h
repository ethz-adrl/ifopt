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

#ifndef IFOPT_SRC_IFOPT_CORE_INCLUDE_IFOPT_SOLVER_H_
#define IFOPT_SRC_IFOPT_CORE_INCLUDE_IFOPT_SOLVER_H_

#include <memory>

#include <ifopt/problem.h>

namespace ifopt {

/**
 * @defgroup Solvers
 * @brief Interfaces to IPOPT and SNOPT to solve the optimization problem.
 *
 * These are included in the folders: @ref ifopt_ipopt/ and @ref ifopt_snopt/.
 */

/**
 * @brief Solver interface implemented by IPOPT and SNOPT.
 *
 * @ingroup Solvers
 */
class Solver {
 public:
  using Ptr = std::shared_ptr<Solver>;

  virtual ~Solver() = default;

  /** @brief  Uses a specific solver (IPOPT, SNOPT) to solve the NLP.
    * @param [in/out]  nlp  The nonlinear programming problem.
    */
  virtual void Solve(Problem& nlp) = 0;

  /** @brief  Get the return status for the optimization. 
    * 
    * e.g. https://coin-or.github.io/Ipopt/OUTPUT.html
    */
  int GetReturnStatus() const { return status_; };

 protected:
  int status_;
};

} /* namespace ifopt */

#endif /* IFOPT_SRC_IFOPT_CORE_INCLUDE_IFOPT_SOLVER_H_ */
