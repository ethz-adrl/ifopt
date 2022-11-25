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

namespace Ipopt {
class IpoptApplication;
}

namespace ifopt {

/**
 * @brief An interface to IPOPT, fully hiding its implementation.
 *
 * To set specific options, see:
 * https://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * @ingroup Solvers
 */
class IpoptSolver : public Solver {
 public:
  using Ptr = std::shared_ptr<IpoptSolver>;

  IpoptSolver(bool rethrow_non_ipopt_exceptions = false);
  virtual ~IpoptSolver() = default;

  /** @brief  Creates an IpoptAdapter and solves the NLP.
    * @param [in/out]  nlp  The specific problem.
    */
  void Solve(Problem& nlp) override;

  /** Set options for the IPOPT solver. A complete list can be found here:
    * https://www.coin-or.org/Ipopt/documentation/node40.html
    */
  void SetOption(const std::string& name, const std::string& value);
  void SetOption(const std::string& name, int value);
  void SetOption(const std::string& name, double value);

  /** @brief  Get the total wall clock time for the optimization, including function evaluations.
    */
  double GetTotalWallclockTime();

 private:
  std::shared_ptr<Ipopt::IpoptApplication> ipopt_app_;
};

} /* namespace ifopt */

#endif /* IFOPT_SRC_IFOPT_IPOPT_INCLUDE_IFOPT_IPOPT_H_ */
