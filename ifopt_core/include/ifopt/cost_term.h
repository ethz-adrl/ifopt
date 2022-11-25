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

#ifndef IFOPT_INCLUDE_IFOPT_COST_TERM_H_
#define IFOPT_INCLUDE_IFOPT_COST_TERM_H_

#include "constraint_set.h"

namespace ifopt {

/**
 * @brief A container holding a single cost term.
 *
 * This container builds a scalar cost term from the values of the variables.
 * This can be seen as a constraint with only one row and no bounds.
 *
 * @ingroup ProblemFormulation
 * @sa Component
 */
class CostTerm : public ConstraintSet {
 public:
  CostTerm(const std::string& name);
  virtual ~CostTerm() = default;

 private:
  /**
   * @brief  Returns the scalar cost term calculated from the @c variables.
   */
  virtual double GetCost() const = 0;

 public:
  /**
   * @brief  Wrapper function that converts double to Eigen::VectorXd.
   */
  VectorXd GetValues() const final;

  /**
   * @brief  Returns infinite bounds (e.g. no bounds).
   */
  VecBound GetBounds() const final;

  /**
   * Cost term printout slightly different from variables/constraints.
   */
  void Print(double tol, int& index) const final;
};

}  // namespace ifopt

#endif /* IFOPT_INCLUDE_IFOPT_COST_TERM_H_ */
