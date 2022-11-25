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

#ifndef IFOPT_INCLUDE_IFOPT_VARIABLE_SET_H_
#define IFOPT_INCLUDE_IFOPT_VARIABLE_SET_H_

#include "composite.h"

namespace ifopt {

/**
 * @brief  A container holding a set of related optimization variables.
 *
 * This is a single set of variables representing a single concept, e.g
 * "spline coefficients" or "step durations".
 *
 * @ingroup ProblemFormulation
 * @sa Component
 */
class VariableSet : public Component {
 public:
  /**
   * @brief Creates a set of variables representing a single concept.
   * @param n_var  Number of variables.
   * @param name   What the variables represent to (e.g. "spline coefficients").
   */
  VariableSet(int n_var, const std::string& name);
  virtual ~VariableSet() = default;

  // doesn't exist for variables, generated run-time error when used.
  Jacobian GetJacobian() const final
  {
    throw std::runtime_error("not implemented for variables");
  };
};

}  // namespace ifopt

#endif /* IFOPT_INCLUDE_IFOPT_VARIABLE_SET_H_ */
