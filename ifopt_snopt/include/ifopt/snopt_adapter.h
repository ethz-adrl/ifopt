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

#ifndef IFOPT_INCLUDE_OPT_SNOPT_ADAPTER_H_
#define IFOPT_INCLUDE_OPT_SNOPT_ADAPTER_H_

#include <snoptProblem.hpp>

#include <ifopt/problem.h>

namespace ifopt {

/**
 * @brief Solves the optimization problem with SNOPT version 7.5 and below.
 *
 * If you have a newer version of SNOPT installed, use snopt_adapter_76.h.
 *
 * Given an optimization Problem with variables, costs and constraints, this
 * class wraps it and makes it conform with the interface defined by SNOPT
 * http://web.stanford.edu/group/SOL/guides/sndoc7.pdf
 *
 * This implements the Adapter pattern. This class should not add any
 * functionality, but merely delegate it to the Adaptee (the Problem object).
 */
class SnoptAdapter : public snoptProblemA {
 public:
  using NLPPtr   = Problem*;
  using VectorXd = Problem::VectorXd;
  using Jacobian = Problem::Jacobian;

  /**
   * @brief  Creates an Adapter Object around the problem to conform to the
   * Snopt interface.
   */
  SnoptAdapter(Problem& nlp);
  virtual ~SnoptAdapter() = default;

  void Init();
  void SetVariables();

  static void ObjectiveAndConstraintFct(int* Status, int* n, double x[],
                                        int* needF, int* neF, double F[],
                                        int* needG, int* neG, double G[],
                                        char* cu, int* lencu, int iu[],
                                        int* leniu, double ru[], int* lenru);

 private:
  static NLPPtr nlp_;  // use raw pointer as SnoptAdapter doesn't own the nlp.

// additional variables as Snopt76 base class doesn't have them, not really
// necessary but to keep the same structure of the original SnoptAdapter
#ifdef SNOPT76

 public:
  int jacComputed = 0;
  int n           = 0;
  int neF         = 0;
  int ObjRow;
  double ObjAdd;

  double *x, *xlow, *xupp, *xmul;
  double *F, *Flow, *Fupp, *Fmul;

  int *xstate, *Fstate;

  int lenA, lenG, neA, neG;
  int *iAfun, *jAvar, *iGfun, *jGvar;
  double* A;
#endif
};

}  // namespace ifopt

#endif /* IFOPT_INCLUDE_OPT_SNOPT_ADAPTER_H_ */
