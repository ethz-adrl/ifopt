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

#include <ifopt/snopt_adapter.h>
#include <ifopt/snopt_solver.h>

namespace ifopt {

void SnoptSolver::Solve(Problem& ref)
{
  SnoptAdapter snopt(ref);
  snopt.Init();

  // A complete list of options can be found in the snopt user guide:
  // https://web.stanford.edu/group/SOL/guides/sndoc7.pdf
  snopt.setProbName("snopt");
  snopt.setIntParameter("Major Print level", 1);
  snopt.setIntParameter("Minor Print level", 1);
  snopt.setIntParameter("Derivative option",
                        1);  // 1 = snopt will not calculate missing derivatives
  snopt.setIntParameter("Verify level ",
                        3);  // full check on gradients, will throw error
  snopt.setIntParameter("Iterations limit", 200000);
  snopt.setRealParameter("Major feasibility tolerance",
                         1.0e-4);  // target nonlinear constraint violation
  snopt.setRealParameter("Minor feasibility tolerance",
                         1.0e-4);  // for satisfying the QP bounds
  snopt.setRealParameter("Major optimality tolerance",
                         1.0e-2);  // target complementarity gap

  // error codes as given in the manual.
  int Cold = 0;  // Basis = 1, Warm = 2;

  // interface changed with snopt version 7.6
#ifdef SNOPT76
  int nS = 0;   // number of super-basic variables (not relevant for cold start)
  int nInf;     // nInf : number of constraints outside of the bounds
  double sInf;  // sInf : sum of infeasibilities

  status_ = snopt.solve(
      Cold, snopt.neF, snopt.n, snopt.ObjAdd, snopt.ObjRow,
      &SnoptAdapter::ObjectiveAndConstraintFct, snopt.iAfun, snopt.jAvar,
      snopt.A, snopt.neA, snopt.iGfun, snopt.jGvar, snopt.neG, snopt.xlow,
      snopt.xupp, snopt.Flow, snopt.Fupp, snopt.x, snopt.xstate, snopt.xmul,
      snopt.F, snopt.Fstate, snopt.Fmul, nS, nInf, sInf);
#else
  status_ = snopt.solve(Cold);
#endif

  int EXIT = status_ - status_ % 10;  // change least significant digit to zero

  if (EXIT != 0) {
    std::string msg =
        "ERROR: Snopt failed to find a solution. EXIT:" + std::to_string(EXIT) +
        ", INFO:" + std::to_string(status_) + "\n";
    throw std::runtime_error(msg);
  }

  snopt.SetVariables();
}

} /* namespace ifopt */
