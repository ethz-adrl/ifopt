﻿/******************************************************************************
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

#include <ifopt/composite.h>

#include <iostream>
#include <iomanip>

namespace ifopt {

    Component::Component(int num_rows, const std::string& name)
    {
        num_rows_ = num_rows;
        name_ = name;
    }

    int
        Component::GetRows() const
    {
        return num_rows_;
    }

    void
        Component::SetRows(int num_rows)
    {
        num_rows_ = num_rows;
    }

    std::string
        Component::GetName() const
    {
        return name_;
    }

    void Component::Print(double tol, int& index) const
    {
        // calculate squared bound violation
        VectorXd x = GetValues();
        VecBound bounds = GetBounds();

        std::vector<int> viol_idx;
        for (std::size_t i = 0; i < bounds.size(); ++i) {
            double lower = bounds.at(i).lower_;
            double upper = bounds.at(i).upper_;
            double val = x(i);
            if (val < lower - tol || upper + tol < val)
                viol_idx.push_back(i); // constraint out of bounds
        }

        std::string black = "\033[0m";
        std::string red = "\033[31m";
        std::string color = viol_idx.empty() ? black : red;

        std::cout.precision(2);
        std::cout << std::fixed
            << std::left
            << std::setw(30) << name_
            << std::right
            << std::setw(4) << num_rows_
            << std::setw(9) << index
            << std::setfill('.')
            << std::setw(7) << index + num_rows_ - 1
            << std::setfill(' ')
            << color
            << std::setw(12) << viol_idx.size()
            << black
            << std::endl;

        index += num_rows_;
    }

    Composite::Composite(const std::string& name, bool is_cost) :Component(0, name)
    {
        is_cost_ = is_cost;
    }

    void
        Composite::AddComponent(const Component::Ptr& c)
    {
        // at this point the number of rows must be specified.
        assert(c->GetRows() != kSpecifyLater);

        components_.push_back(c);

        if (is_cost_)
            SetRows(1);
        else
            SetRows(GetRows() + c->GetRows());
    }

    void
        Composite::ClearComponents()
    {
        components_.clear();
        SetRows(0);
    }

    const Component::Ptr
        Composite::GetComponent(std::string name) const
    {
        for (const auto& c : components_)
            if (c->GetName() == name)
                return c;

        assert(false); // component with name doesn't exist, abort program
        return Component::Ptr();
    }

    Composite::VectorXd
        Composite::GetValues() const
    {
        VectorXd g_all = VectorXd::Zero(GetRows());

        int row = 0;
        for (const auto& c : components_) {
            int n_rows = c->GetRows();
            VectorXd g = c->GetValues();
            g_all.middleRows(row, n_rows) += g;

            if (!is_cost_)
                row += n_rows;
        }
        return g_all;
    }

    void
        Composite::SetVariables(const VectorXd& x)
    {
        int row = 0;
        for (auto& c : components_) {
            int n_rows = c->GetRows();
            c->SetVariables(x.middleRows(row, n_rows));
            row += n_rows;
        }
    }

    Composite::Jacobian
        Composite::GetJacobian() const
    { // set number of variables only the first time this function is called,
        // since number doesn't change during the optimization. Improves efficiency.
        if (n_var == -1)
            n_var = components_.empty() ? 0 : components_.front()->GetJacobian().cols();

        Jacobian jacobian(GetRows(), n_var);

        if (n_var == 0) return jacobian;

        int row = 0;
        std::vector< Eigen::Triplet<double> > triplet_list;

        for (const auto& c : components_) {
            const Jacobian& jac = c->GetJacobian();
            triplet_list.reserve(triplet_list.size() + jac.nonZeros());

            for (int k = 0; k < jac.outerSize(); ++k)
                for (Jacobian::InnerIterator it(jac, k); it; ++it)
                    triplet_list.push_back(Eigen::Triplet<double>(row + it.row(), it.col(), it.value()));

            if (!is_cost_)
                row += c->GetRows();
        }

        jacobian.setFromTriplets(triplet_list.begin(), triplet_list.end());
        return jacobian;
    }

    Component::Jacobian Composite::GetHession(double obj_factor, const  double* lambuda) const
    {
        if (m_var == -1)
        {
            m_var = components_.empty() ? 0 : components_.front()->GetSingleHession(0).cols();
        }
        Jacobian Hes(m_var, m_var);
        Jacobian Hes_Transition_vessel(m_var * 2, m_var * 2);
        //Prevents triples from exceeding the maximum range of the matrix
        int maximum_element = m_var * m_var;
        int elementcounter = 0;
        if (m_var == 0)
        {
            return Hes;
        }
        int row = 0;
        std::vector< Eigen::Triplet<double> > triplet_list;
        double intermidiate;
        for (const auto& c : components_)
        {
            int SIZE = c->GetRows();
            for (int i = 0; i < SIZE; i++)
            {
                if (!is_cost_)
                {
                    intermidiate = lambuda[i + row];
                }
                else
                {
                    intermidiate = obj_factor;
                }
                const Jacobian& jac = intermidiate * c->GetSingleHession(i);
                triplet_list.reserve(triplet_list.size() + jac.nonZeros());
                for (int k = 0; k < jac.outerSize(); ++k)
                {
                    for (Jacobian::InnerIterator it(jac, k); it; ++it)
                    {
                        triplet_list.push_back(Eigen::Triplet<double>(it.row(), it.col(), it.value()));
                        elementcounter += 1;
                        if (elementcounter == maximum_element)
                        {
                            Hes_Transition_vessel.setFromTriplets(triplet_list.begin(), triplet_list.end());
                            triplet_list.clear();
                            elementcounter = 0;
                            //Prevents triples from exceeding the maximum range of the matrix
                            for (int kin = 0; kin < Hes_Transition_vessel.outerSize(); ++kin)
                            {
                                for (Jacobian::InnerIterator itin(Hes_Transition_vessel, kin); itin; ++itin)
                                {
                                    triplet_list.push_back(Eigen::Triplet<double>(itin.row(), itin.col(), itin.value()));
                                    elementcounter += 1;
                                }
                            }
                            Hes_Transition_vessel.setZero();
                            //If the triplet has more elements than the final matrix,
                            //use an intermediate matrix to transition
                        }
                    }
                }
            }



            if (!is_cost_)
            {
                row += c->GetRows();
            }
        }
        Hes.setFromTriplets(triplet_list.begin(), triplet_list.end());
        return Hes;
    }

    Component::Jacobian Composite::GetSingleHession(int irow) const
    {
        throw std::runtime_error("GetSingleHession not implemented for Composite");
        //this will not be valid
    }

    Composite::VecBound
        Composite::GetBounds() const
    {
        VecBound bounds_;
        for (const auto& c : components_) {
            VecBound b = c->GetBounds();
            bounds_.insert(bounds_.end(), b.begin(), b.end());
        }

        return bounds_;
    }

    const Composite::ComponentVec
        Composite::GetComponents() const
    {
        return components_;
    }

    size_t Composite::GetMvar() const
    {
        return m_var;
    }

    void
        Composite::PrintAll() const
    {
        int index = 0;
        double tol = 0.001; ///< tolerance when printing out constraint/bound violation.

        std::cout << GetName() << ":\n";
        for (auto c : components_) {
            std::cout << "   "; // indent components
            c->Print(tol, index);
        }
        std::cout << std::endl;
    }

} /* namespace opt */
