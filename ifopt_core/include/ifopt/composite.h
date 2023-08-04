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

/**
 * @file   composite.h
 * @brief  Declares the classes Composite and Component used as variables,
 * costs and constraints.
 */

#ifndef IFOPT_INCLUDE_OPT_COMPOSITE_H_
#define IFOPT_INCLUDE_OPT_COMPOSITE_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "bounds.h"

namespace ifopt {

	/**
	 * @brief Interface representing either Variable, Cost or Constraint.
	 *
	 * Variables, costs and constraints can all be fit into the same interface
	 * (Component). For example, each has a "value", which is either the actual
	 * value of the variables, the constraint value g or the cost.
	 * This representation provides one common interface
	 * ("smallest common denominator") that can be contain either individual
	 * variables/costs/constraints or a Composite of these. This pattern
	 * takes care of stacking variables, ordering Jacobians and providing bounds
	 * for the complete problem without duplicating code. For more information on
	 * the composite pattern visit https://sourcemaking.com/design_patterns/composite
	 */
	class Component {
	public:
		using Ptr = std::shared_ptr<Component>;

		using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;
		using VectorXd = Eigen::VectorXd;
		using VecBound = std::vector<Bounds>;

		/**
		 * @brief  Creates a component.
		 * @param  num_rows  The number of rows of this components.
		 * @param  name  The identifier for this component.
		 *
		 * The number of rows @c num_rows can represent either
		 * @li number of variables in this variables set
		 * @li number of constraints in this constraint set
		 * @li 1 if this component represents a Cost.
		 */
		Component(int num_rows, const std::string& name);
		virtual ~Component() = default;

		/**
		 * @brief  Returns the "values" of whatever this component represents.
		 *
		 * @li For Variable this represents the actual optimization values.
		 * @li For Constraint this represents the constraint value g.
		 * @li For Cost this represents the cost value.
		 */
		virtual VectorXd GetValues() const = 0;

		/**
		 * @brief  Returns the "bounds" of this component.
		 *
		 * @li For Variable these are the upper and lower variable bound.
		 * @li For Constraint this represents the constraint bounds.
		 * @li For Cost these done't exists (set to infinity).
		 */
		virtual VecBound GetBounds() const = 0;

		/**
		 * @brief  Sets the optimization variables from an Eigen vector.
		 *
		 * This is only done for Variable, where these are set from the current
		 * values of the @ref solvers.
		 */
		virtual void SetVariables(const VectorXd& x) = 0;

		/**
		 * @brief  Returns derivatives of each row w.r.t. the variables
		 *
		 * @li For Constraint this is a matrix with one row per constraint.
		 * @li For a Cost this is a row vector (gradient transpose).
		 * @li Not sensible for Variable.
		 */
		virtual Jacobian GetJacobian() const = 0;
		virtual Jacobian GetHession(double obj_factor, const double* lambuda) const = 0;
		virtual Jacobian GetSingleHession(int irow)  const = 0;
		/**
		 * @brief Returns the number of rows of this component.
		 */
		int GetRows() const;

		/**
		 * @brief Returns the name (id) of this component.
		 */
		std::string GetName() const;

		/**
		 * @brief Prints the relevant information (name, rows, values) of this component.
		 * @param tolerance  When to flag constraint/bound violation.
		 * @param index_start  Of this specific variables-, constraint- or cost set.
		 */
		virtual void Print(double tolerance, int& index_start) const;

		/**
		 * @brief Sets the number of rows of this component.
		 *
		 * @attention This should correctly be done through constructor call, only
		 * delay this by using @c kSpecifyLater if you have good reason.
		 */
		void SetRows(int num_rows);
		static const int kSpecifyLater = -1;

	private:
		int num_rows_ = kSpecifyLater;
		std::string name_;
	};



	/**
	 * @brief A collection of components which is treated as another Component.
	 *
	 * This class follows the Component interface as well, but doesn't actually
	 * do any evaluation, but only stitches together the results of the
	 * components it is holding. This is where multiple sets of variables,
	 * constraints or costs are ordered and combined.
	 *
	 * See Component and Composite Pattern for more information.
	 */
	class Composite : public Component {
	public:
		using Ptr = std::shared_ptr<Composite>;
		using ComponentVec = std::vector<Component::Ptr>;

		/**
		 * @brief  Creates a Composite holding either variables, costs or constraints.
		 * @param  is_cost  True if this class holds cost terms, false for all others.
		 *
		 * Constraints and variables append individual constraint sets and Jacobian
		 * rows below one another, whereas costs terms are all accumulated to a
		 * scalar value/a single Jacobian row.
		 */
		Composite(const std::string& name, bool is_cost);
		virtual ~Composite() = default;

		// see Component for documentation
		VectorXd GetValues() const override;
		Jacobian GetJacobian() const override;

		Jacobian GetHession(double obj_factor, const  double* lambuda) const override;
		Jacobian GetSingleHession(int irow)  const override;

		VecBound GetBounds() const override;
		void SetVariables(const VectorXd& x) override;
		void PrintAll() const;

		/**
		 * @brief  Access generic component with the specified name.
		 * @param  name  The name given to the component.
		 * @return A generic pointer of that component.
		 */
		const Component::Ptr GetComponent(std::string name) const;

		/**
		 * @brief  Access type-casted component with the specified name.
		 * @param  name  The name given to the component.
		 * @tparam T  Type of component.
		 * @return A type-casted pointer possibly providing addtional functionality.
		 */
		template<typename T> std::shared_ptr<T>
		GetComponent(const std::string& name) const;

		/**
		 * @brief Adds a component to this composite.
		 */
		void AddComponent(const Component::Ptr&);

		/**
		 * @brief Removes all component from this composite.
		 */
		void ClearComponents();

		/**
		 * @brief Returns read access to the components.
		 */
		const ComponentVec GetComponents() const;

		size_t GetMvar() const;

	private:
		ComponentVec components_;
		bool is_cost_;
		// The number of variables for costs/constraint composites (not set for variables).
		// Is initialized the first the GetJacobian() is called.
		mutable size_t n_var = -1;
		mutable size_t m_var = -1;
	};


	// implementation of template functions
	template<typename T>
	std::shared_ptr<T> Composite::GetComponent(const std::string& name) const
	{
		Component::Ptr c = GetComponent(name);
		return std::dynamic_pointer_cast<T>(c);
	}


} /* namespace opt */

#endif /* IFOPT_INCLUDE_OPT_COMPOSITE_H_ */
