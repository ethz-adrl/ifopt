^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ifopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.3 (2022-03-24)
------------------
* Enable building of static libs
* (`#77 <https://github.com/ethz-adrl/ifopt/issues/77>`_) Implement Required Method
* Fix CMake install() rules for Windows DLLs
* Contributors: Josh Langsfeld, Konstantinos Chatzilygeroudis, Rafael Rojas

2.1.2 (2021-11-29)
------------------
* Expose epsilon for cost gradient finite difference approximation
* Contributors: Levi Armstrong

2.1.1 (2021-11-15)
------------------
* replace static variable falsely shared between contraints and costs
* Reduce calls to GetJacobian
* Changes to support windows build (#65)
* set default parameter or finite diff for backwards compatibility
* add return-status getter also to SNOPT
* calculate finite difference of cost-term for IPOPT if flag set. (#61)
* add explanation setting jacobian sparsity pattern (#47) (#55)
* Update README.md
* Switch  between catkin and ament based on ROS_VERSION (#52)
* Add GetConstraints and GetCosts to Problem (#51)
* Add GetJacobianOfCosts to Problem (#50)
* Make FillJacobianBlock public in ConstraintSet (#49)

2.0.7 (2019-11-19)
------------------
* Create function to get the time statistics and the return code from the optimization solver. (`#40 <https://github.com/ethz-adrl/ifopt/issues/40>`_)
* Contributors: viviansuzano

2.0.6 (2019-01-17)
------------------
* Enable problems without constraints: Fix issue (`#34 <https://github.com/ethz-adrl/ifopt/issues/34>`_ , `#35 <https://github.com/ethz-adrl/ifopt/issues/35>`_)
* Fix/segfaults (`#33 <https://github.com/ethz-adrl/ifopt/issues/33>`_)
* Contributors: Wolfgang Merkt, viviansuzano

2.0.5 (2018-07-30)
------------------
* set default print level to 4 to show derivative test errors.
* Improve print-out of cost terms (specifically print cost)
* Improve docs (`#27 <https://github.com/ethz-adrl/ifopt/issues/27>`_)
* Implemented more efficient method for building constraint jacobian (`#26 <https://github.com/ethz-adrl/ifopt/issues/26>`_)
* Contributors: Alexander Winkler, fbiemueller

2.0.4 (2018-07-17)
------------------
* generalize ipopt solver interface, so source never has to be touched.
* Fix/simplify image path in doxygen.
* Simplify and generalize testing procedure (`#25 <https://github.com/ethz-adrl/ifopt/issues/25>`_)
* have cmake fail if IPOPT version <3.11.9
* Install binaries to /lib/ifopt, just as catkin does.
* Update README.md to always use `make test` for testing.
* Improve doxygen (`#22 <https://github.com/ethz-adrl/ifopt/issues/22>`_)
* Contributors: Alexander Winkler

2.0.3 (2018-07-10)
------------------
* Add codefactor integration and contributing guidelines
* display indices more precisely in printout
* remove rsl jekins, use ros build farm
* remove warning from version number
* Contributors: Alexander Winkler

2.0.2 (2018-07-02)
------------------
* fully remove catkin dependency
* increase cmake project version number manually
* DRY in cmake list (reuse ${LIB_CORE})
* Update README.md
* Contributors: Alexander Winkler

2.0.1 (2018-06-30)
------------------
* make IPOPT the default ON solver option
* Add documentation and update package.xml to use ubuntu ipopt install coinor-libipopt-dev
* Use FindIpopt.cmake (from robotology/idyntree)
* Set default solver to mumps, as this is free one installed in ubuntu
* Define SNOPT/IPOPT location also through environmental variable
* Contributors: Alexander Winkler

2.0.0 (2018-06-24)
------------------
* allow building with pure cmake (catkin optional) (`#13 <https://github.com/ethz-adrl/ifopt/issues/13>`_)
* generate ifopt-config.cmake to easily include in other cmake projects (`#13 <https://github.com/ethz-adrl/ifopt/issues/13>`_)
* implement pimpl idiom to avoid exporting IPOPT/SNOPT libraries/headers (`#12 <https://github.com/ethz-adrl/ifopt/issues/12>`_)
* Add possibility to set solver options (e.g. "ma27") on user side
* Clean-up and improve printouts
* Reduce to one single catkin package with solvers as cmake subdirectories
* Contributors: Alexander Winkler

1.0.2 (2018-02-05)
------------------
* add correct catkin install folder for ifopt_core
* Contributors: Alexander Winkler

1.0.1 (2018-01-29)
------------------
* update package xml
* make eigen 3.2 compatible (remove header Eigen/Eigen)
* Contributors: Alexander Winkler

1.0.0 (2018-01-27)
------------------
* move IPOPT and SNOPT interfaces to separate package
* add ifopt metapackage for documentation
* Contributors: Alexander Winkler

0.0.2 (2018-01-04)
------------------
* added more explanatory message in package.xml
* added documentations badge
* fixed bounds in example problem
* moved CI to ros buildfarm.
* generate tests also with catkin if installed
* Fixed compiler warnings.
* Contributors: Alexander W Winkler

0.0.1 (2017-12-15)
------------------
* improve doxygen documentation
* removed linear and soft constraint
* add more unit tests
* added logo
* Create LICENSE
* added first version of readme
* remove dependency of constraints/cost on optimization variables
* simplified user interface.
* make project catkin independent
* renamed repo to ifopt
* added ipopt linear solver types url
* opt_solve: add test infrastructure
* add documentation to core classes
* xpp_solve: add copyright boilerplate
* Contributors: Alexander W Winkler
