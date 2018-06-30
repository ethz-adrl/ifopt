^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ifopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
