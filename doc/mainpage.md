IFOPT - Interface for Nonlinear Optimizers {#mainpage}
---------------------

*A modern, light-weight, [Eigen]-based C++ interface to Nonlinear Programming solvers, such as [Ipopt] and [Snopt].* 

|Solves a Nonlinear Optimization Problem as: | |
| -------|------ |
| \image html example_nlp.png | |

* **To see how this problem is formulated, see test_vars_constr_cost.h.** 
* **Afterwards the problem can be solved using e.g. IPOPT as shown in ex_test_ipopt.cc**.

Features:
* Related variables and constraints are implemented (grouped) in *independent sets*. Ifopt automatically generates the overall problem from these sets. No more changing indices in your variable vector or Jacobian when adding or removing variables/constraints. See this large [problem](https://i.imgur.com/4yhohZF.png), that requires multiple variable- and constraint sets to generate the motion for legged robot (implemented in [towr]).
* [Eigen] allows inuitive formulation and fast performance due to sparse matrix exploitation.  
* exports cmake scripts to easily `find_package(ifopt)` in your project.  
* [catkin] integration (optional).  
* light-weight (~[2k lines of code](https://i.imgur.com/NCPJsSw.png)) makes it easy to use and extend.  


[TOC]

Install {#install}
========================
In case the binaries for you ROS distro don't exist, for pure [CMake] build
as well as all required dependencies, please see
github 
<a href="https://github.com/ethz-adrl/ifopt/blob/master/README.md">
README.md
</a>.

In all other cases, this should work for you:
\code{.sh}
sudo apt-get install ros-<ros-distro>-ifopt
\endcode

Linking to custom Ipopt or Snopt {#link}
---------------------------
If you are building from source and want to use a locally installed version of [Ipopt] add the path to your
Ipopt build folder to your `~/.bashrc`, e.g.
\code{.sh}
export IPOPT_DIR=/home/your_name/Code/Ipopt-3.12.8/build
\endcode

In case your OS doesn't provide the precompiled binaries or the required version,
you can also easily install Ipopt from source as described [here](https://www.coin-or.org/Ipopt/documentation/node14.html). This summary might work for you:
\code{.sh}
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.10.zip
unzip Ipopt-3.11.10.zip
cd Ipopt-3.11.10/ThirdParty/Mumps
./get.Mumps  # HSL routines are faster (http://www.hsl.rl.ac.uk/ipopt/)
cd ../../
mkdir build && cd build
../configure --prefix=/usr/local
make
make test
make install
export IPOPT_DIR=`pwd`
\endcode

If you need an interface to [Snopt], point cmake to that build folder in your `~/.bashrc` through e.g.
\code{.sh}
export SNOPT_DIR=/home/your_name/Code/Snopt
\endcode
and run cmake as 
\code{.sh}
cmake -DBUILD_SNOPT=ON ..
\endcode


Contribute {#contribute}
==========================
We love pull requests. Please see 
<a href="https://github.com/ethz-adrl/ifopt/blob/master/CONTRIBUTING.md">
CONTRIBUTING.md
</a> if you want to contribute.
See here the list of 
[contributors](https://github.com/ethz-adrl/ifopt/graphs/contributors) 
who participated in this project.
 

Authors {#authors}
=======================
[Alexander W. Winkler](http://awinkler.me) - Initial Developer & Maintenance


[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[catkin]: http://wiki.ros.org/catkin
[Eigen]: http://eigen.tuxfamily.org
[CMake]: https://cmake.org/cmake/help/v3.0/
[Ipopt]: https://projects.coin-or.org/Ipopt
[Snopt]: http://ampl.com/products/solvers/solvers-we-sell/snopt/
[towr]: https://github.com/ethz-adrl/towr


