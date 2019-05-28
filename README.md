This package tests the pinnochio library:

# Dependency Installation from Source
````
https://github.com/stack-of-tasks/pinocchio
Relevant Paper: https://hal.laas.fr/hal-01866228v2/document

Installation from source so far:

Required dependencies:
    Eigen3 for linear algebra. Version >= 3.0.5
    Boost with components filesystem, unit_test_framework and system. Version >= 1.34
    eigenpy

Optional dependencies:
    Boost with component python. Version >= 1.34. Required for the python bindings.
    EigenPy An efficient binding between Numpy and Eigen using boost::python. Version >= 1.3.0. Required for the python bindings.
    FCL the Fast Collision Library. Version >= 0.4.1. Useful for collision detections.
    assimp for the reading of raw mesh files. Version >= 3.0.0.
    urdfdom for the reading of URDF models. Version >= 0.2.10.

mkdir ~/external_dependencies
cd ~/external_dependencies


git clone --recursive https://github.com/stack-of-tasks/eigenpy
cd eigenpy
git checkout master
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j4
sudo make install

git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio 
git checkout master
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j4
sudo make install

Configure environment variables
All the packages will be installed in the CMAKE_INSTALL_PREFIX path, setting by default to point to /usr/local. To make use of installed libraries and programs, you must need to configure your PATH, PKG_CONFIG_PATH, PYTHONPATH and other similar environment variables to point inside this directory. For instance:

Add the following lines to your ~/.bashrc file:

PATH="/usr/local/bin:$PATH"
PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH"
LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
PYTHONPATH="/usr/local/lib/python2.7/site-packages:$PYTHONPATH"

export PATH
export PKG_CONFIG_PATH
export LD_LIBRARY_PATH
export PYTHONPATH
````