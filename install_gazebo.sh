#! /bin/bash

set -e

sudo apt install -y libtinyxml-dev uuid-dev libfreeimage-dev libgts-dev libavdevice-dev libavformat-dev libavcodec-dev libswscale-dev libavutil-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libtbb-dev freeglut3-dev libogre-1.9-dev libtar-dev libqwt-qt5-dev libqt5core5a libqt5opengl5-dev libqt5test5 libqt5opengl5 libqt5widgets5 

mkdir -p $HOME/gazebo_build
cd $HOME/gazebo_build

rm -rf zeromq-4.2.3
wget https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz -O - | tar xzf -
cd zeromq-4.2.3
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig LDFLAGS="-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib" CFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde" CXXFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/include" ./configure --prefix=/nubots/toolchain/native
make -j$(nproc) && sudo make install
cd ..

rm -rf czmq-4.1.1
wget https://github.com/zeromq/czmq/releases/download/v4.1.1/czmq-4.1.1.tar.gz -O - | tar xzf -
cd czmq-4.1.1
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig LDFLAGS="-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib" CFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde" CXXFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/include" ./configure --prefix=/nubots/toolchain/native
make -j$(nproc) && sudo make install
cd ..

rm -rf cppzmq-4.2.2
wget https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz -O - | tar xzf -
cd cppzmq-4.2.2
mkdir build && cd build
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf assimp-4.1.0
wget https://github.com/assimp/assimp/archive/v4.1.0.tar.gz -O - | tar xzf -
cd assimp-4.1.0
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf libccd-2.0
wget https://github.com/danfis/libccd/archive/v2.0.tar.gz -O - | tar xzf -
cd libccd-2.0
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_SHARED_LIBS=ON ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf flann-1.9.1
wget https://github.com/mariusmuja/flann/archive/1.9.1.tar.gz -O - | tar xzf -
cd flann-1.9.1
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf octomap-1.7.2
wget https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz -O - | tar xzf -
cd octomap-1.7.2
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_OCTOVIS_SUBPROJECT=OFF -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF ../ -G Ninja
ninja && sudo ninja install
rm -rf ./*
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_OCTOVIS_SUBPROJECT=OFF ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf fcl-0.3.4
wget https://github.com/flexible-collision-library/fcl/archive/0.3.4.tar.gz -O - | tar xzf -
cd fcl-0.3.4
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DOCTOMAP_LIBRARY_DIRS=/nubots/toolchain/native/lib -DCCD_LIBRARY_DIRS=/nubots/toolchain/native/lib ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf nlopt-2.4.2
wget http://ab-initio.mit.edu/nlopt/nlopt-2.4.2.tar.gz -O - | tar xzf -
cd nlopt-2.4.2
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig LDFLAGS="-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib" CFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde" CXXFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/include" ./configure --prefix=/nubots/toolchain/native --without-python --without-octave --without-matlab --with-cxx
make -j$(nproc) && sudo make install
cd ..

rm -rf Ipopt-3.11.9
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.9.tgz -O - | tar xzf -
cd Ipopt-3.11.9
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig LDFLAGS="-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib" CFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde" CXXFLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/include" ./configure --prefix=/nubots/toolchain/native --with-blas="-lopenblas" --with-lapack="-lopenblas"
make -j$(nproc) && sudo make install
cd ..

rm -rf tinyxml2-2.2.0
wget https://github.com/leethomason/tinyxml2/archive/2.2.0.tar.gz -O - | tar xzf -
cd tinyxml2-2.2.0
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native  ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf console_bridge-0.4.0
wget https://github.com/ros/console_bridge/archive/0.4.0.tar.gz -O - | tar xzf -
cd console_bridge-0.4.0
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native  ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf urdfdom_headers-0.4.2
wget https://github.com/ros/urdfdom_headers/archive/0.4.2.tar.gz -O - | tar xzf -
cd urdfdom_headers-0.4.2
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native  ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf urdfdom-0.4.2
wget https://github.com/ros/urdfdom/archive/0.4.2.tar.gz -O - | tar xzf -
cd urdfdom-0.4.2
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native  ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf bullet3-2.87
wget https://github.com/bulletphysics/bullet3/archive/2.87.tar.gz -O - | tar xzf -
cd bullet3-2.87
mkdir build && cd build
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_PYBULLET=OFF -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -D_FIND_LIB_PYTHON_PY=/home/vagrant/gazebo/bullet3-2.87/build3/cmake/FindLibPython.py -DINSTALL_LIBS=ON -DBUILD_SHARED_LIBS=ON ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf dart-6.2.0
wget https://github.com/dartsim/dart/archive/v6.2.0.tar.gz -O - | tar xzf -
cd dart-6.2.0
cat << EOF | patch -Np1
--- a/CMakeLists.txt	2018-03-29 23:31:55.322178567 +1100
+++ b/CMakeLists.txt	2018-03-29 23:26:58.438178567 +1100
@@ -82,6 +82,7 @@
 # errors.
 option(DART_ENABLE_SIMD
   "Build DART with all SIMD instructions on the current local machine" OFF)
+option(DART_BUILD_GUI "Build gui" ON)
 option(DART_BUILD_GUI_OSG "Build osgDart library" ON)
 option(DART_CODECOV "Turn on codecov support" OFF)
 option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

--- a/dart/CMakeLists.txt	2018-03-29 23:32:53.722178567 +1100
+++ b/dart/CMakeLists.txt	2018-03-29 23:29:20.898178567 +1100
@@ -86,7 +86,10 @@
 add_subdirectory(simulation)
 add_subdirectory(planning) # flann
 add_subdirectory(utils) # tinyxml, tinyxml2, bullet
-add_subdirectory(gui) # opengl, glut, bullet
+
+IF(DART_BUILD_GUI)
+    add_subdirectory(gui) # opengl, glut, bullet
+ENDIF(DART_BUILD_GUI)
 
 set(DART_CONFIG_HPP_IN ${CMAKE_SOURCE_DIR}/dart/config.hpp.in)
 set(DART_CONFIG_HPP_OUT ${CMAKE_BINARY_DIR}/dart/config.hpp)

--- a/dart/gui/CMakeLists.txt	2018-03-29 23:33:21.414178567 +1100
+++ b/dart/gui/CMakeLists.txt	2018-03-29 23:33:32.946178567 +1100
@@ -59,7 +59,9 @@
 )
 
 # Add subdirectories
-add_subdirectory(osg)
+IF(DART_BUILD_GUI_OSG)
+    add_subdirectory(osg)
+ENDIF(DART_BUILD_GUI_OSG)
 
 # Generate header for this namespace
 dart_get_filename_components(header_names "gui headers" ${hdrs})
EOF
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DDART_BUILD_GUI=OFF ../ -G Ninja
ninja && sudo ninja install
cd ../../

rm -rf ignitionrobotics-ign-cmake-*
wget https://bitbucket.org/ignitionrobotics/ign-cmake/get/ignition-cmake_0.5.0.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-cmake-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-math-*
wget https://bitbucket.org/ignitionrobotics/ign-math/get/ignition-math4_4.0.0.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-math-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-tools-*
wget https://bitbucket.org/ignitionrobotics/ign-tools/get/default.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-tools-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-msgs-*
wget https://bitbucket.org/ignitionrobotics/ign-msgs/get/ignition-msgs_1.0.0.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-msgs-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-transport-*
wget https://bitbucket.org/ignitionrobotics/ign-transport/get/ignition-transport4_4.0.0.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-transport-*
mkdir build && cd build
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-common-*
wget https://bitbucket.org/ignitionrobotics/ign-common/get/ignition-common_1.0.1.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-common-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DFREEIMAGE_RUNS=1 -DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1 ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf ignitionrobotics-ign-fuel-tools-*
wget https://bitbucket.org/ignitionrobotics/ign-fuel-tools/get/ignition-fuel_tools_1.0.0.tar.bz2 -O - | tar xjf -
cd ignitionrobotics-ign-fuel-tools-*
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DFREEIMAGE_RUNS=1 -DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1 ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf sdformat-6.0.0
wget http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-6.0.0.tar.bz2 -O - | tar xjf -
cd sdformat-6.0.0
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf simbody-Simbody-3.6
wget https://github.com/simbody/simbody/archive/Simbody-3.6.tar.gz -O - | tar xzf -
cd simbody-Simbody-3.6
mkdir build && cd build
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DBUILD_USING_OTHER_LAPACK=/nubots/toolchain/native/lib/libopenblas.so ../ -G Ninja
ninja && sudo ninja install
cd ../..

rm -rf osrf-gazebo-*
wget https://bitbucket.org/osrf/gazebo/get/gazebo9_9.0.0.tar.bz2 -O - | tar xjf -
cd osrf-gazebo-*
cat << EOF > patch
--- a/gazebo/common/CMakeLists.txt	2018-03-29 23:53:12.078178567 +1100
+++ b/gazebo/common/CMakeLists.txt	2018-03-29 22:52:32.754178567 +1100
@@ -260,6 +260,7 @@
   \${TBB_LIBRARIES}
   \${SDFormat_LIBRARIES}
   \${IGNITION-FUEL_TOOLS_LIBRARIES}
+  \${IGNITION-COMMON_LIBRARIES}
 )
 
 if (UNIX)
EOF
patch -Np1 -i ./patch
mkdir build && cd build
PKG_CONFIG_PATH=/nubots/toolchain/native/lib/pkgconfig cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/native.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/nubots/toolchain/native -DFREEIMAGE_RUNS=1  -DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1 ../ -G Ninja
ninja -j1 && sudo ninja install
cd ../..

cd $HOME
