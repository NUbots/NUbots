#! /bin/bash

set -e

PREFIX="/nubots/toolchain/native"
LINK_FLAGS="-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib"
C_FLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde"
CXX_FLAGS="-I/nubots/toolchain/native/include -I/nubots/toolchain/inclulde"
PKG_CFG_PATH="/nubots/toolchain/native/lib/pkgconfig"
TOOLCHAIN_FILE="/nubots/toolchain/native.cmake"

function purge_folder {
  if [ -d $1 ];
  then
    rm -rf $1
  fi
}

function download_and_extract {
  wget $1 -O $(basename $1)
  tar xf $(basename $1)
}

function autotools_build {
  PKG_CONFIG_PATH="${PKG_CFG_PATH}" \
    LDFLAGS="${LINK_FLAGS}" \
    CFLAGS="${C_FLAGS}" \
    CXXFLAGS="${CXX_FLAGS}" \
    ./configure --prefix="${PREFIX}" "$@"
  make -j$(nproc)
  sudo make install
}

function cmake_build {
  purge_folder build
  cd build
  PKG_CONFIG_PATH="${PKG_CFG_PATH}" \
    cmake -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
          "$@" \
          ../ -G Ninja
    ninja
    sudo ninja install
    cd ..
}

sudo apt install -y libtinyxml-dev uuid-dev libfreeimage-dev libgts-dev libavdevice-dev libavformat-dev libavcodec-dev libswscale-dev libavutil-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libtbb-dev freeglut3-dev libogre-1.9-dev libtar-dev libqwt-qt5-dev libqt5core5a libqt5opengl5-dev libqt5test5 libqt5opengl5 libqt5widgets5

mkdir -p $HOME/gazebo_build
cd $HOME/gazebo_build

if [ ! -e zeromq.complete ];
then
  purge_folder "zeromq-4.2.3"
  download_and_extract "https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz"
  cd zeromq-4.2.3
  autotools_build
  cd ..
  touch zeromq.complete
fi

if [ ! -e czmq.complete ];
then
  purge_folder "czmq-4.1.1"
  download_and_extract "https://github.com/zeromq/czmq/releases/download/v4.1.1/czmq-4.1.1.tar.gz"
  cd czmq-4.1.1
  autotools_build
  cd ..
  touch czmq.complete
fi

if [ ! -e cppzmq.complete ];
then
  purge_folder "cppzmq-4.2.2"
  download_and_extract "https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz"
  cd cppzmq-4.2.2
  cmake_build
  cd ..
  touch cppzmq.complete
fi

if [ ! -e assimp.complete ];
then
  purge_folder "assimp-4.1.0"
  download_and_extract "https://github.com/assimp/assimp/archive/v4.1.0.tar.gz"
  cd assimp-4.1.0
  cmake_build
  cd ..
  touch assimp.complete
fi

if [ ! -e libccd.complete ];
then
  purge_folder "libccd-2.0"
  download_and_extract "https://github.com/danfis/libccd/archive/v2.0.tar.gz"
  cd libccd-2.0
  cmake_build "-DBUILD_SHARED_LIBS=ON"
  cd ..
  touch libccd.complete
fi

if [ ! -e flann.complete ];
then
  purge_folder "flann-1.9.1"
  download_and_extract "https://github.com/mariusmuja/flann/archive/1.9.1.tar.gz"
  cd flann-1.9.1
cat << EOF > patch
--- a/src/cpp/CMakeLists.txt
+++ b/src/cpp/CMakeLists.txt
@@ -29,7 +29,7 @@
 endif()

 if(CMAKE_SYSTEM_NAME STREQUAL \"Linux\" AND CMAKE_COMPILER_IS_GNUCC)
-    add_library(flann_cpp SHARED \"\")
+    add_library(flann_cpp SHARED \"empty.cpp\")
     set_target_properties(flann_cpp PROPERTIES LINKER_LANGUAGE CXX)
     target_link_libraries(flann_cpp -Wl,-whole-archive flann_cpp_s -Wl,-no-whole-archive)

@@ -43,7 +43,7 @@
 else()
     add_library(flann_cpp SHARED ${CPP_SOURCES})
     if (BUILD_CUDA_LIB)
-		cuda_add_library(flann_cuda SHARED ${CPP_SOURCES})
+		cuda_add_library(flann_cuda SHARED empty.cpp ${CPP_SOURCES})
         set_property(TARGET flann_cpp PROPERTY COMPILE_DEFINITIONS FLANN_USE_CUDA)
     endif()
 endif()
@@ -83,11 +83,11 @@
     set_property(TARGET flann_s PROPERTY COMPILE_DEFINITIONS FLANN_STATIC)

     if(CMAKE_SYSTEM_NAME STREQUAL \"Linux\" AND CMAKE_COMPILER_IS_GNUCC)
-        add_library(flann SHARED \"\")
+        add_library(flann SHARED \"empty.cpp\")
         set_target_properties(flann PROPERTIES LINKER_LANGUAGE CXX)
         target_link_libraries(flann -Wl,-whole-archive flann_s -Wl,-no-whole-archive)
     else()
-        add_library(flann SHARED ${C_SOURCES})
+        add_library(flann SHARED empty.cpp ${C_SOURCES})

         if(MINGW AND OPENMP_FOUND)
           target_link_libraries(flann gomp)
EOF
  patch -Np1 -i ./patch
  touch src/cpp/empty.cpp
  cmake_build "-DBUILD_PYTHON_BINDINGS=OFF" "-DBUILD_MATLAB_BINDINGS=OFF"
  cd ..
  touch flann.complete
fi

if [ ! -e octomap.complete ];
then
  purge_folder "octomap-1.7.2"
  download_and_extract "https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz"
  cd octomap-1.7.2
  cmake_build "-DBUILD_OCTOVIS_SUBPROJECT=OFF" "-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF"
  cmake_build "-DBUILD_OCTOVIS_SUBPROJECT=OFF"
  cd ..
  touch octomap.complete
fi

if [ ! -e fcl.complete ];
then
  purge_folder "fcl-0.3.4"
  download_and_extract "https://github.com/flexible-collision-library/fcl/archive/0.3.4.tar.gz"
  cd fcl-0.3.4
  cmake_build "-DOCTOMAP_LIBRARY_DIRS=${PREFIX}/lib" "-DCCD_LIBRARY_DIRS=${PREFIX}/lib"
  cd ..
  touch fcl.complete
fi

if [ ! -e nlopt.complete ];
then
  purge_folder "nlopt-2.4.2"
  download_and_extract "https://github.com/stevengj/nlopt/releases/download/nlopt-2.4.2/nlopt-2.4.2.tar.gz"
  cd nlopt-2.4.2
  autotools_build "--without-python" "--without-octave" "--without-matlab" "--with-cxx"
  cd ..
  touch nlopt.complete
fi

if [ ! -e ipopt.complete ];
then
  purge_folder "Ipopt-3.11.9"
  download_and_extract "https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.9.tgz"
  cd Ipopt-3.11.9
  autotools_build "F77=gfortran-7" "--with-blas=\"-lopenblas\"" "--with-lapack=\"-lopenblas\""
  cd ..
  touch ipopt.complete
fi

if [ ! -e tinyxml2.complete ];
then
  purge_folder "tinyxml2-2.2.0"
  download_and_extract "https://github.com/leethomason/tinyxml2/archive/2.2.0.tar.gz"
  cd tinyxml2-2.2.0
  cmake_build
  cd ..
  touch tinyxml2.complete
fi

if [ ! -e console_bridge.complete ];
then
  purge_folder "console_bridge-0.4.0"
  download_and_extract "https://github.com/ros/console_bridge/archive/0.4.0.tar.gz"
  cd console_bridge-0.4.0
  cmake_build
  cd ..
  touch console_bridge.complete
fi

if [ ! -e urdfdom_headers.complete ];
then
  purge_folder "urdfdom_headers-0.4.2"
  download_and_extract "https://github.com/ros/urdfdom_headers/archive/0.4.2.tar.gz"
  cd urdfdom_headers-0.4.2
  cmake_build
  cd ..
  touch urdfdom_headers.complete
fi

if [ ! -e urdfdom.complete ];
then
  purge_folder "urdfdom-0.4.2"
  download_and_extract "https://github.com/ros/urdfdom/archive/0.4.2.tar.gz"
  cd urdfdom-0.4.2
  cmake_build
  cd ..
  touch urdfdom.complete
fi

if [ ! -e bullet3.complete ];
then
  purge_folder "bullet3-2.87"
  download_and_extract "https://github.com/bulletphysics/bullet3/archive/2.87.tar.gz"
  cd bullet3-2.87
  cmake_build "-DBUILD_PYBULLET=OFF" \
              "-DBUILD_PYBULLET_NUMPY=OFF" \
              "-DUSE_DOUBLE_PRECISION=ON" \
              "-D_FIND_LIB_PYTHON_PY=../build3/cmake/FindLibPython.py" \
              "-DINSTALL_LIBS=ON" \
              "-DBUILD_SHARED_LIBS=ON"
  cd ..
  touch bullet3.complete
fi

if [ ! -e dart.complete ];
then
  purge_folder "dart-6.2.0"
  download_and_extract "https://github.com/dartsim/dart/archive/v6.2.0.tar.gz"
  cd dart-6.2.0
cat << EOF > patch
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -82,6 +82,7 @@
 # errors.
 option(DART_ENABLE_SIMD
   "Build DART with all SIMD instructions on the current local machine" OFF)
+option(DART_BUILD_GUI "Build gui" ON)
 option(DART_BUILD_GUI_OSG "Build osgDart library" ON)
 option(DART_CODECOV "Turn on codecov support" OFF)
 option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

--- a/dart/CMakeLists.txt
+++ b/dart/CMakeLists.txt
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

--- a/dart/gui/CMakeLists.txt
+++ b/dart/gui/CMakeLists.txt
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
  patch -Np1 -i ./patch
  cmake_build "-DDART_BUILD_GUI=OFF" "-DDART_BUILD_GUI_OSG=OFF"
  cd ..
  touch dart.complete
fi

if [ ! -e ignition-cmake.complete ];
then
  purge_folder "ignitionrobotics-ign-cmake-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-cmake/get/ignition-cmake_0.5.0.tar.bz2"
  cd ignitionrobotics-ign-cmake-*
  cmake_build
  cd ..
  touch ignition-cmake.complete
fi

if [ ! -e ignition-math.complete ];
then
  purge_folder "ignitionrobotics-ign-math-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-math/get/ignition-math4_4.0.0.tar.bz2"
  cd ignitionrobotics-ign-math-*
  cmake_build
  cd ..
  touch ignition-math.complete
fi

if [ ! -e ignition-tools.complete ];
then
  purge_folder "ignitionrobotics-ign-tools-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-tools/get/default.tar.bz2"
  cd ignitionrobotics-ign-tools-*
  cmake_build
  cd ..
  touch ignition-tools.complete
fi

if [ ! -e ignition-msgs.complete ];
then
  purge_folder "ignitionrobotics-ign-msgs-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-msgs/get/ignition-msgs_1.0.0.tar.bz2"
  cd ignitionrobotics-ign-msgs-*
  cmake_build
  cd ..
  touch ignition-msgs.complete
fi

if [ ! -e ignition-transport.complete ];
then
  purge_folder "ignitionrobotics-ign-transport-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-transport/get/ignition-transport4_4.0.0.tar.bz2"
  cd ignitionrobotics-ign-transport-*
  cmake_build
  cd ..
  touch ignition-transport.complete
fi

if [ ! -e ignition-common.complete ];
then
  purge_folder "ignitionrobotics-ign-common-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-common/get/ignition-common_1.0.1.tar.bz2"
  cd ignitionrobotics-ign-common-*
  cmake_build "-DFREEIMAGE_RUNS=1" "-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1" "-DBUILD_TESTING=OFF"
  cd ..
  touch ignition-common.complete
fi

if [ ! -e ignition-fuel-tools.complete ];
then
  purge_folder "ignitionrobotics-ign-fuel-tools-*"
  download_and_extract "https://bitbucket.org/ignitionrobotics/ign-fuel-tools/get/ignition-fuel_tools_1.0.0.tar.bz2"
  cd ignitionrobotics-ign-fuel-tools-*
  cmake_build "-DFREEIMAGE_RUNS=1" "-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1"
  cd ..
  touch ignition-fuel-tools.complete
fi

if [ ! -e sdformat.complete ];
then
  purge_folder "sdformat-6.0.0"
  download_and_extract "http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-6.0.0.tar.bz2"
  cd sdformat-6.0.0
  cmake_build
  cd ..
  touch sdformat.complete
fi

if [ ! -e simbody.complete ];
then
  purge_folder "simbody-Simbody-3.6"
  download_and_extract "https://github.com/simbody/simbody/archive/Simbody-3.6.tar.gz"
  cd simbody-Simbody-3.6
  cmake_build "-DBUILD_USING_OTHER_LAPACK=${PREFIX}/lib/libopenblas.so"
  cd ..
  touch simbody.complete
fi

if [ ! -e gazebo.complete ];
then
  purge_folder "osrf-gazebo-*"
  download_and_extract "https://bitbucket.org/osrf/gazebo/get/gazebo9_9.0.0.tar.bz2"
  cd osrf-gazebo-*
cat << EOF > patch
--- a/gazebo/common/CMakeLists.txt
+++ b/gazebo/common/CMakeLists.txt
@@ -260,6 +260,7 @@
   \${TBB_LIBRARIES}
   \${SDFormat_LIBRARIES}
   \${IGNITION-FUEL_TOOLS_LIBRARIES}
+  \${IGNITION-COMMON_LIBRARIES}
 )

 if (UNIX)
EOF
  patch -Np1 -i ./patch
  cmake_build "-DFREEIMAGE_RUNS=1" "-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1"
  cd ..
  touch gazebo.complete
fi
