# == Class: gazebo
class gazebo {
    $archs = {
      'native'    => {'flags'       => ['', ],
                      'params'      => ['-m64', ],
                      'environment' => {'TARGET' => 'GENERIC',
                                        'USE_THREAD' => '1',
                                        'BINARY' => '64',
                                        'NUM_THREADS' => '2',
                                        'AUDIO' => 'PORTAUDIO',
                                        'LDFLAGS' => '-m64',
                                        'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig',
                                        'CCAS' => '/usr/bin/gcc',
                                        'AS' => '/usr/bin/gcc',
                                        'CCASFLAGS' => '-m64', }, },
    }

    $archives = {
      'zeromq'              => {'url'         => 'https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz',
                                'creates'     => 'lib/libzmq.a',
                                'method'      => 'autotools', },
      'czmq'                => {'url'         => 'https://github.com/zeromq/czmq/releases/download/v4.1.1/czmq-4.1.1.tar.gz',
                                'require'     => [ Installer['zeromq'], ],
                                'method'      => 'autotools', },
      'cppzmq'              => {'url'         => 'https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz',
                                'creates'     => 'include/zmq.hpp',
                                'require'     => [ Installer['czmq'], ],
                                'method'      => 'cmake', },
      'assimp'              => {'url'         => 'https://github.com/assimp/assimp/archive/v4.1.0.tar.gz',
                                'creates'     => 'lib/libassimp.so',
                                'method'      => 'cmake', },
      'libccd'              => {'url'         => 'https://github.com/danfis/libccd/archive/v2.0.tar.gz',
                                'creates'     => 'lib/libccd.so',
                                'args'        => { 'native'   => [ '-DBUILD_SHARED_LIBS=ON', ], },
                                'method'      => 'cmake', },
      'flann'               => {'url'         => 'https://github.com/mariusmuja/flann/archive/1.9.1.tar.gz',
                                'creates'     => 'lib/libflann.so',
                                'args'        => { 'native'   => [ '-DPYTHON_BINDINGS=OFF',
                                                                   '-DMATLAB_BINDINGS=OFF',
                                                                   '-DBUILD_DOC=OFF',
                                                                   '-DBUILD_CUDA_LIB=OFF',
                                                                   '-DBUILD_EXAMPLES=OFF',
                                                                   '-DBUILD_TESTS=OFF', ], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/flann.patch && touch src/cpp/empty.cpp',
                                'method'      => 'cmake', },
      'octomap'             => {'url'         => 'https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz',
                                'args'        => { 'native'   => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF',
                                                                   '-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF', ], },
                                'method'      => 'cmake', },
      'dynamicetd3d'        => {'url'         => 'https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz',
                                'creates'     => 'lib/libdynamicedt3d.so',
                                'args'        => { 'native'   => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF', ], },
                                'require'     => [ Installer['octomap'], ],
                                'method'      => 'cmake', },
      'fcl'                 => {'url'         => 'https://github.com/flexible-collision-library/fcl/archive/0.3.4.tar.gz',
                                'creates'     => 'lib/libfcl.so',
                                'args'        => { 'native'   => [ '-DBUILD_OCTO=OFF', ], },
                                'require'     => [ Installer['octomap'], Installer['libccd'], ],
                                'method'      => 'cmake', },
      'nlopt'               => {'url'         => 'https://github.com/stevengj/nlopt/releases/download/nlopt-2.4.2/nlopt-2.4.2.tar.gz',
                                'creates'     => 'lib/libnlopt_cxx.a',
                                'args'        => { 'native'   => [ '--without-python',
                                                                   '--without-octave',
                                                                   '--without-matlab',
                                                                   '--with-cxx', ], },
                                'method'      => 'autotools', },
      'ipopt'               => {'url'         => 'https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.9.tgz',
                                'creates'     => 'lib/libipopt.so',
                                'args'        => { 'native'   => [ '--with-blas="-lopenblas"',
                                                                   '--with-lapack="-lopenblas"', ], },
                                'method'      => 'autotools', },
      'tinyxml2'            => {'url'         => 'https://github.com/leethomason/tinyxml2/archive/2.2.0.tar.gz',
                                'method'      => 'cmake', },
      'console-bridge'      => {'url'         => 'https://github.com/ros/console_bridge/archive/0.4.0.tar.gz',
                                'creates'     => 'lib/libconsole_bridge.so',
                                'method'      => 'cmake', },
      'urdfdom-headers'     => {'url'         => 'https://github.com/ros/urdfdom_headers/archive/0.4.2.tar.gz',
                                'creates'     => 'lib/pkgconfig/urdfdom_headers.pc',
                                'require'     => [ Installer['console-bridge'], ],
                                'method'      => 'cmake', },
      'urdfdom'             => {'url'         => 'https://github.com/ros/urdfdom/archive/0.4.2.tar.gz',
                                'creates'     => 'lib/pkgconfig/urdfdom.pc',
                                'require'     => [ Installer['urdfdom-headers'], ],
                                'method'      => 'cmake', },
      'bullet3'             => {'url'         => 'https://github.com/bulletphysics/bullet3/archive/2.87.tar.gz',
                                'creates'     => 'lib/libBullet3Common.so',
                                'args'        => { 'native'   => [ '-DBUILD_PYBULLET=OFF ',
                                                                   '-DBUILD_PYBULLET_NUMPY=OFF ',
                                                                   '-DUSE_DOUBLE_PRECISION=ON ',
                                                                   '-D_FIND_LIB_PYTHON_PY=PREFIX/src/bullet3/build3/cmake/FindLibPython.py ',
                                                                   '-DINSTALL_LIBS=ON ',
                                                                   '-DBUILD_SHARED_LIBS=ON', ], },
                                'method'      => 'cmake', },
      'dart'                => {'url'         => 'https://github.com/dartsim/dart/archive/v6.2.0.tar.gz',
                                'creates'     => 'lib/libdart.so',
                                'args'        => { 'native'   => [ '-DDART_BUILD_GUI=OFF ',
                                                                   '-DDART_BUILD_GUI_OSG=OFF', ], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/dart.patch',
                                'require'     => [ Installer['assimp'],
                                                   Installer['bullet3'],
                                                   Installer['fcl'],
                                                   Installer['flann'],
                                                   Installer['libccd'],
                                                   Installer['ipopt'],
                                                   Installer['nlopt'],
                                                   Installer['tinyxml2'],
                                                   Installer['urdfdom'], ],
                                'method'      => 'cmake', },
      'ignition-cmake'      => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-cmake/get/ignition-cmake_0.5.0.tar.bz2',
                                'creates'     => 'lib/cmake/ignition-cmake0/cmake0/IgnCMake.cmake',
                                'method'      => 'cmake', },
      'ignition-math'       => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-math/get/ignition-math4_4.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-math4.so',
                                'require'     => [ Installer['ignition-cmake'], ],
                                'method'      => 'cmake', },
      'ignition-tools'      => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-tools/get/default.tar.bz2',
                                'creates'     => 'lib/pkgconfig/ignition-tools.pc',
                                'require'     => [ Installer['ignition-math'], ],
                                'method'      => 'cmake', },
      'ignition-msgs'       => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-msgs/get/ignition-msgs_1.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-msgs1.so',
                                'require'     => [ Installer['ignition-tools'], ],
                                'method'      => 'cmake', },
      'ignition-transport'  => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-transport/get/ignition-transport4_4.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-transport4.so',
                                'require'     => [ Installer['cppzmq'], Installer['ignition-msgs'], ],
                                'method'      => 'cmake', },
      'ignition-common'     => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-common/get/ignition-common_1.0.1.tar.bz2',
                                'creates'     => 'lib/libignition-common1.so',
                                'args'        => { 'native'   => [ '-DFREEIMAGE_RUNS=1',
                                                                   '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'require'     => [ Installer['ignition-transport'], ],
                                'method'      => 'cmake', },
      'ignition-fuel-tools' => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-fuel-tools/get/ignition-fuel_tools_1.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-fuel_tools1.so',
                                'args'        => { 'native'   => [ '-DFREEIMAGE_RUNS=1',
                                                                   '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'require'     => [ Installer['ignition-common'], ],
                                'method'      => 'cmake', },
      'sdformat'            => {'url'         => 'http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-6.0.0.tar.bz2',
                                'creates'     => 'lib/libsdformat.so',
                                'require'     => [ Installer['ignition-fuel-tools'], ],
                                'method'      => 'cmake', },
      'simbody'             => {'url'         => 'https://github.com/simbody/simbody/archive/Simbody-3.6.tar.gz',
                                'creates'     => 'lib/libSimTKsimbody.so',
                                'args'        => { 'native'   => [ '-DBUILD_EXAMPLES=OFF',
                                                                   '-DBUILD_TESTING=OFF',
                                                                   '-DBUILD_USING_OTHER_LAPACK=PREFIX/lib/libopenblas.so', ], },
                                'require'     => [ Installer['dart'], ],
                                'method'      => 'cmake', },
      'gazebo'              => {'url'         => 'https://bitbucket.org/osrf/gazebo/get/gazebo9_9.0.0.tar.bz2',
                                'creates'     => 'lib/libgazebo.so',
                                'environment' => { 'CFLAGS' => '-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib',
                                                   'CXXFLAGS' => '-L/nubots/toolchain/native/lib -L/nubots/toolchain/lib', },
                                'args'        => { 'native'   => [ '-DFREEIMAGE_RUNS=1',
                                                                   '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/gazebo.patch',
                                'require'     => [ Installer['simbody'], ],
                                'method'      => 'cmake', },
      'gazebo-plugin'       => {'url'         => 'https://github.com/NUbots/Gazebo/archive/master.tar.gz',
                                'creates'     => 'lib/gazebo-9/plugins/libnubotsigus_plugin.so',
                                'src_dir'     => 'plugin',
                                'require'     => [ Installer['gazebo'], ],
                                'method'      => 'cmake', },
    }

    # Download each archive and spawn Installers for each one.
    $archives.each |String $archive,
                    Struct[{'url' => String,
                            Optional['creates'] => String,
                            Optional['args'] => Hash,
                            Optional['require'] => Tuple[Any, 1, default],
                            'method' => String,
                            Optional['src_dir'] => String,
                            Optional['prebuild'] => String,
                            Optional['postbuild'] => String,
                            Optional['environment'] => Hash}] $params| {

          $extension = $params['url'] ? {
            /.*\.zip/       => 'zip',
            /.*\.tgz/       => 'tgz',
            /.*\.tar\.gz/   => 'tar.gz',
            /.*\.txz/       => 'txz',
            /.*\.tar\.xz/   => 'tar.xz',
            /.*\.tbz/       => 'tbz',
            /.*\.tbz2/      => 'tbz2',
            /.*\.tar\.bz2/  => 'tar.bz2',
            /.*\.h/         => 'h',
            /.*\.hpp/       => 'hpp',
            default         => 'UNKNOWN',
          }

          archive { "${archive}":
            url              => $params['url'],
            target           => "/nubots/toolchain/src/${archive}",
            src_target       => "/nubots/toolchain/src",
            purge_target     => true,
            checksum         => false,
            follow_redirects => true,
            timeout          => 0,
            extension        => $extension,
            strip_components => 1,
            root_dir         => '.',
            require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
          }
          installer { "${archive}":
            archs       => $archs,
            creates     => $params['creates'],
            require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['build_tools'], ])),
            args        => $params['args'],
            src_dir     => $params['src_dir'],
            prebuild    => $params['prebuild'],
            postbuild   => $params['postbuild'],
            method      => $params['method'],
            environment => $params['environment'],
            extension   => $extension,
          }
    }

    file { "/nubots/toolchain/native/src/dart.patch":
      content =>
"
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -82,6 +82,7 @@
 # errors.
 option(DART_ENABLE_SIMD
   \"Build DART with all SIMD instructions on the current local machine\" OFF)
+option(DART_BUILD_GUI \"Build gui\" ON)
 option(DART_BUILD_GUI_OSG \"Build osgDart library\" ON)
 option(DART_CODECOV \"Turn on codecov support\" OFF)
 option(DART_TREAT_WARNINGS_AS_ERRORS \"Treat warnings as errors\" OFF)

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
 dart_get_filename_components(header_names \"gui headers\" ${hdrs})
",
      ensure  => present,
      path    => "/nubots/toolchain/native/src/dart.patch",
      before  => [ Installer['dart'], ],
    }

    file { "/nubots/toolchain/native/src/gazebo.patch":
      content =>
"
--- a/gazebo/common/CMakeLists.txt
+++ b/gazebo/common/CMakeLists.txt
@@ -260,6 +260,7 @@
   \${TBB_LIBRARIES}
   \${SDFormat_LIBRARIES}
   \${IGNITION-FUEL_TOOLS_LIBRARIES}
+  \${IGNITION-COMMON_LIBRARIES}
 )

 if (UNIX)
",
      ensure  => present,
      path    => "/nubots/toolchain/native/src/gazebo.patch",
      before  => [ Installer['gazebo'], ],
    }

    file { "/nubots/toolchain/native/src/flann.patch":
      content =>
"
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
",
      ensure  => present,
      path    => "/nubots/toolchain/native/src/flann.patch",
      before  => [ Installer['flann'], ],
    }
}
