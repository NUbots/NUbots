include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node default {

  # We need build tools to compile
  class {'build_tools': }

  # These user tools make the shell much easier
  class {'user_tools':
    user => 'vagrant',
  }

  # Get and install our toolchain
  $toolchain_version = '3.0.6'
  wget::fetch { 'nubots_deb':
    destination => "/root/nubots-toolchain-${toolchain_version}.deb",
    source      => "http://nubots.net/debs/nubots-toolchain-${toolchain_version}.deb",
    timeout     => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure   => 'latest',
    source   => "/root/nubots-toolchain-${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
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
                                      'CCASFLAGS' => '-m64', },
                   },
    'nuc7i7bnh' => {'flags'       => ['-march=broadwell', '-mtune=broadwell', '-mmmx', '-mno-3dnow', '-msse', '-msse2',
                                      '-msse3', '-mssse3', '-mno-sse4a', '-mcx16', '-msahf', '-mmovbe', '-maes',
                                      '-mno-sha', '-mpclmul', '-mpopcnt', '-mabm', '-mno-lwp', '-mfma', '-mno-fma4',
                                      '-mno-xop', '-mbmi', '-mbmi2', '-mno-tbm', '-mavx', '-mavx2', '-msse4.2',
                                      '-msse4.1', '-mlzcnt', '-mno-rtm', '-mno-hle', '-mrdrnd', '-mf16c', '-mfsgsbase',
                                      '-mrdseed', '-mprfchw', '-madx', '-mfxsr', '-mxsave', '-mxsaveopt',
                                      '-mno-avx512f', '-mno-avx512er', '-mno-avx512cd', '-mno-avx512pf',
                                      '-mno-prefetchwt1', '-mclflushopt', '-mxsavec', '-mxsaves', '-mno-avx512dq',
                                      '-mno-avx512bw', '-mno-avx512vl', '-mno-avx512ifma', '-mno-avx512vbmi',
                                      '-mno-clwb', '-mno-mwaitx', ],
                    'params'      => ['-m64', '--param l1-cache-size=32', '--param l1-cache-line-size=64',
                                      '--param l2-cache-size=4096', ],
                    'environment' => {'TARGET' => 'HASWELL',
                                      'USE_THREAD' => '1',
                                      'BINARY' => '64',
                                      'NUM_THREADS' => '2',
                                      'AUDIO' => 'PORTAUDIO',
                                      'LDFLAGS' => '-m64',
                                      'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig',
                                      'CCAS' => '/usr/bin/gcc',
                                      'AS' => '/usr/bin/gcc',
                                      'CCASFLAGS' => '-m64', },
                   },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # These user tools make the shell much easier and these also should be done before installing
  class {'user_tools':
    user => 'vagrant',
  } -> Installer <| |>

  # Install protobuf.
  class { 'protobuf': }

  # Install quex.
  class { 'quex': }

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    # We need to match the protobuf version with the one we install in the python class.
    'protobuf'              => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.5.0/protobuf-cpp-3.5.0.tar.gz',
                                'args'        => { 'native'    => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                                   'nuc7i7bnh' => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ], },
                                'require'     => [ Class['protobuf'], Installer['zlib'], ],
                                'prebuild'    => 'make distclean',
                                'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                                'method'      => 'autotools', },
    'zlib'                  => {'url'         => 'http://www.zlib.net/zlib-1.2.11.tar.gz',
                                'creates'     => 'lib/libz.a',
                                'method'      => 'cmake', },
    'bzip2'                 => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.1.tar.gz',
                                'creates'     => 'lib/libbz2.so',
                                'method'      => 'make', },
    'xml2'                  => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                                'args'        => { 'native'    => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                                   'nuc7i7bnh' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                                'method'      => 'autotools', },
    'nuclear'               => {'url'         => 'https://github.com/Fastcode/NUClear/archive/master.tar.gz',
                                'args'        => { 'native'    => [ '-DBUILD_TESTS=OFF', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_TESTS=OFF', ], },
                                'method'      => 'cmake', },
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'              => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.19.tar.gz',
                                'args'        => { 'native'    => [ '', ],
                                                   'nuc7i7bnh' => [ 'CROSS=1', ], },
                                'method'      => 'make',
                                'creates'     => 'lib/libopenblas.a', },
    'libsvm'                => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                                'creates'     => 'lib/svm.o',
                                'method'      => 'make', },
    'armadillo'             => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.950.1.tar.xz',
                                'method'      => 'cmake',
                                'creates'     => 'lib/libarmadillo.so',
                                'require'     => [ Installer['openblas'], ], },
    'tcmalloc'              => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5.93/gperftools-2.5.93.tar.gz',
                                'args'        => { 'native'    => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                                   'nuc7i7bnh' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                                'creates'     => 'lib/libtcmalloc_minimal.a',
                                'method'      => 'autotools', },
    'yaml-cpp'              => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz',
                                'args'        => { 'native'    => [ '-DYAML_CPP_BUILD_CONTRIB=OFF',
                                                                    '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                                   'nuc7i7bnh' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF',
                                                                    '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                                'method'      => 'cmake', },
    'fftw3'                 => {'url'         => 'http://www.fftw.org/fftw-3.3.6-pl2.tar.gz',
                                'args'        => { 'native'    => [ '--disable-fortran', '--enable-shared', ],
                                                   'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', ], },
                                'method'      => 'autotools', },
    'jpeg'                  => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
                                'args'        => { 'native'    => [ 'CCASFLAGS="-f elf64"', ],
                                                   'nuc7i7bnh' => [ 'CCASFLAGS="-f elf64"', ], },
                                'method'      => 'autotools', },
    'cppformat'             => {'url'         => 'https://github.com/fmtlib/fmt/archive/3.0.1.tar.gz',
                                'method'      => 'cmake',
                                'creates'     => 'lib/libfmt.a', },
    'portaudio'             => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                                'method'      => 'autotools', },
    'eigen3'                => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2',
                                'creates'     => 'include/eigen3/Eigen/Eigen',
                                'method'      => 'cmake', },
    'boost'                 => {'url'         => 'https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz',
                                'args'        => { 'native'    => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                                   'nuc7i7bnh' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                                'method'      => 'boost',
                                'creates'     => 'src/boost/build_complete',
                                'postbuild'   => 'touch build_complete',
                                'require'     => [ Installer['zlib'], Installer['bzip2'], ], },
    'espeak'                => {'url'         => 'https://github.com/Bidski/espeak/archive/v1.48.04.tar.gz',
                                'src_dir'     => 'src',
                                'prebuild'    => 'cp portaudio19.h portaudio.h',
                                'method'      => 'make',
                                'require'     => [ Installer['portaudio'], ], },
    'fswatch'               => {'url'         => 'https://github.com/emcrisostomo/fswatch/releases/download/1.9.3/fswatch-1.9.3.tar.gz',
                                'method'      => 'autotools', },
    'ffi'                   => {'url'         => 'https://github.com/libffi/libffi/archive/v3.2.1.tar.gz',
                                'postbuild'   => 'if [ -e PREFIX/lib32/libffi.a ]; then cp PREFIX/lib32/libffi* PREFIX/lib/; fi',
                                'method'      => 'autotools', },
    'util-linux'            => {'url'         => 'https://www.kernel.org/pub/linux/utils/util-linux/v2.31/util-linux-2.31.tar.xz',
                                'args'        => { 'native'    => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ],
                                                   'nuc7i7bnh' => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ], },
                                'creates'     => 'lib/libmount.so',
                                 'method'     => 'autotools', },
    'glib'                  => {'url'         => 'ftp://ftp.gnome.org/pub/gnome/sources/glib/2.52/glib-2.52.3.tar.xz',
                                'args'        => { 'native'    => [ '--cache-file=PREFIX/src/glib.config', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ],
                                                   # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                                   'nuc7i7bnh' => [ '--cache-file=PREFIX/src/glib.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ], },
                                'postbuild'   => 'cp glib/glibconfig.h PREFIX/include/glibconfig.h',
                                'require'     => [ Installer['ffi'], Installer['util-linux'], ],
                                'creates'     => 'lib/libglib-2.0.so',
                                'method'      => 'autotools', },
    'aravis'                => {'url'         => 'https://github.com/AravisProject/aravis/archive/ARAVIS_0_5_9.tar.gz',
                                'args'        => { 'native'    => [ '--cache-file=PREFIX/src/aravis.config', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ],
                                                   # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                                   'nuc7i7bnh' => [ '--cache-file=PREFIX/src/aravis.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ], },
                                'require'     => [ Installer['xml2'], Installer['zlib'], Installer['glib'], ],
                                'creates'     => 'lib/libaravis-0.6.so',
                                'prebuild'    => 'sed "s/return\s(entry->schema\s>>\s10)\s\&\s0x0000001f;/return ((entry->schema >> 10) \& 0x0000001f) ? ARV_UVCP_SCHEMA_ZIP : ARV_UVCP_SCHEMA_RAW;/" -i src/arvuvcp.h',
                                'postbuild'   => 'cp src/arvconfig.h PREFIX/include/arvconfig.h',
                                'method'      => 'autotools', },
    'zeromq'                => {'url'         => 'https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz',
                                'creates'     => 'lib/libzmq.a',
                                'method'      => 'autotools', },
    'czmq'                  => {'url'         => 'https://github.com/zeromq/czmq/releases/download/v4.1.1/czmq-4.1.1.tar.gz',
                                'require'     => [ Installer['zeromq'], ],
                                'method'      => 'autotools', },
    'cppzmq'                => {'url'         => 'https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz',
                                'creates'     => 'include/zmq.hpp',
                                'require'     => [ Installer['czmq'], ],
                                'method'      => 'cmake', },
    'assimp'                => {'url'         => 'https://github.com/assimp/assimp/archive/v4.1.0.tar.gz',
                                'creates'     => 'lib/libassimp.so',
                                'require'     => [ Installer['zlib'], ],
                                'method'      => 'cmake', },
    'libccd'                => {'url'         => 'https://github.com/danfis/libccd/archive/v2.0.tar.gz',
                                'creates'     => 'lib/libccd.so',
                                'args'        => { 'native'   => [ '-DBUILD_SHARED_LIBS=ON', ], },
                                'method'      => 'cmake', },
    'flann'                 => {'url'         => 'https://github.com/mariusmuja/flann/archive/1.9.1.tar.gz',
                                'creates'     => 'lib/libflann.so',
                                'args'        => { 'native'    => [ '-DPYTHON_BINDINGS=OFF',
                                                                    '-DMATLAB_BINDINGS=OFF',
                                                                    '-DBUILD_DOC=OFF',
                                                                    '-DBUILD_CUDA_LIB=OFF',
                                                                    '-DBUILD_EXAMPLES=OFF',
                                                                    '-DBUILD_TESTS=OFF', ],
                                                   'nuc7i7bnh' => [ '-DPYTHON_BINDINGS=OFF',
                                                                    '-DMATLAB_BINDINGS=OFF',
                                                                    '-DBUILD_DOC=OFF',
                                                                    '-DBUILD_CUDA_LIB=OFF',
                                                                    '-DBUILD_EXAMPLES=OFF',
                                                                    '-DBUILD_TESTS=OFF', ], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/flann.patch && touch src/cpp/empty.cpp',
                                'method'      => 'cmake', },
    'octomap'               => {'url'         => 'https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz',
                                'args'        => { 'native'    => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF',
                                                                    '-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF',
                                                                    '-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF', ], },
                                'method'      => 'cmake', },
    'dynamicetd3d'          => {'url'         => 'https://github.com/OctoMap/octomap/archive/v1.7.2.tar.gz',
                                'creates'     => 'lib/libdynamicedt3d.so',
                                'args'        => { 'native'    => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_OCTOVIS_SUBPROJECT=OFF', ], },
                                'require'     => [ Installer['octomap'], ],
                                'method'      => 'cmake', },
    'fcl'                   => {'url'         => 'https://github.com/flexible-collision-library/fcl/archive/0.3.4.tar.gz',
                                'creates'     => 'lib/libfcl.so',
                                'args'        => { 'native'    => [ '-DBUILD_OCTO=OFF', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_OCTO=OFF', ], },
                                'require'     => [ Installer['octomap'], Installer['libccd'], ],
                                'method'      => 'cmake', },
    'nlopt'                 => {'url'         => 'https://github.com/stevengj/nlopt/releases/download/nlopt-2.4.2/nlopt-2.4.2.tar.gz',
                                'creates'     => 'lib/libnlopt_cxx.a',
                                'args'        => { 'native'    => [ '--without-python',
                                                                    '--without-octave',
                                                                    '--without-matlab',
                                                                    '--with-cxx', ],
                                                   'nuc7i7bnh' => [ '--without-python',
                                                                    '--without-octave',
                                                                    '--without-matlab',
                                                                    '--with-cxx', ], },
                                'method'      => 'autotools', },
    'ipopt'                 => {'url'         => 'https://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.9.tgz',
                                'creates'     => 'lib/libipopt.so',
                                'args'        => { 'native'    => [ '--with-blas="-lopenblas"',
                                                                    '--with-lapack="-lopenblas"', ],
                                                   'nuc7i7bnh' => [ '--with-blas="-lopenblas"',
                                                                    '--with-lapack="-lopenblas"', ], },
                                'require'     => [ Installer['openblas'], ],
                                'method'      => 'autotools', },
    'tinyxml2'              => {'url'         => 'https://github.com/leethomason/tinyxml2/archive/2.2.0.tar.gz',
                                'method'      => 'cmake', },
    'console-bridge'        => {'url'         => 'https://github.com/ros/console_bridge/archive/0.4.0.tar.gz',
                                'creates'     => 'lib/libconsole_bridge.so',
                                'method'      => 'cmake', },
    'urdfdom-headers'       => {'url'         => 'https://github.com/ros/urdfdom_headers/archive/0.4.2.tar.gz',
                                'creates'     => 'lib/pkgconfig/urdfdom_headers.pc',
                                'require'     => [ Installer['console-bridge'], ],
                                'method'      => 'cmake', },
    'urdfdom'               => {'url'         => 'https://github.com/ros/urdfdom/archive/0.4.2.tar.gz',
                                'creates'     => 'lib/pkgconfig/urdfdom.pc',
                                'require'     => [ Installer['urdfdom-headers'], ],
                                'method'      => 'cmake', },
    'bullet3'               => {'url'         => 'https://github.com/bulletphysics/bullet3/archive/2.87.tar.gz',
                                'creates'     => 'lib/libBullet3Common.so',
                                'args'        => { 'native'    => [ '-DBUILD_PYBULLET=OFF ',
                                                                    '-DBUILD_PYBULLET_NUMPY=OFF ',
                                                                    '-DUSE_DOUBLE_PRECISION=ON ',
                                                                    '-D_FIND_LIB_PYTHON_PY=PREFIX/src/bullet3/build3/cmake/FindLibPython.py ',
                                                                    '-DINSTALL_LIBS=ON ',
                                                                    '-DBUILD_SHARED_LIBS=ON', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_PYBULLET=OFF ',
                                                                    '-DBUILD_PYBULLET_NUMPY=OFF ',
                                                                    '-DUSE_DOUBLE_PRECISION=ON ',
                                                                    '-D_FIND_LIB_PYTHON_PY=PREFIX/src/bullet3/build3/cmake/FindLibPython.py ',
                                                                    '-DINSTALL_LIBS=ON ',
                                                                    '-DBUILD_SHARED_LIBS=ON', ], },
                                'method'      => 'cmake', },
    'dart'                  => {'url'         => 'https://github.com/dartsim/dart/archive/v6.2.0.tar.gz',
                                'creates'     => 'lib/libdart.so',
                                'args'        => { 'native'    => [ '-DDART_BUILD_GUI=OFF ',
                                                                    '-DDART_BUILD_GUI_OSG=OFF', ],
                                                   'nuc7i7bnh' => [ '-DDART_BUILD_GUI=OFF ',
                                                                    '-DDART_BUILD_GUI_OSG=OFF', ], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/dart.patch',
                                'require'     => [ Installer['eigen3'],
                                                   Installer['boost'],
                                                   Installer['assimp'],
                                                   Installer['bullet3'],
                                                   Installer['fcl'],
                                                   Installer['flann'],
                                                   Installer['libccd'],
                                                   Installer['ipopt'],
                                                   Installer['nlopt'],
                                                   Installer['tinyxml2'],
                                                   Installer['urdfdom'], ],
                                'method'      => 'cmake', },
    'ignition-cmake'        => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-cmake/get/ignition-cmake_0.5.0.tar.bz2',
                                'creates'     => 'lib/cmake/ignition-cmake0/cmake0/IgnCMake.cmake',
                                'require'     => [ Installer['protobuf'],
                                                   Installer['cppzmq'],
                                                   Installer['tinyxml2'],
                                                   Installer['util-linux'],
                                                   Installer['yaml-cpp'], ],
                                'method'      => 'cmake', },
    'ignition-math'         => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-math/get/ignition-math4_4.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-math4.so',
                                'require'     => [ Installer['ignition-cmake'], ],
                                'method'      => 'cmake', },
    'ignition-tools'        => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-tools/get/default.tar.bz2',
                                'creates'     => 'lib/pkgconfig/ignition-tools.pc',
                                'require'     => [ Installer['ignition-math'], ],
                                'method'      => 'cmake', },
    'ignition-msgs'         => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-msgs/get/ignition-msgs_1.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-msgs1.so',
                                'require'     => [ Installer['protobuf'], Installer['ignition-tools'], ],
                                'method'      => 'cmake', },
    'ignition-transport'    => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-transport/get/ignition-transport4_4.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-transport4.so',
                                'require'     => [ Installer['protobuf'],
                                                   Installer['util-linux'],
                                                   Installer['cppzmq'],
                                                   Installer['ignition-msgs'], ],
                                'method'      => 'cmake', },
    'ignition-common'       => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-common/get/ignition-common_1.0.1.tar.bz2',
                                'creates'     => 'lib/libignition-common1.so',
                                'args'        => { 'native'    => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'],
                                                   'nuc7i7bnh' => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'require'     => [ Installer['util-linux'], Installer['ignition-transport'], ],
                                'method'      => 'cmake', },
    'ignition-fuel-tools'   => {'url'         => 'https://bitbucket.org/ignitionrobotics/ign-fuel-tools/get/ignition-fuel_tools_1.0.0.tar.bz2',
                                'creates'     => 'lib/libignition-fuel_tools1.so',
                                'args'        => { 'native'    => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'],
                                                   'nuc7i7bnh' => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'require'     => [ Installer['ignition-common'], ],
                                'method'      => 'cmake', },
    'sdformat'              => {'url'         => 'http://osrf-distributions.s3.amazonaws.com/sdformat/releases/sdformat-6.0.0.tar.bz2',
                                'creates'     => 'lib/libsdformat.so',
                                'require'     => [ Installer['ignition-fuel-tools'], ],
                                'method'      => 'cmake', },
    'simbody'               => {'url'         => 'https://github.com/simbody/simbody/archive/Simbody-3.6.tar.gz',
                                'creates'     => 'lib/libSimTKsimbody.so',
                                'args'        => { 'native'    => [ '-DBUILD_EXAMPLES=OFF',
                                                                    '-DBUILD_TESTING=OFF',
                                                                    '-DBUILD_USING_OTHER_LAPACK=PREFIX/lib/libopenblas.so', ],
                                                   'nuc7i7bnh' => [ '-DBUILD_EXAMPLES=OFF',
                                                                    '-DBUILD_TESTING=OFF',
                                                                    '-DBUILD_USING_OTHER_LAPACK=PREFIX/lib/libopenblas.so', ], },
                                'require'     => [ Installer['openblas'], Installer['dart'], ],
                                'method'      => 'cmake', },
    'gazebo'                => {'url'         => 'https://bitbucket.org/osrf/gazebo/get/gazebo9_9.0.0.tar.bz2',
                                'creates'     => 'lib/libgazebo.so',
                                'environment' => { 'CFLAGS'    => '-LPREFIX/lib -LPREFIX/../lib',
                                                   'CXXFLAGS'  => '-LPREFIX/lib -LPREFIX/../lib', },
                                'args'        => { 'native'    => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'],
                                                   'nuc7i7bnh' => [ '-DFREEIMAGE_RUNS=1',
                                                                    '-DFREEIMAGE_RUNS__TRYRUN_OUTPUT=1'], },
                                'prebuild'    => 'patch -Np1 -i PREFIX/src/gazebo.patch',
                                'require'     => [ Installer['protobuf'], Installer['simbody'], ],
                                'method'      => 'cmake', },
    'gazebo-plugin'         => {'url'         => 'https://github.com/NUbots/Gazebo/archive/master.tar.gz',
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
          # Gazebo depends on some of our installed libraries
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['build_tools'], Class['protobuf'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          environment => $params['environment'],
          extension   => $extension,
        }
  }

  # Install catch.
  installer { 'catch':
    url       => 'https://github.com/catchorg/Catch2/releases/download/v2.9.1/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
    method    => 'wget',
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    # Update the armadillo config header file for all archs.
    file { "armadillo_${arch}_config":
      path    => "/nubots/toolchain/${arch}/include/armadillo_bits/config.hpp",
      source  => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
      ensure  => present,
      require => [ Installer['armadillo'], ],
    }
  }

  # After we have installed build our deb.
  Installer <| |> ~> class { 'toolchain_deb': }

  # Patch some system utilities to make sure they ignore our preset LD_LIBRARY_PATH
  file { "/nubots/toolchain/bin/msgfmt.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/msgfmt \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/msgfmt.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/msgmerge.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/msgmerge \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/msgmerge.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/xgettext.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/xgettext \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/xgettext.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/pkg-config.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/pkg-config \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/pkg-config.sh",
    mode    => "a+x",
  } -> Installer <| |>

  $archs.each |String $arch, Hash $params| {
    $prefix = '/nubots/toolchain'

    # We need to prevent glib from trying to run tests when cross-compiling glib (to avoid SIGILL).
    file { "${arch}_glib.config":
      content =>
"glib_cv_stack_grows=no
glib_cv_uscore=no
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/glib.config",
      mode    => "a-w",
      before  => [ Installer['glib'], ],
    }

    # Force paths to gettext bianries (to avoid SIGILL).
    file { "${arch}_aravis.config":
      content =>
"ac_cv_path_XGETTEXT=${prefix}/bin/xgettext.sh
ac_cv_path_MSGMERGE=${prefix}/bin/msgmerge.sh
ac_cv_path_MSGFMT=${prefix}/bin/msgfmt.sh
ac_cv_path_PKG_CONFIG=${prefix}/bin/pkg-config.sh
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/aravis.config",
      mode    => "a-w",
      before  => [ Installer['aravis'], ],
    }

    file { "${prefix}/${arch}/src/dart.patch":
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
 dart_get_filename_components(header_names \"gui headers\" ${hdrs}))
",
    ensure  => present,
    path    => "${prefix}/${arch}/src/dart.patch",
    before  => [ Installer['dart'], ],
  }

  file { "${prefix}/${arch}/src/gazebo.patch":
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
    path    => "${prefix}/${arch}/src/gazebo.patch",
    before  => [ Installer['gazebo'], ],
  }

  file { "${prefix}/${arch}/src/flann.patch":
    content =>
"
--- a/src/cpp/CMakeLists.txt
+++ b/src/cpp/CMakeLists.txt
@@ -29,12 +29,12 @@
 endif()

 if(CMAKE_SYSTEM_NAME STREQUAL \"Linux\" AND CMAKE_COMPILER_IS_GNUCC)
-    add_library(flann_cpp SHARED \"\")
+    add_library(flann_cpp SHARED \"empty.cpp\")
     set_target_properties(flann_cpp PROPERTIES LINKER_LANGUAGE CXX)
     target_link_libraries(flann_cpp -Wl,-whole-archive flann_cpp_s -Wl,-no-whole-archive)

     if (BUILD_CUDA_LIB)
-	    cuda_add_library(flann_cuda SHARED \"\")
+	    cuda_add_library(flann_cuda SHARED \"empty.cpp\")
         set_target_properties(flann_cuda PROPERTIES LINKER_LANGUAGE CXX)
         target_link_libraries(flann_cuda -Wl,-whole-archive flann_cuda_s -Wl,-no-whole-archive)
         set_property(TARGET flann_cpp_s PROPERTY COMPILE_DEFINITIONS FLANN_USE_CUDA)
@@ -83,7 +83,7 @@
     set_property(TARGET flann_s PROPERTY COMPILE_DEFINITIONS FLANN_STATIC)

     if(CMAKE_SYSTEM_NAME STREQUAL \"Linux\" AND CMAKE_COMPILER_IS_GNUCC)
-        add_library(flann SHARED \"\")
+        add_library(flann SHARED \"empty.cpp\")
         set_target_properties(flann PROPERTIES LINKER_LANGUAGE CXX)
         target_link_libraries(flann -Wl,-whole-archive flann_s -Wl,-no-whole-archive)
     else()
",
    ensure  => present,
    path    => "${prefix}/${arch}/src/flann.patch",
    before  => [ Installer['flann'], ],
  }

    # Create CMake toolchain files.
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(OpenCL_INCLUDE_DIR \"/opt/intel/opencl/include\")
set(OpenCL_LIBRARY \"/opt/intel/opencl/libOpenCL.so\")

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\")
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\")

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)

# Set cmake up to look in our pkgconfig directories
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON CACHE STRING \"\")
set(CMAKE_PREFIX_PATH \"${prefix}/${arch};${prefix}/${arch}/lib/pkgconfig\" CACHE STRING \"\")
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }
  }
}
