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
  $toolchain_version = '3.0.0'
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
                    'environment' => {'TARGET' => 'GENERIC', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', 'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig', 'CCAS' => 'gcc', 'CCASFLAGS' => '-m64', },
                   },
    'nuc7i7bnh' => {'flags'       => ['-m64', '-march=broadwell', '-mtune=broadwell', '-mmmx', '-mno-3dnow', '-msse', '-msse2', '-msse3', '-mssse3', '-mno-sse4a', '-mcx16', '-msahf', '-mmovbe', '-maes', '-mno-sha', '-mpclmul', '-mpopcnt', '-mabm', '-mno-lwp', '-mfma', '-mno-fma4', '-mno-xop', '-mbmi', '-mbmi2', '-mno-tbm', '-mavx', '-mavx2', '-msse4.2', '-msse4.1', '-mlzcnt', '-mno-rtm', '-mno-hle', '-mrdrnd', '-mf16c', '-mfsgsbase', '-mrdseed', '-mprfchw', '-madx', '-mfxsr', '-mxsave', '-mxsaveopt', '-mno-avx512f', '-mno-avx512er', '-mno-avx512cd', '-mno-avx512pf', '-mno-prefetchwt1', '-mclflushopt', '-mxsavec', '-mxsaves', '-mno-avx512dq', '-mno-avx512bw', '-mno-avx512vl', '-mno-avx512ifma', '-mno-avx512vbmi', '-mno-clwb', '-mno-mwaitx', ],
                    'params'      => ['-m64', '--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                    'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', 'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig', 'CCAS' => 'gcc', 'CCASFLAGS' => '-m64', },
                   },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need build tools to compile and we need it done before the installer
  class {'build_tools': } -> Installer <| |>

  # These user tools make the shell much easier and these also should be done before installing
  class {'user_tools':
    user => 'vagrant',
  } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.5.0/protobuf-cpp-3.5.0.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'nuc7i7bnh' => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'prebuild'    => 'make distclean',
                       'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                       'method'      => 'autotools', },
    'zlib'         => {'url'         => 'http://www.zlib.net/zlib-1.2.11.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake',},
    'bzip2'        => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.1.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'creates'     => 'lib/libbz2.so',
                       'method'      => 'make',},
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'nuc7i7bnh' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools',},
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake',},
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.19.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ 'CROSS=1', ], },
                       'method'      => 'make',
                       'creates'     => 'lib/libopenblas.a' },
    'libsvm'       => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                       'creates'     => 'lib/svm.o',
                       'method'      => 'make', },
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.950.1.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ],},
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5.93/gperftools-2.5.93.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'nuc7i7bnh' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools',},
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'nuc7i7bnh' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake',},
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.7.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', '--enable-openmp', '--enable-threads', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', '--enable-openmp', '--enable-threads', ], },
                       'method'      => 'autotools',},
    'fftw3f'       => {'url'         => 'http://www.fftw.org/fftw-3.3.7.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', '--enable-float', '--enable-openmp', '--enable-threads', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', '--enable-float', '--enable-openmp', '--enable-threads', ], },
                       'method'      => 'autotools',},
    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
                       'args'        => { 'native'   => [ 'CCASFLAGS="-f elf64"', ],
                                          'nuc7i7bnh' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools',},
    'cppformat'    => {'url'         => 'https://github.com/fmtlib/fmt/archive/3.0.1.tar.gz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libfmt.a' },
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'method'      => 'autotools',},
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',
                       'require'     => [ Installer['fftw3'], Installer['fftw3f'], ],},
    'boost'        => {'url'         => 'https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz',
                       'args'        => { 'native'   => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'nuc7i7bnh' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2'], ],},
    'espeak'       => {'url'         => 'https://github.com/Bidski/espeak/archive/v1.48.04.tar.gz',
                       'src_dir'     => 'src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ Installer['portaudio'], ],},
    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/releases/download/1.9.3/fswatch-1.9.3.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'method'      => 'autotools', },
    'ffi'          => {'url'         => 'https://github.com/libffi/libffi/archive/v3.2.1.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ '', ], },
                       'postbuild'   => 'if [ -e PREFIX/lib32/libffi.a ]; then cp PREFIX/lib32/libffi* PREFIX/lib/; fi',
                       'method'      => 'autotools', },
    'glib'         => {'url'         => 'ftp://ftp.gnome.org/pub/gnome/sources/glib/2.52/glib-2.52.3.tar.xz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/glib.config', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ],
                                          # Technically we are cross compiling for the nuv7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/glib.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ], },
                       'postbuild'   => 'cp glib/glibconfig.h PREFIX/include/glibconfig.h',
                       'require'     => [ Installer['ffi'], ],
                       'creates'     => 'lib/libglib-2.0.so',
                       'method'      => 'autotools', },
    'aravis'       => {'url'         => 'https://github.com/AravisProject/aravis/archive/ARAVIS_0_5_9.tar.gz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/aravis.config', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ],
                                          # Technically we are cross compiling for the nuv7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/aravis.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ], },
                       'require'     => [ Installer['xml2'], Installer['zlib'], Installer['glib'], ],
                       'creates'     => 'lib/libaravis-0.6.so',
                       'prebuild'    => 'sed "s/return\s(entry->schema\s>>\s10)\s\&\s0x0000001f;/return ((entry->schema >> 10) \& 0x0000001f) ? ARV_UVCP_SCHEMA_ZIP : ARV_UVCP_SCHEMA_RAW;/" -i src/arvuvcp.h',
                       'postbuild'   => 'cp src/arvconfig.h PREFIX/include/arvconfig.h',
                       'method'      => 'autotools', },
    'pybind11'     => {'url'         => 'https://github.com/pybind/pybind11/archive/v2.2.1.tar.gz',
                       'args'        => { 'native'   => [ '-DPYBIND11_TEST=OFF', ' -DPYBIND11_PYTHON_VERSION=3',  ],
                                          'nuc7i7bnh' => [ '-DPYBIND11_TEST=OFF', ' -DPYBIND11_PYTHON_VERSION=3', ], },
                       'creates'     => 'include/pybind11/pybind11.h',
                       'method'      => 'cmake', },

    # Mesa
    # http://www.linuxfromscratch.org/blfs/view/svn/x/mesa.html
    # 'pybeaker'     => {'url'         => 'https://files.pythonhosted.org/packages/source/B/Beaker/Beaker-1.9.0.tar.gz',
    #                    'args'        => { 'native'   => [ ' --optimize=1',  ],
    #                                       'nuc7i7bnh' => [ ' --optimize=1', ], },
    #                    'method'      => 'python', },
    # 'pymarkupsafe' => {'url'         => 'https://files.pythonhosted.org/packages/source/M/MarkupSafe/MarkupSafe-1.0.tar.gz',
    #                    'args'        => { 'native'   => [ ' --optimize=1',  ],
    #                                       'nuc7i7bnh' => [ ' --optimize=1', ], },
    #                    'method'      => 'python', },
    # 'pymako'       => {'url'         => 'https://files.pythonhosted.org/packages/source/M/Mako/Mako-1.0.4.tar.gz',
    #                    'args'        => { 'native'   => [ ' --optimize=1',  ],
    #                                       'nuc7i7bnh' => [ ' --optimize=1', ], },
    #                    'require'     => [ Installer['pybeaker'], Installer['pymarkupsafe'], ],
    #                    'method'      => 'python', },

    'freetype'     => {'url'         => 'https://downloads.sourceforge.net/freetype/freetype-2.8.1.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-static', ],
                                          'nuc7i7bnh' => [ '--disable-static', ], },
                       'prebuild'    => 'sed -ri "s:.*(AUX_MODULES.*valid):\1:" PREFIX/src/freetype/modules.cfg &&
                                         sed -r "s:.*(#.*SUBPIXEL_RENDERING) .*:\1:" -i PREFIX/src/freetype/include/freetype/config/ftoption.h',
                       'creates'     => 'lib/libfreetype.so',
                       'method'      => 'autotools', },
    'fontconfig'   => {'url'         => 'https://www.freedesktop.org/software/fontconfig/release/fontconfig-2.12.6.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-docs', ],
                                          'nuc7i7bnh' => [ '--disable-docs', ], },
                       'prebuild'    => 'rm -f src/fcobjshash.h',
                       'require'     => [ Installer['freetype'], ],
                       'creates'     => 'lib/libfontconfig.so',
                       'method'      => 'autotools', },
    'util-macros'  => {'url'         => 'https://www.x.org/pub/individual/util/util-macros-1.19.1.tar.bz2',
                       'creates'     => 'share/aclocal/xorg-macros.m4',
                       'method'      => 'autotools', },
    'xcb-proto'    => {'url'         => 'https://xcb.freedesktop.org/dist/xcb-proto-1.12.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-static',  ],
                                          'nuc7i7bnh' => [ '--disable-static',  ], },
                       'prebuild'    => 'wget http://www.linuxfromscratch.org/patches/blfs/svn/xcb-proto-1.12-schema-1.patch -O - | patch -Np1 &&
                                         wget http://www.linuxfromscratch.org/patches/blfs/svn/xcb-proto-1.12-python3-1.patch -O - | patch -Np1',
                       'creates'     => 'lib/python3.5/site-packages/xcbgen/__init__.py',
                       'method'      => 'autotools', },
    'xorg-protocol-headers' => {'url' => '',
                                'prebuild'    => 'PREFIX/bin/xorg-protos.sh',
                                'require'     => [ Installer['util-macros'], ],
                                'creates'     => 'share/doc/xproto/x11protocol.xml',
                                'method'      => 'custom', },
    'Xdmcp'        => {'url'         => 'https://www.x.org/pub/individual/lib/libXdmcp-1.1.2.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-static',  ],
                                          'nuc7i7bnh' => [ '--disable-static',  ], },
                       'creates'     => 'lib/libXdmcp.so',
                       'require'     => [ Installer['xorg-protocol-headers'], ],
                       'method'      => 'autotools', },
    'Xau'          => {'url'         => 'https://www.x.org/pub/individual/lib/libXau-1.0.8.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-static',  ],
                                          'nuc7i7bnh' => [ '--disable-static',  ], },
                       'creates'     => 'lib/libXau.so',
                       'require'     => [ Installer['xorg-protocol-headers'], ],
                       'method'      => 'autotools', },
    'xcb'          => {'url'         => 'https://xcb.freedesktop.org/dist/libxcb-1.12.tar.bz2',
                       'args'        => { 'native'   => [ '--disable-static', '--enable-xinput', '--without-doxygen', ],
                                          'nuc7i7bnh' => [ '--disable-static', '--enable-xinput', '--without-doxygen', ], },
                       'prebuild'    => 'wget http://www.linuxfromscratch.org/patches/blfs/svn/libxcb-1.12-python3-1.patch -O - | patch -Np1 &&
                                         sed -i "s/pthread-stubs//" configure',
                       'creates'     => 'lib/libxcb.so',
                       'require'     => [ Installer['Xau'], Installer['xcb-proto'], ],
                       'method'      => 'autotools', },
    'xorg-libs'    => {'url' => '',
                       'prebuild'    => 'PREFIX/bin/xorg-libs.sh',
                       'creates'     => 'lib/libxshmfence.so',
                       'require'     => [ Installer['fontconfig'], Installer['xcb'], ],
                       'method'      => 'custom', },
    'drm'          => {'url'         => 'https://dri.freedesktop.org/libdrm/libdrm-2.4.85.tar.bz2',
                       'args'        => { 'native'   => [ '--enable-udev', ],
                                          'nuc7i7bnh' => [ '--enable-udev', ], },
                       'creates'     => 'lib/libdrm.so',
                       'require'     => [ Installer['xorg-libs'], ],
                       'method'      => 'autotools', },
    'mesa'         => {'url'         => 'https://mesa.freedesktop.org/archive/mesa-17.2.3.tar.xz',
                       'args'        => { 'native'   => [ '--enable-texture-float', '--enable-osmesa', '--enable-xa', '--enable-glx-tls', '--with-platforms="drm,x11"', '--enable-gles1', '--enable-gles2', '--enable-shared-glapi', '--enable-egl', '--with-dri-drivers="i965,i915"', '--with-gallium-drivers="swrast,svga"', '--with-vulkan-drivers=intel', '--enable-gbm', ],
                                          'nuc7i7bnh' => [ '--enable-texture-float', '--enable-osmesa', '--enable-xa', '--enable-glx-tls', '--with-platforms="drm,x11"', '--enable-gles1', '--enable-gles2', '--enable-shared-glapi', '--enable-egl', '--with-dri-drivers="i965,i915"', '--with-gallium-drivers="swrast,svga"', '--with-vulkan-drivers=intel', '--enable-gbm', ], },
                       'creates'     => 'lib/libEGL.so',
                       'require'     => [ Installer['xorg-libs'], Installer['drm'], ], #Installer['pymako'], ],
                       'method'      => 'autotools', },
    'glm'          => {'url'         => 'https://github.com/g-truc/glm/archive/0.9.8.5.tar.gz',
                       'args'        => { 'native'   => [ '-DGLM_TEST_ENABLE_CXX_14=ON', '-DGLM_TEST_ENABLE_LANG_EXTENSIONS=ON', '-DGLM_TEST_ENABLE_FAST_MATH=ON',  ],
                                          'nuc7i7bnh' => [ '-DGLM_TEST_ENABLE_CXX_14=ON', '-DGLM_TEST_ENABLE_LANG_EXTENSIONS=ON', '-DGLM_TEST_ENABLE_FAST_MATH=ON',  ], },
                       'creates'     => 'include/glm/glm.hpp',
                       'method'      => 'cmake', },

    # Caffe OpenCL
    'gflags'       => {'url'         => 'https://github.com/gflags/gflags/archive/v2.2.1.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTING=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTING=OFF', ], },
                       'creates'     => 'lib/libgflags.a',
                       'method'      => 'cmake', },
    'gtest'        => {'url'         => 'https://github.com/google/googletest/archive/release-1.8.0.tar.gz',
                       'creates'     => 'lib/libgtest.a',
                       'method'      => 'cmake', },
    'snappy'       => {'url'         => 'https://github.com/google/snappy/archive/1.1.7.tar.gz',
                       'args'        => { 'native'   => [ '-DSNAPPY_BUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DSNAPPY_BUILD_TESTS=OFF', ], },
                       'require'     => [ Installer['gtest'], Installer['gflags'], ],
                       'creates'     => 'lib/libsnappy.a',
                       'method'      => 'cmake', },
    'leveldb'      => {'url'         => '',
                       'creates'     => 'lib/leveldb.a',
                       'prebuild'    => 'wget -N https://github.com/google/leveldb/archive/v1.20.tar.gz &&
                                         tar -xf v1.20.tar.gz &&
                                         cd leveldb-1.20 &&
                                         make -j$(nproc) &&
                                         cp out-static/lib* out-shared/lib* PREFIX/lib/ &&
                                         cp -r include/* PREFIX/include/',
                       'creates'     => 'lib/libleveldb.a',
                       'require'     => [ Installer['snappy'], ],
                       'method'      => 'custom', },
    'lmdb'         => {'url'         => 'https://github.com/LMDB/lmdb/archive/LMDB_0.9.21.tar.gz',
                       'creates'     => 'lib/liblmdb.so',
                       'args'        => { 'native'   => [ 'prefix=PREFIX', ],
                                          'nuc7i7bnh' => [ 'prefix=PREFIX', ], },
                       'src_dir'     => 'libraries/liblmdb',
                       'method'      => 'make', },
    'hdf5'         => {'url'         => 'https://support.hdfgroup.org/ftp/HDF5/current/src/hdf5-1.10.1.tar.gz',
                       'args'        => { 'native'   => [ '-DHDF5_BUILD_CPP_LIB=ON', '-DHDF5_ENABLE_Z_LIB_SUPPORT=ON', ],
                                          'nuc7i7bnh' => [ '-DHDF5_BUILD_CPP_LIB=ON', '-DHDF5_ENABLE_Z_LIB_SUPPORT=ON', ], },
                       'creates'     => 'lib/libhdf5.so',
                       'method'      => 'cmake', },
    'glog'         => {'url'         => 'https://github.com/google/glog/archive/v0.3.5.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTING=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTING=OFF', ], },
                       'require'     => [ Installer['gflags'], ],
                       'creates'     => 'lib/libglog.a',
                       'method'      => 'cmake', },
    'opencv'       => {'url'         => 'https://github.com/opencv/opencv/archive/3.3.1.tar.gz',
                       'args'        => { 'native'   => [ '-DOPENCV_ENABLE_NONFREE=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_NVCUVID=OFF', '-DWITH_EIGEN=ON', '-DWITH_ARAVIS=ON', '-DWITH_TBB=ON', '-DWITH_OPENMP=ON', '-DWITH_OPENCL=ON', '-DBUILD_opencv_apps=OFFF', '-DBUILD_opencv_js=OFF', '-DBUILD_opencv_python3=ON', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_WITH_DEBUG_INFO=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_TIFF=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', ],
                                          'nuc7i7bnh' => [ '-DOPENCV_ENABLE_NONFREE=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_NVCUVID=OFF', '-DWITH_EIGEN=ON', '-DWITH_ARAVIS=ON', '-DWITH_TBB=ON', '-DWITH_OPENMP=ON', '-DWITH_OPENCL=ON', '-DBUILD_opencv_apps=OFFF', '-DBUILD_opencv_js=OFF', '-DBUILD_opencv_python3=ON', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_WITH_DEBUG_INFO=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_TIFF=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', ], },
                       'require'     => [ Installer['aravis'], Installer['eigen3'], ],
                       'creates'     => 'lib/libopencv_core.so',
                       'method'      => 'cmake', },
    'viennacl'     => {'url'         => 'https://github.com/viennacl/viennacl-dev/archive/release-1.7.1.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_EXAMPLES=OFF', '-DBUILD_TESTING=OFF', '-DOPENCL_LIBRARY=PREFIX/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DBUILD_EXAMPLES=OFF', '-DBUILD_TESTING=OFF', '-DOPENCL_LIBRARY=PREFIX/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'include/viennacl/version.hpp',
                       'method'      => 'cmake', },
    'clfft'        => {'url'         => 'https://github.com/clMathLibraries/clFFT/archive/v2.12.2.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_CLIENT=OFF', '-DBUILD_TEST=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_CALLBACK_CLIENT=OFF', '-DSUFFIX_LIB=""', ],
                                          'nuc7i7bnh' => [ '-DBUILD_CLIENT=OFF', '-DBUILD_TEST=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_CALLBACK_CLIENT=OFF', '-DSUFFIX_LIB=""', ], },
                       'creates'     => 'lib/libclFFT.so',
                       'src_dir'     => 'src',
                       'require'     => [ Installer['viennacl'], Installer['fftw3f'], Installer['fftw3'], Installer['boost'], ],
                       'method'      => 'cmake', },
    'isaac'        => {'url'         => 'https://github.com/intel/isaac/archive/master.tar.gz',
                       'creates'     => 'lib/libisaac.so',
                       'require'     => [ Installer['viennacl'], ],
                       'method'      => 'cmake', },
    'libdnn'       => {'url'         => 'https://github.com/naibaf7/libdnn/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DUSE_CUDA=OFF', '-DUSE_OPENCL=ON', '-DUSE_INDEX_64=OFF', ],
                                          'nuc7i7bnh' => [ '-DUSE_CUDA=OFF', '-DUSE_OPENCL=ON', '-DUSE_INDEX_64=OFF', ], },
                       'creates'     => 'lib/libgreentea_libdnn.so',
                       'require'     => [ Installer['viennacl'], ],
                       'method'      => 'cmake', },
    'caffe'        => {'url'         => 'https://github.com/01org/caffe/archive/inference-optimize.tar.gz',
                       'prebuild'    => 'export ISAAC_HOME=PREFIX',
                       'args'        => { 'native'   => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES=PREFIX/opt/intel/opencl/libOpenCL.so', '-DOPENCL_INCLUDE_DIRS=PREFIX/opt/intel/opencl/include', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_INDEX_64=OFF', '-DUSE_FFT=OFF', '-DBUILD_examples=OFF', '-DBUILD_tools=OFF', ],
                                          'nuc7i7bnh' => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES=PREFIX/opt/intel/opencl/libOpenCL.so', '-DOPENCL_INCLUDE_DIRS=PREFIX/opt/intel/opencl/include', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_INDEX_64=OFF', '-DUSE_FFT=OFF', '-DBUILD_examples=OFF', '-DBUILD_tools=OFF', ], },
                       'require'     => [ Installer['isaac'], Installer['boost'], Installer['hdf5'], Installer['leveldb'], Installer['lmdb'], Installer['glog'], Installer['gflags'], Installer['clfft'], Installer['libdnn'], Installer['opencv'], ],
                       'creates'     => 'lib/libcaffe.so',
                       'method'      => 'cmake', },
    'tensorflow'   => {'url'         => 'https://github.com/tensorflow/tensorflow/archive/master.tar.gz',
                       'prebuild'    => "patch -Np1 -i PREFIX/src/tensorflow.patch &&
                                         cp PREFIX/src/tensorflow_eigen3.patch PREFIX/src/tensorflow/third_party/eigen3/remove_unsupported_devices.patch &&
                                         /usr/bin/python3 tensorflow/tools/git/gen_git_source.py --configure .  &&
                                         echo \"export PYTHON_BIN_PATH=/usr/bin/python3\" > tools/python_bin_path.sh &&
                                         export BAZELRC=PREFIX/src/tensorflow.bazelrc",
                       'postbuild'   => "./bazel-bin/tensorflow/tools/pip_package/build_pip_package PREFIX/tmp/tensorflow_pkg &&
                                         /usr/bin/pip3 install --target PREFIX/lib/python3.5/site-packages PREFIX/tmp/tensorflow_pkg/tensorflow-1.4.0-cp35-cp35m-linux_x86_64.whl &&
                                         cp PREFIX/lib/python3.5/site-packages/tensorflow/libtensorflow_framework.so PREFIX/lib/ &&
                                         cp -r PREFIX/lib/python3.5/site-packages/tensorflow/include/tensorflow PREFIX/include/ &&
                                         export BAZELRC=PREFIX/src/tensorflow.bazelrc &&
                                         bazel --blazerc \$BAZELRC build --jobs \$(nproc) -c opt --config=sycl //tensorflow:libtensorflow_cc.so",
                       'args'        => { 'native'   => [ '-c opt', '--config=sycl', '//tensorflow/tools/pip_package:build_pip_package', '--verbose_failures', ],
                                          'nuc7i7bnh' => [ '-c opt', '--config=sycl', '//tensorflow/tools/pip_package:build_pip_package', '--verbose_failures', ], },
                       'require'     => [ Installer['protobuf'], ],
                       'creates'     => 'lib/libtensorflow_framework.so',
                       'method'      => 'bazel', },
  }

  file { '/usr/share/cmake-3.5/Modules/FindBoost.cmake':
      path   => '/usr/share/cmake-3.5/Modules/FindBoost.cmake',
      ensure => present,
      source => 'puppet:///modules/files/FindBoost.cmake',
   } -> Installer <| |>

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Hash,
                          Optional['require'] => Tuple[Any, 1, default],
                          'method' => String,
                          Optional['src_dir'] => String,
                          Optional['prebuild'] => String,
                          Optional['postbuild'] => String}] $params| {

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

        if $params['url'] != '' {
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
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ $params['require'], Class['installer::prerequisites'], Class['build_tools'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          extension   => $extension,
        }
  }

  # Install quex.
  class { 'quex': }

  # Install protobuf.
  class { 'protobuf': }

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
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

  # After we have installed, create the CMake toolchain files and then build our deb.
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

  file { "/nubots/toolchain/nuc7i7bnh/bin/pkg-config.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/x86_64-linux-gnu-pkg-config \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/nuc7i7bnh/bin/pkg-config.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/native/bin/pkg-config.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/pkg-config \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/native/bin/pkg-config.sh",
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
ac_cv_path_PKG_CONFIG=${prefix}/${arch}/bin/pkg-config.sh
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/aravis.config",
      mode    => "a-w",
      before  => [ Installer['aravis'], ],
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

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\")
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\")

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }

    file { "${prefix}/${arch}/bin/xorg-libs.sh":
      content => "#! /bin/bash

set -e

xorg_libs=(xtrans-1.3.5 libX11-1.6.5 libXext-1.3.3 libFS-1.0.7 libICE-1.0.9 libSM-1.2.2 libXScrnSaver-1.2.2 libXt-1.1.5 libXmu-1.1.2 libXpm-3.5.12 libXaw-1.0.13 libXfixes-5.0.3 libXcomposite-0.4.4 libXrender-0.9.10 libXcursor-1.1.14 libXdamage-1.1.4 libfontenc-1.1.3 libXfont2-2.0.2 libXft-2.3.2 libXi-1.7.9 libXinerama-1.1.3 libXrandr-1.5.1 libXres-1.2.0 libXtst-1.2.3 libXv-1.0.11 libXvMC-1.0.10 libXxf86dga-1.1.4 libXxf86vm-1.1.4 libdmx-1.1.3 libpciaccess-0.14 libxkbfile-1.0.9 libxshmfence-1.2)

for xorg_lib in \${xorg_libs[*]};
do
    wget https://www.x.org/pub/individual/lib/\"\${xorg_lib}.tar.bz2\" -O - | tar xjf -
    cd \"\${xorg_lib}\"
    case \${xorg_lib} in
        libICE* )
        ./configure --prefix=${prefix}/${arch} --disable-static ICE_LIBS=-lpthread
        ;;

        libXfont2-[0-9]* )
        ./configure --prefix=${prefix}/${arch} --disable-static --disable-devel-docs
        ;;

        libXt-[0-9]* )
        ./configure --prefix=${prefix}/${arch} --disable-static --with-appdefaultdir=${prefix}/${arch}/etc/X11/app-defaults
        ;;

        * )
        ./configure --prefix=${prefix}/${arch} --disable-static
        ;;
    esac
    make -j\$(nproc)
    make install
    cd ..
done
",
      ensure  => present,
      path    => "${prefix}/${arch}/bin/xorg-libs.sh",
      mode    => "a+x",
      before  => Installer['xorg-libs'],
    }

    file { "${prefix}/${arch}/bin/xorg-protos.sh":
      content => "#! /bin/bash

set -e

protos=(bigreqsproto-1.1.2 compositeproto-0.4.2 damageproto-1.2.1 dmxproto-2.3.1 dri2proto-2.8 dri3proto-1.0 fixesproto-5.0 fontsproto-2.1.3 glproto-1.4.17 inputproto-2.3.2 kbproto-1.0.7 presentproto-1.1 randrproto-1.5.0 recordproto-1.14.2 renderproto-0.11.1 resourceproto-1.2.0 scrnsaverproto-1.2.2 videoproto-2.3.3 xcmiscproto-1.2.2 xextproto-7.3.0 xf86bigfontproto-1.2.0 xf86dgaproto-2.1 xf86driproto-2.1.1 xf86vidmodeproto-2.3.1 xineramaproto-1.2.1 xproto-7.0.31)

for proto in \${protos[*]};
do
    wget https://www.x.org/pub/individual/proto/\"\${proto}.tar.bz2\" -O - | tar xjf -
    cd \"\${proto}\"
    ./configure --prefix=${prefix}/${arch} --disable-static
    make -j\$(nproc)
    make install
    cd ..
done
",
      ensure  => present,
      path    => "${prefix}/${arch}/bin/xorg-protos.sh",
      mode    => "a+x",
      before  => Installer['xorg-protocol-headers'],
    }

    exec { "${prefix}-${arch}-intel-opencl":
      creates    => "${prefix}/${arch}/opt/intel/opencl/libOpenCL.so",
      command    => "cd ${prefix}/src &&
                     wget -N http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB4.1_linux64.zip &&
                     mkdir -p ${prefix}/${arch}/src/OpenCL-Linux/ &&
                     unzip -d ${prefix}/${arch}/src/OpenCL-Linux/ -o ${prefix}/src/SRB4.1_linux64.zip &&
                     cd ${prefix}/${arch}/src/OpenCL-Linux/ &&
                     mkdir -p intel-opencl &&
                     tar -C intel-opencl -Jxf intel-opencl-r4.1-61547.x86_64.tar.xz &&
                     tar -C intel-opencl -Jxf intel-opencl-devel-r4.1-61547.x86_64.tar.xz &&
                     tar -C intel-opencl -Jxf intel-opencl-cpu-r4.1-61547.x86_64.tar.xz &&
                     cp -R intel-opencl/* ${prefix}/${arch}/",
      cwd       => "${prefix}/${arch}/src",
      path      =>  [ '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout   => 0,
      provider  => 'shell',
      before    => Installer['viennacl'],
      require   => Class['installer::prerequisites'],
    }

    file { "${prefix}/${arch}/src/tensorflow.patch":
      content   => "
diff --git a/tensorflow/core/kernels/cwise_op_add_1.cc b/tensorflow/core/kernels/cwise_op_add_1.cc
index 608a6dce3d..059ad8c193 100644
--- a/tensorflow/core/kernels/cwise_op_add_1.cc
+++ b/tensorflow/core/kernels/cwise_op_add_1.cc
@@ -48,7 +48,7 @@ REGISTER_KERNEL_BUILDER(Name(\"AddV2\")
 #if TENSORFLOW_USE_SYCL
 #define REGISTER_KERNEL(type)                          \\
   REGISTER(BinaryOp, SYCL, \"Add\", functor::add, type); \\
-  REEGISTER(BinaryOp, SYCL, \"AddV2\", functor::add, type);
+  REGISTER(BinaryOp, SYCL, \"AddV2\", functor::add, type);

 TF_CALL_SYCL_NUMBER_TYPES(REGISTER_KERNEL);

diff --git a/tensorflow/core/kernels/random_op.cc b/tensorflow/core/kernels/random_op.cc
index 55a8b9c9b6..e96c445ab7 100644
--- a/tensorflow/core/kernels/random_op.cc
+++ b/tensorflow/core/kernels/random_op.cc
@@ -577,7 +577,7 @@ struct FillPhiloxRandomKernel<Distribution, false> {
     const size_t kGroupSize = Distribution::kResultElementCount;

     const size_t item_id = item.get_global(0);
-    const size_t total_item_count = item.get_global_range();
+    const size_t total_item_count = item.get_global_range().size();
     size_t offset = item_id * kGroupSize;
     gen_.Skip(item_id);

@@ -633,7 +633,7 @@ struct FillPhiloxRandomKernel<Distribution, true> {
                                                 PhiloxRandom::kResultElementCount;

     const size_t item_id = item.get_global(0);
-    const size_t total_item_count = item.get_global_range();
+    const size_t total_item_count = item.get_global_range().size();
     size_t group_index = item_id;
     size_t offset = group_index * kGroupSize;

diff --git a/tensorflow/core/kernels/transpose_functor_cpu.cc b/tensorflow/core/kernels/transpose_functor_cpu.cc
index 41b73fdaf4..245f5f6b8c 100644
--- a/tensorflow/core/kernels/transpose_functor_cpu.cc
+++ b/tensorflow/core/kernels/transpose_functor_cpu.cc
@@ -169,7 +169,7 @@ template <typename T, bool conjugate>
 struct Transpose<SYCLDevice, T, conjugate> {
   static void run(const SYCLDevice& d, const Tensor& in,
                   const gtl::ArraySlice<int32> perm, Tensor* out) {
-    internal::TransposeSycl(d, in, perm, conjugate, out);
+    internal::TransposeSYCL<T>(d, in, perm, conjugate, out);
   }
 };

diff --git a/tensorflow/workspace.bzl b/tensorflow/workspace.bzl
index 6a496f53f0..fe27f0aa02 100644
--- a/tensorflow/workspace.bzl
+++ b/tensorflow/workspace.bzl
@@ -95,12 +95,13 @@ def tf_workspace(path_prefix=\"\", tf_repo_name=\"\"):
   tf_http_archive(
       name = \"eigen_archive\",
       urls = [
-          \"https://mirror.bazel.build/bitbucket.org/eigen/eigen/get/c2947c341c68.tar.gz\",
-          \"https://bitbucket.org/eigen/eigen/get/c2947c341c68.tar.gz\",
+          \"https://mirror.bazel.build/bitbucket.org/eigen/eigen/get/default.tar.bz2\",
+          \"https://bitbucket.org/eigen/eigen/get/default.tar.bz2\",
       ],
-      sha256 = \"f21f8ab8a8dbcb91cd0deeade19a043f47708d0da7a4000164cdf203b4a71e34\",
-      strip_prefix = \"eigen-eigen-c2947c341c68\",
+      sha256 = \"17c0e255752c8382a58a75fcc46632a790eb51a99cfffa7b23ac5729b9362089\",
+      strip_prefix = \"eigen-eigen-034b6c3e1017\",
       build_file = str(Label(\"//third_party:eigen.BUILD\")),
+      patch_file = str(Label(\"//third_party/eigen3:remove_unsupported_devices.patch\")),
   )

   tf_http_archive(
@@ -295,11 +296,11 @@ def tf_workspace(path_prefix=\"\", tf_repo_name=\"\"):
   tf_http_archive(
       name = \"protobuf_archive\",
       urls = [
-          \"https://mirror.bazel.build/github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
-          \"https://github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
+          \"https://mirror.bazel.build/github.com/google/protobuf/archive/v3.5.0.tar.gz\",
+          \"https://github.com/google/protobuf/archive/v3.5.0.tar.gz\",
       ],
-      sha256 = \"e178a25c52efcb6b05988bdbeace4c0d3f2d2fe5b46696d1d9898875c3803d6a\",
-      strip_prefix = \"protobuf-b04e5cba356212e4e8c66c61bbe0c3a20537c5b9\",
+      sha256 = \"0cc6607e2daa675101e9b7398a436f09167dffb8ca0489b0307ff7260498c13c\",
+      strip_prefix = \"protobuf-3.5.0\",
       # TODO: remove patching when tensorflow stops linking same protos into
       #       multiple shared libraries loaded in runtime by python.
       #       This patch fixes a runtime crash when tensorflow is compiled
@@ -313,21 +314,21 @@ def tf_workspace(path_prefix=\"\", tf_repo_name=\"\"):
   tf_http_archive(
       name = \"com_google_protobuf\",
       urls = [
-          \"https://mirror.bazel.build/github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
-          \"https://github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
+          \"https://mirror.bazel.build/github.com/google/protobuf/archive/v3.5.0.tar.gz\",
+          \"https://github.com/google/protobuf/archive/v3.5.0.tar.gz\",
       ],
-      sha256 = \"e178a25c52efcb6b05988bdbeace4c0d3f2d2fe5b46696d1d9898875c3803d6a\",
-      strip_prefix = \"protobuf-b04e5cba356212e4e8c66c61bbe0c3a20537c5b9\",
+      sha256 = \"0cc6607e2daa675101e9b7398a436f09167dffb8ca0489b0307ff7260498c13c\",
+      strip_prefix = \"protobuf-3.5.0\",
   )

   tf_http_archive(
       name = \"com_google_protobuf_cc\",
       urls = [
-          \"https://mirror.bazel.build/github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
-          \"https://github.com/google/protobuf/archive/b04e5cba356212e4e8c66c61bbe0c3a20537c5b9.tar.gz\",
+          \"https://mirror.bazel.build/github.com/google/protobuf/archive/v3.5.0.tar.gz\",
+          \"https://github.com/google/protobuf/archive/v3.5.0.tar.gz\",
       ],
-      sha256 = \"e178a25c52efcb6b05988bdbeace4c0d3f2d2fe5b46696d1d9898875c3803d6a\",
-      strip_prefix = \"protobuf-b04e5cba356212e4e8c66c61bbe0c3a20537c5b9\",
+      sha256 = \"0cc6607e2daa675101e9b7398a436f09167dffb8ca0489b0307ff7260498c13c\",
+      strip_prefix = \"protobuf-3.5.0\",
   )

   tf_http_archive(
diff --git a/third_party/protobuf/add_noinlines.patch b/third_party/protobuf/add_noinlines.patch
index af74798f06..960e480ad5 100644
--- a/third_party/protobuf/add_noinlines.patch
+++ b/third_party/protobuf/add_noinlines.patch
@@ -10,15 +10,6 @@ diff -u -r a/src/google/protobuf/compiler/cpp/cpp_file.cc b/src/google/protobuf/
          \"  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);\\n\"
          \"  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);\\n\"
          \"}\\n\"
-@@ -656,7 +656,7 @@
-   printer->Print(
-       \"}\\n\"
-       \"\\n\"
--      \"void InitDefaults() {\\n\"
-+      \"GOOGLE_ATTRIBUTE_NOINLINE void InitDefaults() {\\n\"
-       \"  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);\\n\"
-       \"  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);\\n\"
-       \"}\\n\");
 @@ -737,7 +737,7 @@
    printer->Print(
        \"}\\n\"
diff --git a/third_party/sycl/crosstool/computecpp.tpl b/third_party/sycl/crosstool/computecpp.tpl
index c699eabb6f..4c90e0f55e 100755
--- a/third_party/sycl/crosstool/computecpp.tpl
+++ b/third_party/sycl/crosstool/computecpp.tpl
@@ -60,7 +60,7 @@ def main():
     return pipe.returncode

   # check if it has parallel_for in it
-  if not '.parallel_for' in preprocessed_file_str:
+  if not '.parallel_for' in str(preprocessed_file_str):
     # call CXX compiler like usual
     with tempfile.NamedTemporaryFile(suffix=\".ii\") as preprocessed_file: # Force '.ii' extension so that g++ does not preprocess the file again
       preprocessed_file.write(preprocessed_file_str)
diff --git a/third_party/sycl/sycl/BUILD.tpl b/third_party/sycl/sycl/BUILD.tpl
index 21b1a2bbf7..28011eff20 100755
--- a/third_party/sycl/sycl/BUILD.tpl
+++ b/third_party/sycl/sycl/BUILD.tpl
@@ -20,8 +20,8 @@ config_setting(
 config_setting(
     name = \"using_sycl_trisycl\",
     define_values = {
-        \"using_sycl\": \"true\",
-        \"using_trisycl\": \"false\",
+        \"using_sycl\": \"false\",
+        \"using_trisycl\": \"true\",
     },
 )
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/tensorflow.patch",
      mode    => "a+r",
      before  => Installer['tensorflow'],
    }

    file { "${prefix}/${arch}/src/tensorflow_eigen3.patch":
      content =>
"
--- a/unsupported/Eigen/CXX11/src/Tensor/TensorDeviceSycl.h 2017-12-24 10:09:13.505638510 +1100
+++ b/unsupported/Eigen/CXX11/src/Tensor/TensorDeviceSycl.h 2017-12-24 10:08:53.265413927 +1100
@@ -110,12 +110,12 @@
   for(const auto& device : device_list){
     auto vendor = device.template get_info<cl::sycl::info::device::vendor>();
     std::transform(vendor.begin(), vendor.end(), vendor.begin(), ::tolower);
-    bool unsuported_condition = (device.is_cpu() && platform_name.find(\"amd\")!=std::string::npos && vendor.find(\"apu\") == std::string::npos) ||
-    (device.is_gpu() && platform_name.find(\"intel\")!=std::string::npos);
-    if(!unsuported_condition){
+    //bool unsuported_condition = (device.is_cpu() && platform_name.find(\"amd\")!=std::string::npos && vendor.find(\"apu\") == std::string::npos) ||
+    //(device.is_gpu() && platform_name.find(\"intel\")!=std::string::npos);
+    //if(!unsuported_condition){
       std::cout << \"Platform name \"<< platform_name << std::endl;
         supported_devices.push_back(device);
-    }
+    //}
   }
 }
 return supported_devices;
 ",
      ensure  => present,
      path    => "${prefix}/${arch}/src/tensorflow_eigen3.patch",
      mode    => "a+r",
      before  => Installer['tensorflow'],
    }

    exec { "install-${arch}-computecpp":
      creates   => "${prefix}/${arch}/bin/compute++",
      command   => "cd ${prefix}/src &&
                    wget -N http://nubots.net/tarballs/ComputeCpp-CE-0.4.0-Ubuntu.16.04-64bit.tar.gz &&
                    tar xf ${prefix}/src/ComputeCpp-CE-0.4.0-Ubuntu.16.04-64bit.tar.gz &&
                    cp -r ${prefix}/src/ComputeCpp-CE-0.4.0-Ubuntu-16.04-64bit/* ${prefix}/${arch}/",
      cwd       => "${prefix}/src",
      path      =>  [ '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout   => 0,
      provider  => 'shell',
      before    => Installer['tensorflow'],
      require   => Class['installer::prerequisites'],
    }

    $bazel_c_options   = join(prefix($params['flags'], 'build:opt --copt='), "\n")
    $bazel_c_params    = join(prefix(regsubst($params['params'], ' ', '\nbuild:opt --copt='), 'build:opt --copt='), "\n")
    $bazel_cxx_options = join(prefix($params['flags'], 'build:opt --cxxopt='), "\n")
    $bazel_cxx_params  = join(prefix(regsubst($params['params'], ' ', "\nbuild:opt --cxxopt="), 'build:opt --cxxopt='), "\n")

    file { "${prefix}/${arch}/src/tensorflow.bazelrc":
      content   => "
build --action_env PATH=\"${prefix}/${arch}/bin:${prefix}/bin:/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin\"
build --action_env PYTHON_BIN_PATH=\"/usr/bin/python3\"
build --action_env PYTHON_LIB_PATH=\"/usr/lib/python3/dist-packages\"
build --force_python=py3
build --host_force_python=py3
build --python_path=\"/usr/bin/python3\"
build:gcp --define with_gcp_support=true
build:hdfs --define with_hdfs_support=true
build:s3 --define with_s3_support=true
build:xla --define with_xla_support=true
build:gdr --define with_gdr_support=true
build:verbs --define with_verbs_support=true
build --action_env TF_NEED_OPENCL_SYCL=\"1\"
build --action_env HOST_CXX_COMPILER=\"/usr/bin/g++\"
build --action_env HOST_C_COMPILER=\"/usr/bin/gcc\"
build --action_env TF_NEED_COMPUTECPP=\"1\"
build --action_env COMPUTECPP_TOOLKIT_PATH=\"${prefix}/${arch}\"
build --action_env TF_NEED_CUDA=\"0\"
build --define grpc_no_ares=true
${bazel_c_options}
${bazel_c_params}
${bazel_cxx_options}
${bazel_cxx_params}
build:opt --define with_default_optimizations=true
build --copt=-DGEMMLOWP_ALLOW_SLOW_SCALAR_FALLBACK
build --host_copt=-DGEMMLOWP_ALLOW_SLOW_SCALAR_FALLBACK
build:monolithic --define framework_shared_object=false
build --define framework_shared_object=true
build:android --crosstool_top=//external:android/crosstool
build:android --host_crosstool_top=@bazel_tools//tools/cpp:toolchain
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/tensorflow.bazelrc",
      mode    => "a+r",
      before  => Installer['tensorflow'],
    }
  }
}
