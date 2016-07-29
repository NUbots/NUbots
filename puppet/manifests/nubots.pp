include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  # We need dev tools
  class {'dev_tools': }

  # Get and install our toolchain
  $toolchain_version = '1.1.6'
  wget::fetch { 'nubots_deb':
    destination => "/root/nubots-toolchain${toolchain_version}.deb",
    source => "http://nubots.net/debs/nubots-toolchain${toolchain_version}.deb",
    timeout => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/root/nubots-toolchain${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
  $archs = {
    'DarwinOp' => {'flags'       => ['-march=bonnell', '-mtune=bonnell', '-m32', '-mno-movbe', '-mfxsr', '-mmmx', '-msahf', '-msse', '-msse2', '-msse3', '-mssse3', ],
                   'params'      => ['--param l1-cache-size=24', '--param l1-cache-line-size=64', '--param l2-cache-size=512', ],
                   'environment' => {'TARGET' => 'YONAH', 'USE_THREAD' => '1', 'BINARY' => '32', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m32', },
                   },
    'NimbroOp' => {'flags'       => ['-march=broadwell', '-mtune=broadwell', '-m64', '-mabm', '-madx', '-maes', '-mavx', '-mavx2', '-mbmi', '-mbmi2', '-mcx16', '-mf16c', '-mfma', '-mfsgsbase', '-mfxsr', '-mlzcnt', '-mmmx', '-mmovbe', '-mpclmul', '-mpopcnt', '-mprfchw', '-mrdrnd', '-mrdseed', '-msahf', '-msse', '-msse2', '-msse3', '-msse4', '-msse4.1', '-msse4.2', '-mssse3', '-mxsave', '-mxsaveopt', ],
                   'params'      => ['--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                   'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    'zlib'         => {'url'         => 'http://zlib.net/zlib-1.2.8.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake',},
    'bzip2-static' => {'url'         => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz',
                       'creates'     => 'lib/libbz2.a',
                       'method'      => 'make',},
    'bzip2-shared' => {'url'         => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz',
                       'creates'     => 'lib/libbz2.so',
                       'args'        => { 'DarwinOp' => [ '-f Makefile-libbz2_so', ],
                                          'NimbroOp' => [ '-f Makefile-libbz2_so', ], },
                       'method'      => 'make',
                       'require'     => [ Exec['correct_bzip2-shared_Makefile_08'], ],},
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'NimbroOp' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools',},
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/develop.tar.gz',
                       'args'        => { 'DarwinOp' => [ '-DBUILD_TESTS=OFF', ],
                                          'NimbroOp' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake',},
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.18.tar.gz',
                       'method'      => 'make',},
    'libsvm'       => {'url'         => 'https://github.com/cjlin1/libsvm/archive/v321.tar.gz',
                       'creates'     =>'lib/svm.o',
                       'method'      => 'make',
                       'require'     => [ File_line['correct_libsvm_Makefile_06'], ],},
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.200.2.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ],},
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5/gperftools-2.5.tar.gz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'NimbroOp' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools',},
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                       'args'        => { 'DarwinOp' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'NimbroOp' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake',},
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-fortran', '--enable-shared', ],
                                          'NimbroOp' => [ '--disable-fortran', '--enable-shared', ], },
                       'method'      => 'autotools',},
    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.2/libjpeg-turbo-1.4.2.tar.gz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', 'CCASFLAGS="-f elf32"', ],
                                          'NimbroOp' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools',},
    'cppformat'    => {'url'         => 'https://github.com/cppformat/cppformat/archive/2.0.0.tar.gz',
                       'method'      => 'cmake',},
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools',},
    'rtaudio'      => {'url'         => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz',
                       'args'        => { 'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools',},
    'muparserx'    => {'url'         => 'https://github.com/beltoforion/muparserx/archive/v4.0.4.tar.gz',
                       'method'      => 'cmake',},
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',},
    'boost'        => {'url'         => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                       'args'        => { 'DarwinOp' => [ 'address-model=32', 'architecture=x86', 'link=static', ],
                                          'NimbroOp' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2-shared'], Installer['bzip2-static'], ],},
    'mlpack'       => {'url'         => 'https://github.com/mlpack/mlpack/archive/mlpack-2.0.0.tar.gz',
                       'args'        => { 'DarwinOp' => [ '-DLAPACK_LIBRARY=PREFIX/lib/libopenblas.so', '-DBLAS_LIBRARY=PREFIX/lib/libopenblas.so', ],
                                          'NimbroOp' => [ '-DLAPACK_LIBRARY=PREFIX/lib/libopenblas.so', '-DBLAS_LIBRARY=PREFIX/lib/libopenblas.so', ], },
                       'require'     => [ Exec['correct_mlpack_CMakeLists'], Installer['armadillo'], Installer['boost'], Installer['xml2'], ],
                       'creates'     => 'lib/libmlpack.so',
                       'method'      => 'cmake',},
    'espeak'       => {'url'         => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
                       'src_dir'     => 'espeak-1.48.04-source/src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ File_line['correct_espeak_Makefile'], Installer['portaudio'] ]},
  }

  # Correct CXXFLAGS definition in eSpeak Makefile to firstly append to CXXFLAGS and to allow narrowing conversions to be treated as warnings.
  file_line { 'correct_espeak_Makefile':
    path    => '/nubots/toolchain/src/espeak/espeak-1.48.04-source/src/Makefile',
    match   => '^CXXFLAGS=-O2',
    line    => 'CXXFLAGS += -Wno-error=narrowing',
    ensure  => present,
    require => [ Archive['espeak'], ],
  }

  # Append an install target to the bzip2-shared Makefile.
  file_line { 'correct_bzip2-shared_Makefile_00':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => 'PREFIX=/usr/local',
    require => [ Archive['bzip2-shared'], ],
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_01':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => 'install: all'
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_02':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\ttest -d $(PREFIX) || mkdir -p $(PREFIX)",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_03':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\ttest -d $(PREFIX)/lib || mkdir -p $(PREFIX)/lib",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_04':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tinstall -m 0755 libbz2.so.1.0.6 $(PREFIX)/lib",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_05':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tln -s $(PREFIX)/lib/libbz2.so.1.0.6 $(PREFIX)/lib/libbz2.so.1.0",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_06':
    path    => '/nubots/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tln -s $(PREFIX)/lib/libbz2.so.1.0.6 $(PREFIX)/lib/libbz2.so",
  } ~>
  exec { 'correct_bzip2-shared_Makefile_07':
    command => "sed -i 's/\$(CC) -shared/\$(CC) \$(CFLAGS) -shared/' Makefile-libbz2_so",
    cwd     => '/nubots/toolchain/src/bzip2-shared',
    path    =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
  } ~>
  exec { 'correct_bzip2-shared_Makefile_08':
    command => "sed -i 's/^CFLAGS=/CFLAGS +=/' Makefile-libbz2_so",
    cwd     => '/nubots/toolchain/src/bzip2-shared',
    path    =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
  }

  # Append an install target to the libsvm Makefile.
  file_line { 'correct_libsvm_Makefile_00':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => 'PREFIX=/usr/local',
    require => [ Archive['libsvm'], ],
  } ~>
  file_line { 'correct_libsvm_Makefile_01':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => 'install: all'
  } ~>
  file_line { 'correct_libsvm_Makefile_02':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => "\ttest -d $(PREFIX) || mkdir -p $(PREFIX)",
  } ~>
  file_line { 'correct_libsvm_Makefile_03':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => "\ttest -d $(PREFIX)/include || mkdir -p $(PREFIX)/include",
  } ~>
  file_line { 'correct_libsvm_Makefile_04':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => "\ttest -d $(PREFIX)/lib || mkdir -p $(PREFIX)/lib",
  } ~>
  file_line { 'correct_libsvm_Makefile_05':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => "\tinstall -m 0644 svm.h $(PREFIX)/include",
  } ~>
  file_line { 'correct_libsvm_Makefile_06':
    path    => '/nubots/toolchain/src/libsvm/Makefile',
    line    => "\tinstall -m 0755 svm.o $(PREFIX)/lib",
  }

  # Prohibit the execution of tests when building mlpack
  exec { 'correct_mlpack_CMakeLists':
    command => "sed -i 's/^\s*tests/#tests/' CMakeLists.txt",
    cwd     => '/nubots/toolchain/src/mlpack/src/mlpack',
    path    =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    require => [ Archive['mlpack'], ],
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
          require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['dev_tools'], ])),
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

  $archs.each |String $arch, Hash $params| {
    # Create CMake toolchain files.
    $prefix          = '/nubots/toolchain'
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }

    # Ensure toolchain initialisation functions are generated.
    file { "${arch}_toolchain_init.sh":
      content =>
"export LD_LIBRARY_PATH=\"${prefix}/${arch}/lib\"
 export PATH=\"${prefix}/${arch}/bin:${prefix}/bin:/usr/local/bin:/usr/local/sbin:/usr/sbin:/usr/bin:/sbin:/bin\"
 export PKG_CONFIG_PATH=\"${prefix}/${arch}/lib/pkgconfig\"
 export CMAKE_PREFIX_PATH=\"${prefix}/${arch}\"",
      ensure  => present,
      path    => "${prefix}/${arch}_toolchain_init.sh",
      before  => Class['toolchain_deb'],
    }
  }
