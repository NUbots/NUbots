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
  }
  -> package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/root/nubots-toolchain${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
  $archs = {
    'DarwinOp' => {'abi'         => 32,
                   'args'        => ['-march=bonnell', '-mtune=bonnell', '-m32', '-mno-movbe', '-mfxsr', '-mmmx', '-msahf', '-msse', '-msse2', '-msse3', '-mssse3 --param l1-cache-size=24 --param l1-cache-line-size=64 --param l2-cache-size=512', ],
                   'environment' => ['TARGET=YONAH', 'USE_THREAD=1', 'BINARY=32', 'NUM_THREADS=2', 'AUDIO=PORTAUDIO']
                   },
    'NimbroOp' => {'abi'         => 64,
                   'args'        => ['-march=broadwell', '-mtune=broadwell', '-m64', '-mabm', '-madx', '-maes', '-mavx', '-mavx2', '-mbmi', '-mbmi2', '-mcx16', '-mf16c', '-mfma', '-mfsgsbase', '-mfxsr', '-mlzcnt', '-mmmx', '-mmovbe', '-mpclmul', '-mpopcnt', '-mprfchw', '-mrdrnd', '-mrdseed', '-msahf', '-msse', '-msse2', '-msse3', '-msse4', '-msse4.1', '-msse4.2', '-mssse3', '-mxsave', '-mxsaveopt --param l1-cache-size=32 --param l1-cache-line-size=64 --param l2-cache-size=4096', ],
                   'environment' => ['TARGET=HASWELL', 'USE_THREAD=1', 'BINARY=64', 'NUM_THREADS=2', 'AUDIO=PORTAUDIO']
                  },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # After we have installed, build our deb
  Installer <| |> ~> class { 'toolchain_deb': }

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
      'zlib'      => {'url'         => 'http://zlib.net/zlib-1.2.8.tar.gz',
                      'creates'     => 'libz.a'},
      'bzip2'     => {'url'         => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz'},
      'xml2'      => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz'},
      'protobuf'  => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.0.0-beta-3/protobuf-python-3.0.0-beta-3.tar.gz',
                      'args'        => [ '--with-zlib', ],
                      'require'     => [ Installer['zlib'], ]},
      'nuclear'   => {'url'         => 'https://github.com/Fastcode/NUClear/archive/develop.tar.gz',
                      'args'        => [ '-DBUILD_TESTS=OFF', ]},
      'openblas'  => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.18.tar.gz',
                      'method'      => 'make'},
      'libsvm'    => {'url'         => 'https://github.com/cjlin1/libsvm/archive/v321.tar.gz',
                      'creates'     =>'svm.o',
                      'method'      => 'make'},
      'armadillo' => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.200.2.tar.xz',
                      'method'      => 'cmake',
                      'creates'     => 'libarmadillo.so',
                      'require'     => [ Installer['openblas'], ]},
      'tcmalloc'  => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5/gperftools-2.5.tar.gz',
                      'args'        => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                      'creates'     => 'libtcmalloc_minimal.a'},
      'yaml-cpp'  => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                      'args'        => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ]},
      'fftw3'     => {'url'         => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                      'args'        => [ '--disable-fortran', '--enable-shared', ]},
      'jpeg'      => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.2/libjpeg-turbo-1.4.2.tar.gz'},
      'cppformat' => {'url'         => 'https://github.com/cppformat/cppformat/archive/2.0.0.tar.gz'},
      'portaudio' => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz'},
      'rtaudio'   => {'url'         => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz'},
      'muparserx' => {'url'         => 'https://github.com/beltoforion/muparserx/archive/v4.0.4.tar.gz'},
      'eigen3'    => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz',
                      'creates'     => 'Eigen'},
      'boost'     => {'url'         => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                      'args'        => [ 'link=static', ],
                      'require'     => [ Package['python-dev'], Installer['zlib'], Installer['bzip2'], ]},
      'mlpack'    => {'url'         => 'https://github.com/mlpack/mlpack/archive/mlpack-2.0.0.tar.gz',
                      'require'     => [ Installer['armadillo'], Installer['boost'], Installer['xml2'], ],
                      'creates'     => 'libmlpack.so'},
      'espeak'    => {'url'         => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
                      'src_dir'     => 'espeak-1.48.04-source/src',
                      'prebuild'    => 'cp portaudio19.h portaudio.h',
                      'require'     => [ Installer['portaudio'] ]},
  }

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Array[String, 1, default],
                          Optional['require'] => Tuple[Any, 1, default],
                          Optional['method'] => String,
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
          require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        } ~>
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => $params['require'],
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

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    # Find the correct ABI (for libsvm).
    if $params['abi'] == 64 {
      $abi = $params['abi']
    }

    else {
      $abi = ''
    }

    # Update the armadillo config header file for all archs.
    file { "armadillo_${arch}_config":
        path    => "/nubots/toolchain/${arch}/include/armadillo_bits/config.hpp",
        source  => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
        ensure  => present,
        require => [ Installer['armadillo'], ],
    }

    # Copy the libsvm header and library to their correct locations.
    file { "svm_${arch}_include_postbuild":
        path    => "/nubots/toolchain/${arch}/include/svm.h",
        source  => "/nubots/toolchain/${arch}/src/libsvm/svm.h",
        ensure  => present,
        require => [ Installer['libsvm'], ],
    }
    file { "svm_${arch}_lib_postbuild":
        path    => "/nubots/toolchain/${arch}/lib${abi}/svm.o",
        source  => "/nubots/toolchain/${arch}/src/libsvm/svm.o",
        ensure  => present,
        require => [ Installer['libsvm'], ],
    }
  }
}
