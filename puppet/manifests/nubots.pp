include apt
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  # We need dev tools
  class {'dev_tools': }

  # Get and install our toolchain
  $toolchain_version = '1.1.3'
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
  installer { 'openrave':       url => 'https://github.com/rdiankov/openrave/archive/v0.8.2.tar.gz',}
  
}

node nubotsvmbuild {

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # After we have installed, build our deb
  Installer <| |> ~> class { 'toolchain_deb': }

  class { 'quex': }
  installer { 'zlib':           url => 'http://zlib.net/zlib-1.2.8.tar.gz',
                                creates => '/nubots/toolchain/lib/libz.a' }
  installer { 'bzip2':          url => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz', }
  installer { 'xml2':           url => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz', }
  installer { 'protobuf':       url => 'https://github.com/google/protobuf/releases/download/v3.0.0-beta-2/protobuf-python-3.0.0-beta-2.tar.gz',
                                args => '--with-zlib',
                                require => Installer['zlib'], }
  installer { 'catch':          url => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp', }
  installer { 'nuclear':        url => 'https://github.com/Fastcode/NUClear/archive/develop.tar.gz',
                                args => '-DBUILD_TESTS=OFF', }
  installer { 'openblas':       url => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.15.tar.gz',
                                environment => ['TARGET=YONAH', 'USE_THREAD=1', 'BINARY=32', 'NUM_THREADS=2'],
                                method => 'make', }
  installer { 'libsvm':         url => 'https://github.com/cjlin1/libsvm/archive/v321.tar.gz',
                                method => 'make', }
  installer { 'armadillo':      url => 'https://downloads.sourceforge.net/project/arma/armadillo-6.500.5.tar.gz',
                                method => 'cmake',
                                creates => '/nubots/toolchain/lib/libarmadillo.so',
                                require => Installer['openblas'], }
  installer { 'tcmalloc':       url => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.4/gperftools-2.4.tar.gz',
                                args => '--with-tcmalloc-pagesize=64 --enable-minimal',
                                creates => '/nubots/toolchain/lib/libtcmalloc_minimal.a', }
  installer { 'yaml-cpp':       url => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                                args => '-DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TOOLS=OFF', }
  # installer { 'ncurses':        url => 'http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz',
  #                               args => "--with-shared --mandir=/usr/share/man --without-profile --without-debug --disable-rpath --enable-echo --enable-const --enable-pc-files --without-ada --without-tests --without-progs --enable-symlinks --disable-lp64 --with-chtype='long' --with-mmask-t='long' --disable-termcap --with-default-terminfo-dir=/etc/terminfo --with-terminfo-dirs='/etc/terminfo:/lib/terminfo:/usr/share/terminfo' --with-ticlib=tic --with-termlib=tinfo --with-xterm-kbs=del",
  #                               environment => [ 'CPPFLAGS=-P', ], }
  installer { 'fftw3':          url => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                                args => '--disable-fortran --enable-shared', }
  installer { 'jpeg':           url => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.2/libjpeg-turbo-1.4.2.tar.gz', }
  installer { 'cppformat':      url => 'https://github.com/cppformat/cppformat/archive/2.0.0.tar.gz', }
  installer { 'portaudio':      url => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                                lto => false, }
  installer { 'rtaudio':        url => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz', }
  installer { 'muparserx':      url => 'https://github.com/beltoforion/muparserx/archive/v4.0.4.tar.gz', }
  installer { 'eigen3':         url => 'http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz',
                                creates => '/nubots/toolchain/include/eigen3/Eigen/Eigen', }
  installer { 'boost':          url => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                                args => ['link=static'],
                                require => [ Package['python-dev'], Installer['zlib'], Installer['bzip2'], ], }
  installer { 'mlpack':         url => 'https://github.com/mlpack/mlpack/archive/mlpack-2.0.0.tar.gz',
                                require => [ Installer['armadillo'], Installer['boost'], Installer['xml2'], ],
                                creates => '/nubots/toolchain/lib/libmlpack.so', }
  installer { 'ompl':           url => 'https://bitbucket.org/ompl/ompl/downloads/ompl-1.1.0-Source.tar.gz',
                                args => "-DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_PYTESTS=OFF -DCMAKE_INSTALL_LIBDIR=lib",
                                creates => '/nubots/toolchain/lib/libompl.so',
                                require => [ Installer['eigen3'], Installer['boost'] ], }
  installer { 'espeak':         url => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
                                src_dir => 'espeak-1.48.04-source/src',
                                prebuild => 'cp portaudio19.h portaudio.h',
                                environment => ['AUDIO=PORTAUDIO'],
                                require => [ Installer['portaudio'] ], }
  installer { 'openrave':       url => 'https://github.com/rdiankov/openrave/archive/v0.8.2.tar.gz',}
  # Patch armadillo
  file { 'armadillo_config':    path => '/nubots/toolchain/include/armadillo_bits/config.hpp',
                                source => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
                                ensure => present,
                                require => Installer['armadillo'], }
}
