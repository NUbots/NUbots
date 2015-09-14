include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  package { 'libboost-system-dev': }
  package { 'libboost-filesystem-dev': }
  package { 'libboost-thread-dev': }
  package { 'libboost-serialization-dev': }
  package { 'libboost-program-options-dev': }
  package { 'libboost-test-dev': }
  package { 'libboost-chrono-dev': }
  package { 'libboost-date-time-dev': }

  # We need dev tools
  class {'dev_tools': }

  # Get and install our toolchain
  $toolchain_version = '1.0.7'
  exec { "nubots_toolchain":
    command => "wget http://nubots.net/debs/nubots-toolchain${toolchain_version}.deb",
    creates => "/tmp/nubots-toolchain${toolchain_version}.deb",
    cwd => "/tmp/",
  }
  ~> package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/tmp/nubots-toolchain${toolchain_version}.deb",
  }
}

node nubotsvmbuild {


  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # After we have installed, build our deb
  # Installer <| |> ~> class { 'toolchain_deb': }

  class { 'quex': }
  installer { 'zlib':           url => 'http://zlib.net/zlib-1.2.8.tar.gz', }
  installer { 'bzip2':          url => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz', }
  installer { 'xml2':           url => 'http://xmlsoft.org/sources/libxml2-2.9.2.tar.gz', }
  installer { 'protobuf':       url => 'https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz',
                                args => '--with-zlib',
                                require => Installer['zlib'], }
  installer { 'fswatch':        url => 'https://github.com/emcrisostomo/fswatch/releases/download/1.5.1/fswatch-1.5.1.tar.gz', }
  wget::fetch { 'catch.hpp':    destination => '/nubots/toolchain/include/catch.hpp',
                                source => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
                                require => Class['installer::prerequisites'], }
  installer { 'nuclear':        url => 'https://github.com/Fastcode/NUClear/archive/develop.tar.gz',
                                args => '-DBUILD_TESTS=OFF', }
  installer { 'openblas':       url => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.14.tar.gz',
                                environment => ['TARGET=YONAH', 'USE_THREAD=1', 'BINARY=32', 'NUM_THREADS=2'], }
  installer { 'armadillo':      url => 'http://sourceforge.net/projects/arma/files/armadillo-5.400.2.tar.gz',
                                method => 'cmake',
                                require => Installer['openblas'], }
  installer { 'tcmalloc':       url => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.4/gperftools-2.4.tar.gz',
                                args => '--with-tcmalloc-pagesize=64 --enable-minimal', }
  installer { 'yaml-cpp':       url => 'https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz',
                                args => '-DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TOOLS=OFF',
                                require => Installer['boost'], }
  # installer { 'ncurses':        url => 'http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz',
  #                               args => "--with-shared --mandir=/usr/share/man --without-profile --without-debug --disable-rpath --enable-echo --enable-const --enable-pc-files --without-ada --without-tests --without-progs --enable-symlinks --disable-lp64 --with-chtype='long' --with-mmask-t='long' --disable-termcap --with-default-terminfo-dir=/etc/terminfo --with-terminfo-dirs='/etc/terminfo:/lib/terminfo:/usr/share/terminfo' --with-ticlib=tic --with-termlib=tinfo --with-xterm-kbs=del",
  #                               environment => [ 'CPPFLAGS=-P', ], }
  installer { 'fftw3':          url => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                                args => '--disable-fortran --enable-shared', }
  installer { 'jpeg':           url => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.1/libjpeg-turbo-1.4.1.tar.gz', }
  installer { 'cppformat':      url => 'https://github.com/cppformat/cppformat/archive/1.1.0.tar.gz', }
  installer { 'alsalib':        url => 'ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.29.tar.bz2',
                                lto => false, }
  installer { 'portaudio':      url => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                                require => Installer['alsalib'],
                                lto => false, }
  installer { 'rtaudio':        url => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz', }
  installer { 'muparser':       url => 'https://github.com/beltoforion/muparser/archive/v2.2.5.tar.gz',
                                args => '--disable-shared --disable-debug --disable-samples', }
  installer { 'eigen3':         url => 'http://bitbucket.org/eigen/eigen/get/3.2.5.tar.gz', }
  installer { 'boost':          url => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                                args => ['link=static'],
                                require => [ Installer['zlib'], Installer['bzip2'], ], }
  installer { 'mlpack':         url => 'https://github.com/mlpack/mlpack/archive/mlpack-1.0.12.tar.gz',
                                require => [ Installer['armadillo'], Installer['boost'], Installer['xml2'], ], }
  installer { 'ompl':           url => 'https://github.com/ompl/ompl/archive/master.tar.gz',
                                args => "-DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_PYTESTS=OFF -DCMAKE_INSTALL_LIBDIR=lib",
                                require => [ Installer['eigen3'], Installer['boost'] ], }

  # Patch armadillo
  file { 'armadillo_config':    path => '/nubots/toolchain/include/armadillo_bits/config.hpp',
                                source => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
                                ensure => present,
                                require => Installer['armadillo'], }

  # Install eSpeak
  archive { "espeak":
    url    => "http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip",
    target => '/nubots/toolchain/src',
    checksum => false,
    extension => "zip", } ~>
  file { '/nubots/toolchain/src/espeak/espeak-1.48.04-source/src/portaudio.h':
    ensure => present,
    mode => '666',
    source => '/nubots/toolchain/src/espeak/espeak-1.48.04-source/src/portaudio19.h', } ~>
  exec { "install_espeak":
    command => "install_from_source '/nubots/toolchain' 'auto' true",
    creates => "/nubots/toolchain/lib/libespeak.a",
    cwd => "/nubots/toolchain/src/espeak/espeak-1.48.04-source/src",
    environment => ['AUDIO="PORTAUDIO"'],
    path =>  [  '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout => 0,
    refreshonly => true,
    require => [
      File['install_from_source'],
      Installer['portaudio'],
      Archive["espeak"],
      Exec['fix_compiler_environment'],
    ],
    # notify => Class['toolchain_deb'],
  }
}
