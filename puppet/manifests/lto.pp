include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  file { '/nubots': ensure => directory, } ->
  file { '/nubots/toolchain': ensure => directory, } ->
  file { 'toolchain_include_dir':
    path => '/nubots/toolchain/include',
    ensure => directory,
  }

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  class { 'quex': }
  installer { 'zlib':           url => 'http://zlib.net/zlib-1.2.8.tar.gz', }
  installer { 'protobuf':       url => 'https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz',
                                args => '--with-zlib',
                                require => Installer['zlib'], }
  installer { 'pgm':            url => 'https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz',
                                strip_components => 3, }
  installer { 'zmq':            url => 'http://download.zeromq.org/zeromq-4.1.2.tar.gz',
                                args => '-with-pgm --without-libsodium',
                                require => Installer['pgm'], }
  wget::fetch { 'zmq.hpp':      destination => '/nubots/toolchain/include/zmq.hpp',
                                source => 'https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp',
                                require => [ Installer['zmq'], File['toolchain_include_dir'] ], }
  wget::fetch { 'catch.hpp':    destination => '/nubots/toolchain/include/catch.hpp',
                                source => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
                                require => File['toolchain_include_dir'], }
  installer { 'nuclear':        url => 'https://github.com/Fastcode/NUClear/archive/OldDSL.tar.gz',
                                args => '-DNUCLEAR_BUILD_TESTS=OFF',
                                require => [ Installer['zmq'], Installer['protobuf'], Wget::Fetch['zmq.hpp'] , Wget::Fetch['catch.hpp'] ], }
  installer { 'openblas':       url => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.14.tar.gz',
                                environment => ['TARGET=ATOM', 'USE_THREAD=1', 'BINARY=32'], }
  installer { 'armadillo':      url => 'http://sourceforge.net/projects/arma/files/armadillo-5.200.2.tar.gz',
                                method => 'cmake',
                                require => Installer['openblas'], }
  installer { 'tcmalloc':       url => 'https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz',
                                args => '--with-tcmalloc-pagesize=64 --enable-minimal', }
  installer { 'yaml-cpp':       url => 'https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz',
                                args => '-DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TOOLS=OFF', }
  installer { 'ncurses':        url => 'http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz',
                                args => '--without-progs --without-test --with-shared',
                                environment => [ 'CPPFLAGS=-P' ] }
  installer { 'fftw3':          url => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                                args => '--disable-fortran --enable-shared', }
  installer { 'libjpeg-turbo':  url => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.1/libjpeg-turbo-1.4.1.tar.gz',
                                args => '--build=i686-linux-gnu --host=i686-linux-gnu',
                                lto => false, }
  installer { 'cppformat':      url => 'https://github.com/cppformat/cppformat/archive/1.1.0.tar.gz', }
  installer { 'alsalib':        url => 'ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.29.tar.bz2',
                                lto => false, }
  installer { 'portaudio':      url => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                                require => Installer['alsalib'],
                                lto => false, }
  # installer { 'espeak':         url => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
  #                               environment => ['AUDIO="PORTAUDIO"'],
  #                               require => Installer['portaudio'], }
  installer { 'rtaudio':        url => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz', }
  installer { 'muparser':       url => 'https://github.com/TrentHouliston/muparser/archive/master.tar.gz',
                                args => '--disable-shared --disable-debug --disable-samples', }
  installer { 'eigen3':         url => 'http://bitbucket.org/eigen/eigen/get/3.2.5.tar.gz', }

  package { 'libboost-system-dev': }
  package { 'libboost-filesystem-dev': }
  package { 'libboost-thread-dev': }
  package { 'libboost-serialization-dev': }
  package { 'libboost-program-options-dev': }
  package { 'libboost-test-dev': }
  package { 'libboost-chrono-dev': }
  package { 'libboost-date-time-dev': }

  installer { 'ompl':           url => 'https://github.com/ompl/ompl/archive/master.tar.gz',
                                args => "-DOMPL_BUILD_DEMOS -DOMPL_BUILD_PYBINDINGS -DOMPL_BUILD_PYTESTS",
                                environment => [ 'CXXFLAGS=-std=c++11' ],
                                require => [ Installer['eigen3'],
                                             Package['libboost-system-dev'],
                                             Package['libboost-filesystem-dev'],
                                             Package['libboost-thread-dev'],
                                             Package['libboost-serialization-dev'],
                                             Package['libboost-program-options-dev'],
                                             Package['libboost-test-dev'],
                                             Package['libboost-chrono-dev'],
                                             Package['libboost-date-time-dev'] ], }

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
    command => "/usr/local/bin/install_from_source '/nubots/toolchain' 'auto' true",
    creates => "/nubots/toolchain/lib/libespeak.a",
    cwd => "/nubots/toolchain/src/espeak/espeak-1.48.04-source/src",
    environment => ['AUDIO="PORTAUDIO"'],
    path =>  [  '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout => 0,
    refreshonly => true,
    require => [
      File['install_from_source'],
      Archive["espeak"],
      Exec['fix_compiler_environment'],
    ],
  }
}
