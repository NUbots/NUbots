include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

# All packages should be installed before building
Package <| |> -> Installer <| |>

node nubotsvm {

  exec { 'install apt-add-repository':
    command => "/usr/bin/apt-get install software-properties-common -y",
  } ->
  apt::ppa {'ppa:ubuntu-toolchain-r/test': } ->
  exec { "/usr/bin/apt-get update":
    timeout => 0,
  } ->
  exec { "/usr/bin/apt-get -y dist-upgrade":
    timeout => 0,
  } -> Package <| |>


  # Install developer tools
  package { 'vim': ensure => latest, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'gcc-5': ensure => latest, }
  package { 'gfortran-5': ensure => latest, }
  package { 'ninja-build': ensure => latest, }
  package { 'yasm': ensure => latest, }
  package { 'libboost-dev': ensure => latest, }

  installer { 'zlib':           url => 'http://zlib.net/zlib-1.2.8.tar.gz', }
  installer { 'protobuf':       url => 'https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz',
                                args => '--with-zlib',
                                require => Installer['zlib'], }
  installer { 'pgm':            url => 'https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz',
                                strip_components => 3, }
  installer { 'sodium':         url => 'https://github.com/jedisct1/libsodium/releases/download/1.0.3/libsodium-1.0.3.tar.gz' }
  installer { 'zmq':            url => 'http://download.zeromq.org/zeromq-4.1.2.tar.gz',
                                args => '-with-pgm --with-libsodium',
                                require => [ Installer['pgm'], Installer['sodium'] ], }
  exec { 'download_zmqhpp':     command => 'wget https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp',
                                creates => '/nubots/toolchain/include/zmq.hpp',
                                cwd => '/nubots/toolchain/include',
                                require => Installer['zmq'] }
  exec { 'download_catchhpp':   command => 'wget https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
                                creates => '/nubots/toolchain/include/catch.hpp',
                                cwd => '/nubots/toolchain/include', }
  installer { 'nuclear':        url => 'https://github.com/Fastcode/NUClear/archive/OldDSL.tar.gz',
                                args => '-DNUCLEAR_BUILD_TESTS=OFF',
                                require => [ Installer['zmq'], Installer['protobuf'], Exec['download_zmqhpp'] , Exec['download_catchhpp'] ], }
  installer { 'openblas':       url => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.14.tar.gz',
                                environment => ['TARGET=ATOM', 'USE_THREAD=1', 'BINARY=32'], }
  installer { 'armadillo':      url => 'http://sourceforge.net/projects/arma/files/armadillo-5.200.2.tar.gz',
                                method => 'cmake',
                                require => Installer['openblas'], }


  # installer { 'tcmalloc':       url => 'https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz',
  #                               args => '--with-tcmalloc-pagesize=64 --enable-minimal', }
  installer { 'yaml-cpp':       url => 'https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz',
                                args => '-DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TOOLS=OFF', }
  # installer { 'ncurses':        url => 'http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz',
  #                               args => '--without-progs --without-test', }
  installer { 'fftw3':          url => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                                args => '--disable-fortran --enable-shared', }
  installer { 'libjpeg-turbo':  url => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.1/libjpeg-turbo-1.4.1.tar.gz',
                                args => '--build=i686-linux-gnu --host=i686-linux-gnu', }
  # installer { 'espeak':         url => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
  #                               environment => ['AUDIO="PORTAUDIO"'],
  #                               require => Installer['portaudio'], }
  installer { 'cppformat':      url => 'https://github.com/cppformat/cppformat/archive/1.1.0.tar.gz', }
  installer { 'alsalib':        url => 'ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.29.tar.bz2',
                                lto => false, }
  installer { 'portaudio':      url => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                                require => Installer['alsalib'],
                                lto => false, }
  installer { 'muparser':       url => 'https://github.com/TrentHouliston/muparser/archive/master.tar.gz',
                                args => '--disable-shared --disable-debug --disable-samples', }
}
