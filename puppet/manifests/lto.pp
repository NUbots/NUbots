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
  Package <| |>

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
  package { 'gfortran': ensure => latest, }
  package { 'ninja-build': ensure => latest, }
  package { 'yasm': ensure => latest, }

  installer { 'zlib':           url => 'http://zlib.net/zlib-1.2.8.tar.gz', }
  installer { 'protobuf':       url => 'https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz',
                                args => '--with-zlib',
                                require => Installer['zlib'], }
  installer { 'openpgm':        url => 'https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz',
                                strip_components => 3, }
  # installer { 'zeromq':         url => 'http://download.zeromq.org/zeromq-4.1.2.tar.gz',
  #                               args => '-with-pgm --without-libsodium',
  #                               environment => [ 'pgm_CFLAGS=""', 'pgm_LIBS=""'], }
  # installer { 'nuclear':        url => 'https://github.com/Fastcode/NUClear/archive/OldDSL.tar.gz',
  #                               args => '-DNUCLEAR_BUILD_TESTS=OFF',
  #                               require => [ Installer['zeromq'], Installer['protobuf']], }
  installer { 'openblas':       url => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.14.tar.gz',
                                environment => ['TARGET=ATOM',
                                                'USE_THREAD=1',
                                                'BINARY=32']}
  # installer { 'armadillo':      url => 'http://sourceforge.net/projects/arma/files/armadillo-5.200.2.tar.gz',
  #                               require => Installer['openblas'] }



  # # https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp

  # installer { 'tcmalloc':       url => 'https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz',
  #                               args => '--with-tcmalloc-pagesize=64 --enable-minimal', }
  # installer { 'yaml-cpp':       url => 'https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz',
  #                               args => '-DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TOOLS=OFF', }
  # installer { 'ncurses':        url => 'http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz',
  #                               args => '--without-progs --without-test', }
  # installer { 'fftw3':          url => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
  #                               args => '--disable-fortran --enable-shared', }
  # installer { 'libjpeg-turbo':  url => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.1/libjpeg-turbo-1.4.1.tar.gz',
  #                               args => '--build=i686-linux-gnu --host=i686-linux-gnu', }
  # installer { 'cppformat':      url => 'https://github.com/cppformat/cppformat/archive/1.1.0.tar.gz', }
  # installer { 'alsalib':        url => 'ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.29.tar.bz2', }
  # installer { 'portaudio':      url => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
  #                               require => Installer['alsalib'], }
  # installer { 'espeak':         url => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
  #                               environment => ['AUDIO="PORTAUDIO"'],
  #                               require => Installer['portaudio'], }
  # installer { 'muparser':       url => 'https://downloads.sourceforge.net/project/muparser/muparser/Version%202.2.3/muparser_v2_2_3.zip',
  #                               args => '--disable-shared --disable-debug --disable-samples'}
}

  # Install dependencies

  # zlib
  #     http://zlib.net/zlib-1.2.8.tar.gz

  # libprotobuf + protobuf compiler
  #     https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
  #     --with-zlib

  # openpgm
  #     https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz

  # libzmq4
  #     http://download.zeromq.org/zeromq-4.1.2.tar.gz

  # NUClear OldDSL
  #     https://github.com/Fastcode/NUClear/archive/OldDSL.tar.gz

  # OpenBLAS
 #  https://github.com/xianyi/OpenBLAS/archive/v0.2.14.tar.gz | tar -xz \
 # && cd OpenBLAS-0.2.14 \
 # && TARGET=ATOM \
 #    USE_THREAD=1 \
 #    BINARY=32 \
 #    COMMON_OPT="$COMPILER_FLAGS -O3" \
 #    FCOMMON_OPT="$COMPILER_FLAGS -O3" \
 #    make \
 # && make PREFIX="$TOOLCHAIN_PATH" install \
 # && cd .. \
 # && rm -rf OpenBLAS-0.2.13

# cmake_install http://sourceforge.net/projects/arma/files/armadillo-5.200.2.tar.gz \
#  && sed -i 's/^\/\* #undef ARMA_USE_LAPACK \*\//#define ARMA_USE_LAPACK/' $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
#  && sed -i 's/^#define ARMA_USE_WRAPPER/\/\/ #define ARMA_USE_WRAPPER/'   $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
#  && sed -i 's/^\/\/ #define ARMA_USE_CXX11/#define ARMA_USE_CXX11/'       $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
#  && sed -i 's/^\/\/ #define ARMA_USE_U64S64/#define ARMA_USE_U64S64/'     $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp

# https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp

# TCmalloc
# https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz
# --with-tcmalloc-pagesize=64 \
#     --enable-minimal

# https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.tar.gz \
    # -DYAML_CPP_BUILD_CONTRIB=OFF \
    # -DYAML_CPP_BUILD_TOOLS=OFF

# http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz
#--without-progs \
    # --without-tests

    # http://www.fftw.org/fftw-3.3.4.tar.gz \
    # --disable-fortran \
    # --enable-shared

# http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.1/libjpeg-turbo-1.4.1.tar.gz \
#     --build=i686-linux-gnu \
#     --host=i686-linux-gnu

# https://github.com/cppformat/cppformat/archive/1.1.0.tar.gz

# ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.29.tar.bz2

# http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz

# http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip

# https://downloads.sourceforge.net/project/muparser/muparser/Version%202.2.3/muparser_v2_2_3.zip \
#     --disable-shared \
#     --disable-debug \
#     --disable-samples

#   include initial_apt_update
#   include virtualbox_sharing_fix
#   include archive::prerequisites


#   # apt-add-repository ubutnu-toolchain-r/test
#   # apt-get dist-upgrade -y

#   # Install build-essential gfortran cmake cmake-curses-gui ninja-build vim screen htop gdb linux-headers-generic dos2unix rsync

#   # define variables for this node
#   $username = 'vagrant'

#   class { 'robocup::build_dep': username => $username, }
#   class { 'cppformat':  }

#   class { 'nusight': username => $username, }

#   # ps3 controller tools
#   package { 'software-properties-common': ensure => latest, }
