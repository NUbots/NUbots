# == Class: armadillo
#
# Installs armadillo
#
class armadillo {
  package { 'libblas-dev': ensure => latest }
  package { 'liblapack-dev': ensure => latest }


  # Note: This installs an armadillo .deb that was manually created using the
  # following process:
  #   ~$ sudo apt-get install libblas-dev
  #   ~$ sudo apt-get install liblapack-dev
  #   ~$ sudo apt-get install libffi-dev
  #   ~$ sudo gem install fpm
  #   ~$ wget http://sourceforge.net/projects/arma/files/armadillo-4.000.4.tar.gz
  #   ~$ tar -zxf armadillo-4.000.4.tar.gz
  #   ~$ cd armadillo-4.000.4/
  #   ~/armadillo-4.000.4$ ./configure --prefix=/usr
  #   ~/armadillo-4.000.4$ make
  #   ~/armadillo-4.000.4$ mkdir -p tmp/build
  #   ~/armadillo-4.000.4$ make install DESTDIR=./tmp/build
  #   ~/armadillo-4.000.4$ fpm -s dir -t deb -n armadillo -v 4.000.4 \
  #                            -C ./tmp/build \
  #                            -d libblas-dev \
  #                            -d liblapack-dev \
  #                            usr/include usr/lib usr/share
  wget::fetch { 'armadillo.deb':
  	require    => [Package['libblas-dev'], Package['liblapack-dev']],
    destination => '/tmp/armadillo.deb',
    source      => 'https://dl.dropboxusercontent.com/u/27568639/armadillo_4.000.4_i386.deb',
  }
  -> package { '/tmp/armadillo.deb':
  	provider    => 'dpkg',
  }
}
