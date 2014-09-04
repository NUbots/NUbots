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

  # The following script attempts to automate the process of .deb creation
/*
arma_version=4.100.2
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libffi-dev
sudo gem install fpm
wget http://sourceforge.net/projects/arma/files/armadillo-${arma_version}.tar.gz
tar -zxf armadillo-${arma_version}.tar.gz
cd armadillo-${arma_version}/
./configure --prefix=/usr
make
mkdir -p tmp/build
make install DESTDIR=./tmp/build
fpm -s dir -t deb -n armadillo -v ${arma_version} \
                         -C ./tmp/build \
                         -d libblas-dev \
                         -d liblapack-dev \
                         usr/include usr/lib usr/share

*/
  $arma_pkg = 'armadillo_4.100.2_i386.deb'

  wget::fetch { "${arma_pkg}":
  	require    => [Package['libblas-dev'], Package['liblapack-dev']],
    destination => "/tmp/${arma_pkg}",
    source      => "https://dl.dropboxusercontent.com/u/27568639/${arma_pkg}",
  }
  ~> package { 'armadillo':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/tmp/${arma_pkg}"
  }
}

