#!/bin/sh

function purge_folder {
  if [ -d $1 ];
  then
    rm -rf $1
  fi
  mkdir $1
}
​
function download_and_extract {
  wget $1 -O $(basename $1)
  purge_folder $2
  tar xf $(basename $1) -C $2 --strip-components 1
}
​
function make_build {
  make -j$(nproc) "$@"
  sudo make install
}
​
function autotools_build {
  PKG_CONFIG_PATH="${PKG_CFG_PATH}" \
    LDFLAGS="${LINK_FLAGS}" \
    CFLAGS="${C_FLAGS}" \
    CXXFLAGS="${CXX_FLAGS}" \
    ./configure --prefix="${PREFIX}" "$@"
  make -j$(nproc)
  sudo make install
}
​
function cmake_build {
  purge_folder build
  cd build
  PKG_CONFIG_PATH="${PKG_CFG_PATH}" \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
          -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON \
          -DCMAKE_PREFIX_PATH="${PREFIX};${PREFIX}/lib/pkgconfig" \
          "$@" \
          ../ -G Ninja
    ninja -j2
    sudo ninja install
    cd ..
}
