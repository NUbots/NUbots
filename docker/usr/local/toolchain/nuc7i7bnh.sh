#!/bin/sh

# Set our compilers
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

# Set our package config so it finds things in the toolchain
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

# Set our optimisation flags
export CFLAGS="-march=broadwell -mtune=broadwell -mmmx -mno-3dnow -msse -msse2 -msse3 -mssse3 -mno-sse4a -mcx16 -msahf -mmovbe -maes -mno-sha -mpclmul -mpopcnt -mabm -mno-lwp -mfma -mno-fma4 -mno-xop -mbmi -mbmi2 -mno-tbm -mavx -mavx2 -msse4.2 -msse4.1 -mlzcnt -mno-rtm -mno-hle -mrdrnd -mf16c -mfsgsbase -mrdseed -mprfchw -madx -mfxsr -mxsave -mxsaveopt -mno-avx512f -mno-avx512er -mno-avx512cd -mno-avx512pf -mno-prefetchwt1 -mclflushopt -mxsavec -mxsaves -mno-avx512dq -mno-avx512bw -mno-avx512vl -mno-avx512ifma -mno-avx512vbmi -mno-clwb -mno-mwaitx --param l1-cache-size=32 --param l1-cache-line-size=64 --param l2-cache-size=4096"
export CXXFLAGS="${CFLAGS}"
export CPPFLAGS="${CFLAGS}"
