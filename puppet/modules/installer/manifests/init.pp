
class installer::prerequisites (Hash $archs) {

  # Make the generic directories.
  file { [ '/nubots',
           '/nubots/toolchain',
           '/nubots/toolchain/bin',
           '/nubots/toolchain/include',
           '/nubots/toolchain/src' ]:
    ensure => directory,
  }

  # Make the architecture specific directories.
  $archs.each |String $arch, Hash $params| {
    file { [ "/nubots/toolchain/${arch}",
             "/nubots/toolchain/${arch}/bin",
             "/nubots/toolchain/${arch}/include",
             "/nubots/toolchain/${arch}/lib",
             "/nubots/toolchain/${arch}/man",
             "/nubots/toolchain/${arch}/share",
             "/nubots/toolchain/${arch}/src" ]:
      ensure => directory,
    }
  }
}

define installer (
  $url = '',
  $creates = '',
  $args = undef,
  $src_dir = '.',
  $method,
  $prefix = '/nubots/toolchain',
  $prebuild = 'echo', # Colon is a noop
  $postbuild = 'echo', # Colon is a noop
  $strip_components = 1,
  $extension = 'UNKNOWN',
  Hash $archs = $archs,
) {

  $archs.each |String $arch, Hash $params| {
    # Ensure we have a valid "creates" variable.
    if $creates == '' {
      $create = "${prefix}/${arch}/lib/lib${name}.a"
    }

    else {
      $create = "${prefix}/${arch}/${creates}"
    }

    # Get the environment.
    $compiler_flags  = $params['flags'].reduce |$compiler_flags, $value| { "${compiler_flags} ${value}" }
    $compiler_params = $params['params'].reduce |$compiler_params, $value| { "${compiler_params} ${value}" }
    $flags           = "${compiler_flags} ${compiler_params} -fPIC -I${prefix}/${arch}/include -Os"
    $libzlib         = "ZLIB_LIBPATH=${prefix}/${arch}/lib"
    $inczlib         = "ZLIB_INCLUDE=${prefix}/${arch}/include"
    $libbzip         = "BZIP2_LIBPATH=${prefix}/${arch}/lib"
    $incbzip         = "BZIP2_INCLUDE=${prefix}/${arch}/include"

    # "Intelligently" merge environment variables.
    if has_key($params['environment'], 'CFLAGS') {
      $cflags = "CFLAGS=${flags} ${params['environment']['CFLAGS']}"
      $env0 = delete($params['environment'], 'CFLAGS')
    }

    else {
      $cflags = "CFLAGS=${flags}"
      $env0 = $params['environment']
    }

    if has_key($params['environment'], 'CXXFLAGS') {
      $cxxflags = "CXXFLAGS=${flags} ${params['environment']['CXXFLAGS']}"
      $env1 = delete($env0, 'CXXFLAGS')
    }

    else {
      $cxxflags = "CXXFLAGS=${flags}"
      $env1 = $env0
    }

    if has_key($params['environment'], 'COMMON_OPT') {
      $common_opt = "COMMON_OPT=${flags} ${params['environment']['COMMON_OPT']}"
      $env2 = delete($env1, 'COMMON_OPT')
    }

    else {
      $common_opt = "COMMON_OPT=${flags}"
      $env2 = $env1
    }

    if has_key($params['environment'], 'FCOMMON_OPT') {
      $fcommon_opt = "FCOMMON_OPT=${flags} ${params['environment']['FCOMMON_OPT']}"
      $env3 = delete($env2, 'FCOMMON_OPT')
    }

    else {
      $fcommon_opt = "FCOMMON_OPT=${flags}"
      $env3 = $env2
    }

    if has_key($params['environment'], 'LDFLAGS') {
      $linkflags  = "-L${prefix}/${arch}/lib ${params['environment']['LDFLAGS']}"
      $env4 = delete($env3, 'LDFLAGS')
    }

    else {
      $linkflags = "-L${prefix}/${arch}/lib"
      $env4 = $env3
    }

    $ldflags = "LDFLAGS=${linkflags}"

    if has_key($params['environment'], 'CCASFLAGS') {
      $ccasflags = "CCASFLAGS=${params['environment']['CCASFLAGS']}"
      $env5 = delete($env4, 'CCASFLAGS')
    }

    else {
      $ccasflags = "CCASFLAGS=-f elf"
      $env5 = $env4
    }

    if has_key($params['environment'], 'PKG_CONFIG_PATH') {
      $pkgconfig = "PKG_CONFIG_PATH=${params['environment']['PKG_CONFIG_PATH']}"
      $env6 = delete($env5, 'PKG_CONFIG_PATH')
    }

    else {
      $pkgconfig = "PKG_CONFIG_PATH=${prefix}/${arch}/lib/pkgconfig"
      $env6 = $env5
    }

    if has_key($params['environment'], 'CMAKE_PREFIX_PATH') {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${params['environment']['CMAKE_PREFIX_PATH']}"
      $env7 = delete($env6, 'CMAKE_PREFIX_PATH')
    }

    else {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${prefix}/${arch}"
      $env7 = $env6
    }

    if has_key($params['environment'], 'LT_SYS_LIBRARY_PATH') {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${params['environment']['LT_SYS_LIBRARY_PATH']}"
      $env8 = delete($env7, 'LT_SYS_LIBRARY_PATH')
    }

    else {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${prefix}/${arch}/lib"
      $env8 = $env7
    }

    if has_key($params['environment'], 'LD_LIBRARY_PATH') {
      $ldlibrarypath = "LD_LIBRARY_PATH=${params['environment']['LD_LIBRARY_PATH']}"
      $env9 = delete($env8, 'LD_LIBRARY_PATH')
    }

    else {
      $ldlibrarypath = "LD_LIBRARY_PATH=${prefix}/${arch}/lib"
      $env9 = $env8
    }

    $environment = [$cflags, $cxxflags, $common_opt, $fcommon_opt,
                    $ldflags, $ccasflags, $pkgconfig, $cmake_prefix,
                    $ltsyslibpath, $ldlibrarypath,
                    $libzlib, $inczlib, $libbzip, $incbzip, ] + join_keys_to_values($env9, "=")

    # Reduce the args array to a space separated list of arguments.
    if $args {
      $arg = regsubst($args["${arch}"], 'ZLIB_PATH', "${prefix}/${arch}")
      $arg1 = regsubst($arg, 'PREFIX', "${prefix}/${arch}")
      $args_str = $arg1.reduce |$args_str, $value| { "${args_str} ${value}" }
    }

    case $extension {
      'h', 'hpp': {
        # Download and put in include
        $basename = basename($url)

        wget::fetch { "${arch}_${name}":
          destination => "${prefix}/${arch}/include/${basename}",
          source      => $url,
          unless      => "test -e ${prefix}/${arch}/include/${basename}",
        }
      }
      default: {
        # Sync extracted archive to local arch directory, then build and install.
        # NOTE: Done as an exec to suppress to boat-loads of output that 'file' generates for recursive copies.
        exec { "mirror_${arch}_${name}":
          command     => "rsync -a ${prefix}/src/${name} .",
          cwd         => "${prefix}/${arch}/src",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
        }

        exec { "autotools_${arch}_${name}":
          creates     => "${create}",
          onlyif      => "test \"${method}\" = \"autotools\" ",
          command     => "${prebuild} &&
                          if [ -e \"autogen.sh\" ]; then ./autogen.sh ; fi &&
                          ./configure ${args_str} --prefix=\"${prefix}/${arch}\" &&
                          make -j\$(nproc) &&
                          make install &&
                          ${postbuild}",
          cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout     => 0,
          provider    => 'shell',
          require     => [ Exec["mirror_${arch}_${name}"], ],
        }

        exec { "cmake_${arch}_${name}":
          creates     => "${create}",
          onlyif      => "test \"${method}\" = \"cmake\" ",
          command     => "${prebuild} &&
                          if [ -d \"build\" ]; then rm -rf build; fi &&
                          mkdir build ; cd build &&
                          cmake .. ${args_str} -DCMAKE_BUILD_TYPE=\"Release\" -DCMAKE_C_FLAGS_RELEASE=\"${flags}\" -DCMAKE_CXX_FLAGS_RELEASE=\"${flags}\" -DCMAKE_INSTALL_PREFIX:PATH=\"${prefix}/${arch}\" &&
                          make -j\$(nproc) &&
                          make install &&
                          ${postbuild}",
          cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout     => 0,
          provider    => 'shell',
          require     => [ Exec["mirror_${arch}_${name}"], ],
        }

        exec { "boost_${arch}_${name}":
          creates     => "${create}",
          onlyif      => "test \"${method}\" = \"boost\" ",
          command     => "${prebuild} &&
                          ./bootstrap.sh --prefix=\"${prefix}/${arch}\" --without-libraries=python &&
                          ./bjam include=\"${prefix}/${arch}/include\" library-path=\"${prefix}/${arch}/lib\" ${args_str} -j\$(nproc) -q \\
                                cflags=\"${flags}\" cxxflags=\"${flags}\" linkflags=\"${linkflags}\" &&
                          ./bjam install &&
                          ${postbuild}",
          cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout     => 0,
          provider    => 'shell',
          require     => [ Exec["mirror_${arch}_${name}"], ],
        }

        exec { "make_${arch}_${name}":
          creates     => "${create}",
          onlyif      => "test \"${method}\" = \"make\" ",
          command     => "${prebuild} &&
                          make ${args_str} -j\$(nproc) &&
                          make ${args_str} PREFIX=\"${prefix}/${arch}\" install &&
                          ${postbuild}",
          cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout     => 0,
          provider    => 'shell',
          require     => [ Exec["mirror_${arch}_${name}"], ],
        }
      }
    }
  }
}
