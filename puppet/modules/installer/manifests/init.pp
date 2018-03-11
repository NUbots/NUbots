
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
  $environment = undef,
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

    # Merge the provided environment with the archs environment.
    # Duplicates are replaced with the value from the provided environment.
    if $environment {
      $env = deep_merge($params['environment'], $environment)
    }

    else {
      $env = $params['environment']
    }

    # "Intelligently" merge environment variables.
    if has_key($env, 'CFLAGS') {
      $cflags = "CFLAGS=${flags} ${env['CFLAGS']}"
      $env0 = delete($env, 'CFLAGS')
    }

    else {
      $cflags = "CFLAGS=${flags}"
      $env0 = $env
    }

    if has_key($env, 'CXXFLAGS') {
      $cxxflags = "CXXFLAGS=${flags} ${env['CXXFLAGS']}"
      $env1 = delete($env0, 'CXXFLAGS')
    }

    else {
      $cxxflags = "CXXFLAGS=${flags}"
      $env1 = $env0
    }

    if has_key($env, 'COMMON_OPT') {
      $common_opt = "COMMON_OPT=${flags} ${env['COMMON_OPT']}"
      $env2 = delete($env1, 'COMMON_OPT')
    }

    else {
      $common_opt = "COMMON_OPT=${flags}"
      $env2 = $env1
    }

    if has_key($env, 'FCOMMON_OPT') {
      $fcommon_opt = "FCOMMON_OPT=${flags} ${env['FCOMMON_OPT']}"
      $env3 = delete($env2, 'FCOMMON_OPT')
    }

    else {
      $fcommon_opt = "FCOMMON_OPT=${flags}"
      $env3 = $env2
    }

    if has_key($env, 'LDFLAGS') {
      $linkflags  = "-L${prefix}/${arch}/lib -L${prefix}/lib ${env['LDFLAGS']}"
      $env4 = delete($env3, 'LDFLAGS')
    }

    else {
      $linkflags = "-L${prefix}/${arch}/lib -L${prefix}/lib"
      $env4 = $env3
    }

    $ldflags = "LDFLAGS=${linkflags}"

    if has_key($env, 'CCASFLAGS') {
      $ccasflags = "CCASFLAGS=${env['CCASFLAGS']}"
      $env5 = delete($env4, 'CCASFLAGS')
    }

    else {
      $ccasflags = "CCASFLAGS=-f elf"
      $env5 = $env4
    }

    if has_key($env, 'PKG_CONFIG_PATH') {
      $pkgconfig = "PKG_CONFIG_PATH=${env['PKG_CONFIG_PATH']}:${prefix}/${arch}/lib/pkgconfig"
      $env6 = delete($env5, 'PKG_CONFIG_PATH')
    }

    else {
      $pkgconfig = "PKG_CONFIG_PATH=${prefix}/${arch}/lib/pkgconfig"
      $env6 = $env5
    }

    if has_key($env, 'CMAKE_PREFIX_PATH') {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${env['CMAKE_PREFIX_PATH']}"
      $env7 = delete($env6, 'CMAKE_PREFIX_PATH')
    }

    else {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${prefix}/${arch}"
      $env7 = $env6
    }

    if has_key($env, 'LT_SYS_LIBRARY_PATH') {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${env['LT_SYS_LIBRARY_PATH']}"
      $env8 = delete($env7, 'LT_SYS_LIBRARY_PATH')
    }

    else {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${prefix}/${arch}/lib:${prefix}/lib"
      $env8 = $env7
    }

    if has_key($env, 'LD_LIBRARY_PATH') {
      $ldlibrarypath = "LD_LIBRARY_PATH=${env['LD_LIBRARY_PATH']}"
      $env9 = delete($env8, 'LD_LIBRARY_PATH')
    }

    else {
      $ldlibrarypath = "LD_LIBRARY_PATH=${prefix}/${arch}/lib:${prefix}/lib"
      $env9 = $env8
    }

    if has_key($env, 'CCAS') {
      $ccas = "CCAS=${env['CCAS']}"
      $env10 = delete($env9, 'CCAS')
    }

    else {
      $ccas = "CCAS=as"
      $env10 = $env9
    }

    $environment = ['CC=/usr/bin/gcc', 'CXX=/usr/bin/g++', $cflags, $cxxflags, $common_opt, $fcommon_opt,
                    $ldflags, $ccas, $ccasflags, $pkgconfig, $cmake_prefix,
                    $ltsyslibpath, $ldlibrarypath,
                    $libzlib, $inczlib, $libbzip, $incbzip, ] + join_keys_to_values($env10, "=")

    # Reduce the args array to a space separated list of arguments.
    if $args {
      $arg      = regsubst($args["${arch}"], 'ZLIB_PATH', "${prefix}/${arch}", 'G')
      $arg1     = regsubst($arg, 'PREFIX', "${prefix}/${arch}", 'G')
      $arg2     = regsubst($arg1, 'PROTOC_PATH', "${prefix}/bin/protoc", 'G')
      $args_str = $arg2.reduce |$args_str, $value| { "${args_str} ${value}" }
    }

    $prebuild_cmd = regsubst($prebuild, 'PREFIX', "${prefix}/${arch}", 'G')
    $postbuild_cmd = regsubst($postbuild, 'PREFIX', "${prefix}/${arch}", 'G')

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
          # Sometimes, autogen.sh will automatically run configure
          command     => "${prebuild_cmd} &&
                          if [ -e \"autogen.sh\" ]; then NOCONFIGURE=1 ./autogen.sh; fi &&
                          ./configure ${args_str} --prefix=\"${prefix}/${arch}\" &&
                          make -j\$(nproc) &&
                          make install &&
                          ${postbuild_cmd}",
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
          command     => "${prebuild_cmd} &&
                          if [ -d \"build\" ]; then rm -rf build; fi &&
                          mkdir build ; cd build &&
                          cmake .. ${args_str} -DCMAKE_BUILD_TYPE=\"Release\" -DCMAKE_C_FLAGS_RELEASE=\"${flags}\" -DCMAKE_CXX_FLAGS_RELEASE=\"${flags}\" -DCMAKE_INSTALL_PREFIX:PATH=\"${prefix}/${arch}\" &&
                          make -j\$(nproc) &&
                          make install &&
                          ${postbuild_cmd}",
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
          command     => "${prebuild_cmd} &&
                          ./bootstrap.sh --prefix=\"${prefix}/${arch}\" --without-libraries=python &&
                          ./bjam include=\"${prefix}/${arch}/include\" library-path=\"${prefix}/${arch}/lib\" ${args_str} -j\$(nproc) -q \\
                                cflags=\"${flags}\" cxxflags=\"${flags}\" linkflags=\"${linkflags}\" &&
                          ./bjam install &&
                          ${postbuild_cmd}",
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
          command     => "${prebuild_cmd} &&
                          make ${args_str} -j\$(nproc) &&
                          make ${args_str} PREFIX=\"${prefix}/${arch}\" install &&
                          ${postbuild_cmd}",
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
