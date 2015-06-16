# == Definition: archive::download
#
# Archive downloader with integrity verification.
#
# Parameters:
#
# - *$url:
# - *$digest_url:
# - *$digest_string: Default value ''
# - *$digest_type: Default value 'md5'.
# - *$timeout: Default value 120.
# - *$src_target: Default value '/usr/src'.
# - *$allow_insecure: Default value false.
# - *$allow_redirects: Default value true
# - *$proxy: HTTP proxy in the form of "hostname:port"
# - *$exec_path: Path being searched for all Exec resources, default: ['/usr/local/bin', '/usr/bin', '/bin']
#
# Example usage:
#
#   archive::download {'apache-tomcat-6.0.26.tar.gz':
#     ensure => present,
#     url => 'http://archive.apache.org/dist/tomcat/tomcat-6/v6.0.26/bin/apache-tomcat-6.0.26.tar.gz',
#   }
#
#   archive::download {'apache-tomcat-6.0.26.tar.gz':
#     ensure => present,
#     digest_string => 'f9eafa9bfd620324d1270ae8f09a8c89',
#     url => 'http://archive.apache.org/dist/tomcat/tomcat-6/v6.0.26/bin/apache-tomcat-6.0.26.tar.gz',
#     proxy => 'myproxy.example.com:8080',
#   }
define archive::download (
  $url,
  $ensure          = present,
  $checksum        = true,
  $digest_url      = '',
  $digest_string   = '',
  $digest_type     = 'md5',
  $timeout         = 120,
  $src_target      = '/usr/src',
  $allow_insecure  = false,
  $allow_redirects = true,
  $username        = undef,
  $password        = undef,
  $proxy           = undef,
  $exec_path       = ['/usr/local/bin', '/usr/bin', '/bin']) {

  if ($username == undef and $password == undef) {
    $basic_auth = ''
  } else {
    $basic_auth = "--user ${username}:${password}"
  }

  if ($proxy == undef) {
    $proxy_arg = ''
  } else {
    $proxy_arg = "--proxy ${proxy}"
  }

  $insecure_arg = $allow_insecure ? {
    true    => '-k',
    default => '',
  }

  $redirects_arg = $allow_redirects ? {
    true    => '-L',
    default => '',
  }

  case $checksum {
    true : {
      case $digest_type {
        'md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512' : {
          $checksum_cmd = "${digest_type}sum -c ${name}.${digest_type}"
        }
        default: { fail('Unimplemented digest type') }
      }

      if $digest_url != '' and $digest_string != '' {
        fail('digest_url and digest_string should not be used together!')
      }

      if $digest_string == '' {

        case $ensure {
          present: {

            if $digest_url == '' {
              $digest_src = "${url}.${digest_type}"
            } else {
              $digest_src = $digest_url
            }

            exec {"download digest of archive ${name}":
              command => "curl ${basic_auth} ${insecure_arg} ${redirects_arg} ${proxy_arg} -s -o ${src_target}/${name}.${digest_type} ${digest_src}",
              path    => $exec_path,
              creates => "${src_target}/${name}.${digest_type}",
              timeout => $timeout,
              notify  => Exec["download archive ${name} and check sum"],
              require => Package['curl'],
            }

          }
          absent: {
            file{"${src_target}/${name}.${digest_type}":
              ensure => absent,
              purge  => true,
              force  => true,
            }
          }
          default: { fail("Unknown ensure value: '${ensure}'") }
        }
      }

      if $digest_string != '' {
        case $ensure {
          present: {
            file {"${src_target}/${name}.${digest_type}":
              ensure  => $ensure,
              content => "${digest_string} *${name}",
              notify  => Exec["download archive ${name} and check sum"],
            }
          }
          absent: {
            file {"${src_target}/${name}.${digest_type}":
              ensure => absent,
              purge  => true,
              force  => true,
            }
          }
          default: { fail("Unknown ensure value: '${ensure}'") }
        }
      }
    }
    false :  { notice('No checksum for this archive') }
    default: { fail("Unknown checksum value: '${checksum}'") }
  }

  case $ensure {
    present: {
      $notify = $checksum ? {
        true    => Exec["rm-on-error-${name}"],
        default => undef,
      }

      $refreshonly = $checksum ? {
        true    => true,
        default => undef,
      }

      exec {"download archive ${name} and check sum":
        command     => "curl ${basic_auth} -s ${insecure_arg} ${redirects_arg} ${proxy_arg} -o ${src_target}/${name} ${url}",
        path        => $exec_path,
        creates     => "${src_target}/${name}",
        logoutput   => true,
        timeout     => $timeout,
        require     => Package['curl'],
        notify      => $notify,
        refreshonly => $refreshonly,
      }

      exec {"rm-on-error-${name}":
        command     => "rm -f ${src_target}/${name} ${src_target}/${name}.${digest_type} && exit 1",
        path        => $exec_path,
        unless      => $checksum_cmd,
        cwd         => $src_target,
        refreshonly => true,
      }
    }
    absent: {
      file {"${src_target}/${name}":
        ensure => absent,
        purge  => true,
        force  => true,
      }
    }
    default: { fail("Unknown ensure value: '${ensure}'") }
  }
}
