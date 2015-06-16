# == Definition: archive
#
# Download and extract an archive.
#
# Parameters:
#
# - *$url:
# - *$target: Destination directory
# - *$checksum: Default value "true"
# - *$digest_url: Default value ""
# - *$digest_string: Default value ""
# - *$digest_type: Default value "md5"
# - *$src_target: Default value "/usr/src"
# - *$root_dir: Default value ""
# - *$extension: Default value ".tar.gz"
# - *$timeout: Default value 120
# - *$allow_insecure: Default value false
# - *$allow_redirects: Default value true
# - *$strip_components: Strip the top most n directories from each file name; default value 0
# - *$username: set basic auth username
# - *$password: set basic auth password
# - *$proxy: HTTP proxy in the form of "hostname:port"; e.g. "myproxy:8080"
# - *$dependency_class: Puppet class which installs the required programs (curl, tar, unzip)
# - *$exec_path: Path being searched for all Exec resources, default: ['/usr/local/bin', '/usr/bin', '/bin']
#
# Example usage:
#
#   archive {"apache-tomcat-6.0.26":
#     ensure => present,
#     url => "http://archive.apache.org/dist/tomcat/tomcat-6/v6.0.26/bin/apache-tomcat-6.0.26.tar.gz",
#     target => "/opt",
#   }
#
#   archive {"apache-tomcat-6.0.26":
#     ensure   => present,
#     url      => "http://archive.apache.org/dist/tomcat/tomcat-6/v6.0.26/bin/apache-tomcat-6.0.26.tar.gz",
#     username => "example",
#     password => "example",
#     target   => "/opt",
#   }
define archive (
  $url,
  $target,
  $ensure           = present,
  $checksum         = true,
  $digest_url       = '',
  $digest_string    = '',
  $digest_type      = 'md5',
  $timeout          = 120,
  $root_dir         = '',
  $extension        = 'tar.gz',
  $src_target       = '/usr/src',
  $allow_insecure   = false,
  $allow_redirects  = true,
  $strip_components = 0,
  $username         = undef,
  $password         = undef,
  $proxy            = undef,
  $dependency_class = Class['archive::prerequisites'],
  $exec_path        = ['/usr/local/bin', '/usr/bin', '/bin']) {

  archive::download {"${name}.${extension}":
    ensure          => $ensure,
    url             => $url,
    checksum        => $checksum,
    digest_url      => $digest_url,
    digest_string   => $digest_string,
    digest_type     => $digest_type,
    timeout         => $timeout,
    src_target      => $src_target,
    allow_insecure  => $allow_insecure,
    allow_redirects => $allow_redirects,
    username        => $username,
    password        => $password,
    proxy           => $proxy,
    require         => $dependency_class,
    exec_path       => $exec_path,
  }

  archive::extract { $name:
    ensure           => $ensure,
    target           => $target,
    src_target       => $src_target,
    root_dir         => $root_dir,
    extension        => $extension,
    timeout          => $timeout,
    strip_components => $strip_components,
    exec_path        => $exec_path,
    require          => Archive::Download["${name}.${extension}"]
  }
}
