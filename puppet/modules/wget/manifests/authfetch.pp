################################################################################
# Definition: wget::authfetch
#
# This class will download files from the internet.  You may define a web proxy
# using $::http_proxy if necessary. Username must be provided. And the user's
# password must be stored in the password variable within the .wgetrc file.
#
################################################################################
define wget::authfetch (
  $destination,
  $user,
  $source             = $title,
  $password           = '',
  $timeout            = '0',
  $verbose            = false,
  $redownload         = false,
  $nocheckcertificate = false,
  $execuser           = 'root',
) {

  include wget

  if $::http_proxy {
    $environment = [ "HTTP_PROXY=${::http_proxy}", "http_proxy=${::http_proxy}", "WGETRC=/tmp/wgetrc-${name}" ]
  }
  else {
    $environment = [ "WGETRC=/tmp/wgetrc-${name}" ]
  }

  $verbose_option = $verbose ? {
    true  => '--verbose',
    false => '--no-verbose'
  }

  $unless_test = $redownload ? {
    true  => 'test',
    false => "test -s ${destination}"
  }

  $nocheckcert_option = $nocheckcertificate ? {
    true  => ' --no-check-certificate',
    false => ''
  }

  case $::operatingsystem {
    'Darwin': {
      # This is to work around an issue with macports wget and out of date CA cert bundle.  This requires
      # installing the curl-ca-bundle package like so:
      #
      # sudo port install curl-ca-bundle
      $wgetrc_content = "password=${password}\nCA_CERTIFICATE=/opt/local/share/curl/curl-ca-bundle.crt\n"
    }
    default: {
      $wgetrc_content = "password=${password}"
    }
  }

  file { "/tmp/wgetrc-${name}":
    owner   => 'root',
    mode    => '0600',
    content => $wgetrc_content,
  } ->
  exec { "wget-${name}":
    command     => "wget ${verbose_option}${nocheckcert_option} --user=${user} --output-document='${destination}' '${source}'",
    timeout     => $timeout,
    unless      => $unless_test,
    environment => $environment,
    user        => $execuser,
    path        => '/usr/bin:/usr/sbin:/bin:/usr/local/bin:/opt/local/bin',
    require     => Class['wget'],
  }
}
