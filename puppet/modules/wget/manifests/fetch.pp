################################################################################
# Definition: wget::fetch
#
# This class will download files from the internet.  You may define a web proxy
# using $http_proxy if necessary.
#
################################################################################
define wget::fetch (
  $destination,
  $source             = $title,
  $timeout            = '0',
  $verbose            = false,
  $redownload         = false,
  $nocheckcertificate = false,
  $execuser           = 'root',
) {

  include wget

  # using "unless" with test instead of "creates" to re-attempt download
  # on empty files.
  # wget creates an empty file when a download fails, and then it wouldn't try
  # again to download the file
  if $::http_proxy {
    $environment = [ "HTTP_PROXY=${::http_proxy}", "http_proxy=${::http_proxy}" ]
  } else {
    $environment = []
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

  exec { "wget-${name}":
    command     => "wget ${verbose_option}${nocheckcert_option} --output-document='${destination}' '${source}'",
    timeout     => $timeout,
    unless      => $unless_test,
    environment => $environment,
    user        => $execuser,
    path        => '/usr/bin:/usr/sbin:/bin:/usr/local/bin:/opt/local/bin',
    require     => Class['wget'],
  }
}
