Puppet Archive Module
=====================

[![Build Status](https://secure.travis-ci.org/gini/puppet-archive.png)](http://travis-ci.org/gini/puppet-archive)

Overview
--------

Puppet module to download and extract tar and zip archives based on [camptocamp/puppet-archive](https://github.com/camptocamp/puppet-archive).

Supported archive types are:

- `tar.gz`, `tgz`
- `tar.bz2`, `tbz2`
- `tar.xz`, `txz`
- `zip`


Usage
-----

Example:

    archive { 'apache-tomcat-6.0.26':
      ensure => present,
      url    => 'http://archive.apache.org/dist/tomcat/tomcat-6/v6.0.26/bin/apache-tomcat-6.0.26.tar.gz',
      target => '/opt',
    }


Supported Platforms
-------------------

The module has been tested on the following operating systems. Testing and patches for other platforms are welcome.

* Debian Linux 7.0 (Wheezy)


Support
-------

Please create bug reports and feature requests in [GitHub issues](https://github.com/gini/puppet-archive/issues).


License
-------

Puppet module originally from [camptocamp/puppet-archive](https://github.com/camptocamp/puppet-archive).

All changes copyright (c) 2012-2013 smarchive GmbH, 2013-2014 Gini GmbH

This script is licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0.html).


Contributors
------------

* Marc Remy (mremy)
* Marc Fournier (mfournier)
* Cedric Jeanneret (cjeanneret)
* Zijad Purkovic (zajk)
* Martin Konrad (mark0n)
* Brendan Murtagh (bmurt)
