#!/bin/bash
cd /var/tmp
version="0.65.4"
wget "https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-$version.tar.gz" -O "quex-$version.tar.gz"
# wget "https://downloads.sourceforge.net/project/quex/HISTORY/$version/quex-$version.tar.gz" -O "quex-$version.tar.gz"
tar -zxf "quex-$version.tar.gz"
mv "quex-$version" /usr/local/etc/quex
ln -s /usr/local/etc/quex/quex /usr/local/include/quex
echo '#!/bin/bash' > /usr/local/bin/quex
echo 'QUEX_PATH=/usr/local/etc/quex python /usr/local/etc/quex/quex-exe.py "$@"' >> /usr/local/bin/quex
chmod +x /usr/local/bin/quex
