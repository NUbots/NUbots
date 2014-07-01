#!/bin/bash
cd /tmp
wget https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-0.64.8.tar.gz -O quex-0.64.8.tar.gz
tar -zxf quex-0.64.8.tar.gz
mv quex-0.64.8/ /usr/local/etc/quex
ln -s /usr/local/etc/quex/quex/ /usr/local/include/quex
echo '#!/bin/bash' >> /usr/local/bin/quex
echo 'QUEX_PATH=/usr/local/etc/quex python /usr/local/etc/quex/quex-exe.py "$@"' >> /usr/local/bin/quex
chmod +x /usr/local/bin/quex