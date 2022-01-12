# Maintainer: Jingbei Li <i@jingbei.li>
pkgname=phonetisaurus
pkgver=0.9.1
pkgrel=1
pkgdesc="WFST-driven grapheme-to-phoneme (g2p) framework suitable for rapid development of high quality g2p or p2g systems."
arch=('i686' 'x86_64' 'arm' 'armv6h' 'armv7h')
url="https://github.com/AdolfVonKleist/Phonetisaurus"
license=('BSD')
depends=('python') # 'openfst' is installed manually.
makedepends=('python-setuptools' 'git') # 'python-pybindgen' is installed manually.
source=("https://github.com/AdolfVonKleist/Phonetisaurus/archive/refs/tags/0.9.1.tar.gz")
sha256sums=('SKIP')
provides=('phonetisaurus')
conflicts=('phonetisaurus')

prepare() {
        cd "$srcdir/Phonetisaurus-$pkgver"
        sed '41ausing namespace std;' -i src/include/util.h
        #sed '/MapToken/s/string&/std::string&/g' -i src/include/util.h
}

build() {
        cd "$srcdir/Phonetisaurus-$pkgver"
        PYTHON=/usr/bin/python ./configure \
                --prefix=/usr/local \
                --with-openfst-includes=/usr/local/include/fst \
                --with-openfst-libs=/usr/local/lib \
                --enable-python
        make
}

package() {
        cd "$srcdir/Phonetisaurus-$pkgver/python"
        cp ../.libs/Phonetisaurus.so .
        python setup.py install --root="$pkgdir"/ --optimize=1

        cd "$srcdir/Phonetisaurus-$pkgver"
        make DESTDIR=${pkgdir} install

        install -Dm644 LICENSE "${pkgdir}/usr/local/share/licenses/${pkgname}/LICENSE"
}
