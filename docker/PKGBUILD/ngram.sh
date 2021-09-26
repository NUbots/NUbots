# Maintainer: Benoit Favre <benoit.favre@gmail.com>

pkgname=ngram
pkgver=1.3.7
pkgrel=1
pkgdesc="OpenGrm tool for making n-gram language models as weighted finite-state transducers"
arch=('i686' 'x86_64')
url="http://www.opengrm.org/"
license=('APACHE')
depends=('openfst')
source=("http://opengrm.org/twiki/pub/GRM/NGramDownload/${pkgname}-${pkgver}.tar.gz")
sha256sums=('SKIP')


build() {
    cd ${srcdir}/${pkgname}-${pkgver}
    ./configure --prefix=/usr LDFLAGS=-L/usr/lib/fst
    make
}

package() {
    cd ${srcdir}/${pkgname}-${pkgver}
    make DESTDIR=${pkgdir} install
}
