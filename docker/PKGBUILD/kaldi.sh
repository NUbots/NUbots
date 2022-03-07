# See the Arch User Repository (AUR) for the original source of these scripts.
pkgname='kaldi'
pkgdesc='Speech recognition research toolkit'
pkgver=5.5.r9195.3b8a97d85
pkgrel=1
depends=('cblas' 'lapack' 'python2') # 'openfst' is installed manually and not checked here because it would cause the dependency check to fail.
optdepends=('cuda' 'kaldi-irstlm' 'kaldi-kaldi_lm' 'kaldi-sctk' 'kaldi-sph2pipe' 'kaldi-srilm')
makedepends=('git' 'wget' 'sed')
arch=('x86_64' 'i686')
url='https://github.com/kaldi-asr/kaldi'
license=('APACHE')
source=("git+${url}")
sha256sums=('SKIP')

prepare() {
    cd $srcdir/$pkgname
    # Use this specific version instead of build from whatever is in the master branch.
    git checkout 3b8a97d859464912ddc50877f7280048f654c275 &> /dev/null

    find . -name '*.py' -exec sed '1s/python/python2/' -i {} \;


    # Removing static libs in https://github.com/kaldi-asr/kaldi/blob/8f94bd0698d503c5c18e02f8fd4209b6802ff17a/src/makefiles/linux_clapack.mk#L16
    sed '/^LDLIBS = /s/\$(CLAPACKLIBS)//' -i src/makefiles/linux_clapack.mk
    # Replace -isystem with -I because including system headers was broken: https://github.com/kaldi-asr/kaldi/blob/8f94bd0698d503c5c18e02f8fd4209b6802ff17a/src/makefiles/linux_clapack.mk#L20
    # See also this article explaining the difference: https://stackoverflow.com/questions/2579576/i-dir-vs-isystem-dir#comment2590674_2579576
    sed '/-isystem/s//-I/' -i src/makefiles/linux_clapack.mk
}

build() {
    if (false && pacman -Q cuda &> /dev/null); then
        msg2 "Compiling with CUDA support"
        _cuda_config_opts="--cudatk-dir=/opt/cuda"
    else
        msg2 "Compiling without CUDA support"
        _cuda_config_opts="--use-cuda=no"
    fi

    cd $srcdir/$pkgname/src
    LDFLAGS='-lcblas -llapack' \
    ./configure $_cuda_config_opts \
        --shared \
        --fst-root=/usr/local \
        --fst-version=1.7.2 \
        --clapack-root=/usr
    make -j 4 depend
    make -j 4
}

package() {
    echo "Packaging..."

    # Create the library directory.
    mkdir --parents $pkgdir/usr/local/lib/
    # Copy kaldi libraries into it, always following symbolic links.
    cp --dereference --target-directory=$pkgdir/usr/local/lib $srcdir/$pkgname/src/lib/*.so

    local kaldi=$pkgdir/usr/local/$pkgname

    mkdir --parents $kaldi
    cp --recursive --target-directory=$kaldi $srcdir/$pkgname/src $srcdir/$pkgname/egs $srcdir/$pkgname/tools

    rm --force $kaldi/src/*/*.{cc,cu,o,a,orig}
    rm --recursive $kaldi/src/{doc,feat/test_data,lm/examples,lm/test_data,makefiles,onlinebin,probe}
    find $kaldi/src \( \
        -name 'Makefile*' \
        -or -name 'README' \
        -or -name 'CMake*' \
        -or -name '*.mk' \
        -not -name 'kaldi.mk'\
        \) -exec rm {} \;
    find $kaldi/src -maxdepth 1 -type f -not -name 'kaldi.mk' -exec rm {} \;
    rm --recursive $kaldi/tools/{ATLAS_headers,CLAPACK,INSTALL,Makefile}

    # Some python scripts have the wrong program in the shebang line.
    sed "s|#!/usr/bin/env python23|#!/usr/bin/env python3|g" -i `grep python23 "$kaldi" -rIl`

    # Some scripts hardcode the path that we are building in. Ops...
    sed "s|$kaldi|/usr/local|g" -i `grep --fixed-strings --recursive --binary-files=without-match --files-with-matches "$srcdir" "$kaldi"`

    find $kaldi -name 'path.sh' -exec sed 's?^\(export KALDI_ROOT\)=.*$?\1=/usr/local/'$pkgname'?' -i {} \;
    # echo "export OPENFST=$(find /opt/$pkgname/tools -type d -name 'openfst*')" >> tools/env.sh
    echo "export OPENFST=" >> $kaldi/tools/env.sh
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:${OPENFST}/lib' >> $kaldi/tools/env.sh
    echo "export IRSTLM=/usr/local/$pkgname/tools/irstlm" >> $kaldi/tools/env.sh
    echo 'export PATH=${PATH}:${IRSTLM}/bin' >> $kaldi/tools/env.sh
}
