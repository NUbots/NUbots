FROM nuhear:base AS openfst

RUN git clone https://aur.archlinux.org/openfst.git /home/nubots/openfst
WORKDIR /home/nubots/openfst
# Enable configure build options for openfst
RUN sed "34a \ \ OPTIONS+=' --enable-static'" -i PKGBUILD
RUN sed "35a \ \ OPTIONS+=' --enable-shared'" -i PKGBUILD
RUN makepkg || true

FROM nuhear:base AS kaldi-openfst

RUN install-package python2 subversion unzip sox gcc-fortran gcc-libs
RUN git clone https://aur.archlinux.org/kaldi-openfst.git /home/nubots/kaldi-openfst
WORKDIR /home/nubots/kaldi-openfst
# Set to a better(?) version number.
RUN sed '/^pkgver=/s/1.6.7/1.7.2/' -i PKGBUILD
RUN makepkg || true

FROM nuhear:base AS kaldi

RUN git clone https://aur.archlinux.org/kaldi.git /home/nubots/kaldi
WORKDIR /home/nubots/kaldi
RUN install-package lapack python2 cblas
RUN sed '/^pkgver=/s/5.5.r9114.d6198906f/5.5.r9187.d53b62fd4/' -i PKGBUILD
COPY --from=kaldi-openfst /home/nubots/kaldi-openfst/kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst /home/nubots/kaldi/
RUN sudo pacman -U --noconfirm kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst
RUN makepkg || true

FROM nuhear:base AS ngram

RUN git clone
WORKDIR /home/nubots/opengrm-ngram

RUN git clone https://aur.archlinux.org/opengrm-ngram.git /home/nubots/opengrm-ngram
WORKDIR /home/nubots/opengrm-ngram
COPY --from=openfst /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/opengrm-ngram
RUN sudo pacman -U --noconfirm openfst-1.7.9-1-x86_64.pkg.tar.zst

# Set the package name because it changed recently at opengrm.org
RUN sed '/^pkgname/s/=.*/=ngram/' -i PKGBUILD
# Set version number.
RUN sed '/^pkgver/s/=.*/=1.3.11/' -i PKGBUILD
# Set source url. The old one didn't exist.
RUN sed '/^source/s/=.*/=("http:\/\/opengrm.org\/twiki\/pub\/GRM\/NGramDownload\/${pkgname}-${pkgver}.tar.gz")/' -i PKGBUILD
# Set the hash value. This value wasn't obtained securely (i.e. I downloaded insecure file then ran sha256sum)
RUN sed "/^md5sums=/s/.*/sha256sums=('05d417b57751dc7eca43b13d5f50eebe9d2338851bc47a854bb9318b91649773')/" -i PKGBUILD
RUN makepkg || true
