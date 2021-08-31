FROM nuhear:base

RUN git clone https://aur.archlinux.org/kaldi.git /home/nubots/kaldi
WORKDIR /home/nubots/kaldi
RUN install-package lapack python2 cblas
RUN sed '/^pkgver=/s/5.5.r9114.d6198906f/5.5.r9187.d53b62fd4/' -i PKGBUILD
RUN makepkg || true
