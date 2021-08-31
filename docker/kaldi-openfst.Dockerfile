FROM nuhear:base

RUN install-package python2 subversion unzip sox gcc-fortran gcc-libs
RUN git clone https://aur.archlinux.org/kaldi-openfst.git /home/nubots/kaldi-openfst
WORKDIR /home/nubots/kaldi-openfst
# Set to a better(?) version number.
RUN sed '/^pkgver=/s/1.6.7/1.7.2/' -i PKGBUILD
RUN makepkg || true
