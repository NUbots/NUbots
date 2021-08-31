# syntax = docker/dockerfile:1.3-labs
FROM nuhear:base AS openfst

RUN git clone https://aur.archlinux.org/openfst.git /home/nubots/openfst
WORKDIR /home/nubots/openfst
# Enable configure build options for openfst
RUN sed "34a \ \ OPTIONS+=' --enable-static'" -i PKGBUILD
RUN sed "35a \ \ OPTIONS+=' --enable-shared'" -i PKGBUILD
RUN makepkg || true
RUN test -f openfst-1.7.9-1-x86_64.pkg.tar.zst

FROM nuhear:base AS kaldi-openfst

RUN install-package python2 subversion unzip sox gcc-fortran gcc-libs
RUN git clone https://aur.archlinux.org/kaldi-openfst.git /home/nubots/kaldi-openfst
WORKDIR /home/nubots/kaldi-openfst
# Set to a better(?) version number.
RUN sed '/^pkgver=/s/1.6.7/1.7.2/' -i PKGBUILD
RUN makepkg || true
RUN test -f kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst

FROM nuhear:base AS kaldi

RUN git clone https://aur.archlinux.org/kaldi.git /home/nubots/kaldi
WORKDIR /home/nubots/kaldi
RUN install-package lapack python2 cblas
RUN sed '/^pkgver=/s/5.5.r9114.d6198906f/5.5.r9187.d53b62fd4/' -i PKGBUILD
COPY --from=kaldi-openfst /home/nubots/kaldi-openfst/kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst /home/nubots/kaldi/
RUN sudo pacman -U --noconfirm kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst
RUN makepkg || true
RUN test -f kaldi-5.5.r9187.d53b62fd4-1-x86_64.pkg.tar.zst

FROM nuhear:base AS ngram

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
RUN test -f ngram-1.3.11-1-x86_64.pkg.tar.zst

FROM nuhear:base AS phonetisaurus

RUN git clone https://aur.archlinux.org/python-pybindgen.git /home/nubots/python-pybindgen
WORKDIR /home/nubots/python-pybindgen
RUN makepkg
RUN sudo pacman -U --noconfirm python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst

RUN git clone https://aur.archlinux.org/phonetisaurus-git.git /home/nubots/phonetisaurus-git
WORKDIR /home/nubots/phonetisaurus-git
COPY --from=openfst /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/phonetisaurus-git/
RUN sudo pacman -U --noconfirm openfst-1.7.9-1-x86_64.pkg.tar.zst

# Set the version number.
RUN sed -i PKGBUILD -e '/^pkgver/s/=.*$/=r171.23f182f/'
# Add configuration options to ./configure
RUN sed -i PKGBUILD -e '30a\\ \t\t--with-openfst-includes=/usr/include/fst \\'
RUN sed -i PKGBUILD -e '31a\\ \t\t--with-openfst-libs=/usr/lib \\'
RUN makepkg || true
RUN test -f phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst

FROM nuhear:base as pocketsphinx

RUN install-package bison swig python2 lapack libpulse gst-plugins-base-libs

RUN git clone https://aur.archlinux.org/sphinxbase.git /home/nubots/sphinxbase
WORKDIR /home/nubots/sphinxbase
# Skip dependency checks because we assume LAPACK is provided by openblas?
RUN makepkg --nodeps || true
RUN test -f sphinxbase-5prealpha-11-x86_64.pkg.tar.zst
RUN sudo pacman -U --noconfirm sphinxbase-5prealpha-11-x86_64.pkg.tar.zst

RUN git clone https://aur.archlinux.org/pocketsphinx.git /home/nubots/pocketsphinx
WORKDIR /home/nubots/pocketsphinx
RUN makepkg || true
RUN test -f pocketsphinx-5prealpha-10-x86_64.pkg.tar.zst

FROM nuhear:base as voice2json

COPY --from=kaldi-openfst /home/nubots/kaldi-openfst/kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=kaldi /home/nubots/kaldi/kaldi-5.5.r9187.d53b62fd4-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=openfst /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=ngram /home/nubots/opengrm-ngram/ngram-1.3.11-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=phonetisaurus /home/nubots/python-pybindgen/python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=phonetisaurus /home/nubots/phonetisaurus-git/phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=pocketsphinx /home/nubots/sphinxbase/sphinxbase-5prealpha-11-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=pocketsphinx /home/nubots/pocketsphinx/pocketsphinx-5prealpha-10-x86_64.pkg.tar.zst /home/nubots/packages/

# RUN install-package python-wheel swig pulseaudio alsa-lib gst-plugins-base-libs python2

WORKDIR /home/nubots/packages
RUN sudo pacman -U --noconfirm \
    kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst \
    kaldi-5.5.r9187.d53b62fd4-1-x86_64.pkg.tar.zst \
    openfst-1.7.9-1-x86_64.pkg.tar.zst \
    ngram-1.3.11-1-x86_64.pkg.tar.zst \
    python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst \
    phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst \
    sphinxbase-5prealpha-11-x86_64.pkg.tar.zst \
    pocketsphinx-5prealpha-10-x86_64.pkg.tar.zst

RUN install-package python-wheel swig alsa-lib jack portaudio

RUN git clone https://github.com/synesthesiam/voice2json /home/nubots/voice2json
WORKDIR /home/nubots/voice2json

# ./configure does not work for us here because it tries to do too much. Run it if you don't believe me.
# We only need the setup.py script to work and it requires us to substitute these values.
# They were taken from around lines 580 where it says "Identity of this package."
RUN <<EOF cat > mod.sed
/@ENABLE_POCKETSPHINX@/s//true/
/@ENABLE_KALDI@/s//true/
/@ENABLE_DEEPSPEECH@/s//false/
/@PACKAGE_NAME@/s//voice2json/
/@PACKAGE_VERSION@/s//0.2.0/
/@PACKAGE_BUGREPORT@/s//mike@rhasspy.org/
EOF
RUN sed -f mod.sed setup.py.in > setup.py
RUN sudo python setup.py install

RUN git clone https://github.com/mrwerdo/nuhear /home/nubots/nuhear
WORKDIR /home/nubots/nuhear

# RUN install -D "$(srcdir)/voice2json.sh" "$(DESTDIR)$(prefix)/bin/voice2json"
# RUN install -D "--target-directory=$(DESTDIR)$(prefix)/share/voice2json/etc" "$(srcdir)/etc/profile.defaults.yml"
# RUN install -D "--target-directory=$(DESTDIR)$(prefix)/share/voice2json/etc/profiles" "$(srcdir)/etc/profiles/"*
# RUN install -D "--target-directory=$(DESTDIR)$(prefix)/share/voice2json/etc/precise" "$(srcdir)/etc/precise"/*
# RUN install -D "--target-directory=$(DESTDIR)$(prefix)/share/voice2json" VERSION README.md LICENSE
# RUN if [[ -d "$(srcdir)/site" ]]; then cp -fR "$(srcdir)/site" "$(DESTDIR)$(prefix)/share/voice2json/"; fi || true
