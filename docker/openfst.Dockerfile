FROM nuhear:base

RUN git clone https://aur.archlinux.org/openfst.git /home/nubots/openfst
WORKDIR /home/nubots/openfst
# Enable configure build options for openfst
RUN sed "34a \ \ OPTIONS+=' --enable-static'" -i PKGBUILD
RUN sed "35a \ \ OPTIONS+=' --enable-shared'" -i PKGBUILD
RUN makepkg || true
