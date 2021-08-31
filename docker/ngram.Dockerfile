FROM nuhear:base

RUN git clone https://aur.archlinux.org/opengrm-ngram.git /home/nubots/opengrm-ngram
WORKDIR /home/nubots/opengrm-ngram
# Set the package name because it changed recently at opengrm.org
RUN sed sed '/^pkgname/s/=.*/=ngram/' -i PKGBUILD
# Set version number.
RUN sed '/^pkgver/s/=.*/=1.3.11/' -i PKGBUILD
# Set source url. The old one didn't exist.
RUN sed '/^source/s/=.*/=("http:\/\/opengrm.org\/twiki\/pub\/GRM\/NGramDownload\/${pkgname}-${pkgver}.tar.gz")/' -i PKGBUILD
# Set the hash value. This value wasn't obtained securely (i.e. I downloaded insecure file then ran sha256sum)
RUN sed "/^md5sums=/s/.*/sha256sums=('05d417b57751dc7eca43b13d5f50eebe9d2338851bc47a854bb9318b91649773')/" -i PKGBUILD
RUN makepkg || true
