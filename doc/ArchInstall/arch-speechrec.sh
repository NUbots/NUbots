!# /bin/bash
sudo pacman -Sy sox --noconfirm
sudo pacman -S subversion --noconfirm
sudo pacman -S python --noconfirm
sudo pacman -S python2 --noconfirm
sudo pacman -S unzip --noconfirm
sudo chmod a+rw /usr
cd /usr
git clone -b vosk --single-branch https://github.com/alphacep/kaldi
cd /usr/kaldi/tools/
sed -i 's:--enable-ngram-fsts:--enable-ngram-fsts --disable-bin:g' Makefile
make -j 6 openfst cub
extras/install_openblas_clapack.sh
cd /usr/kaldi/src
./configure --use-cuda=no --mathlib=OPENBLAS_CLAPACK --shared
sed -i 's:-msse -msse2:-msse -msse2:g' kaldi.mk || echo "ERROR"
sed -i 's: -O1 : -O3 :g' kaldi.mk  || echo "ERROR"
make -j 6 online2 lm rnnlm
cd /usr
git clone https://github.com/alphacep/vosk-api.git
cd /usr/vosk-api/src
KALDI_MKL=$KALDI_MKL KALDI_ROOT=/usr/kaldi make -j 6
cd /usr/vosk-api/c
make
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
sudo unzip vosk-model-small-en-us-0.15.zip
sudo mv vosk-model-small-en-us-0.15 model
