!# /bin/bash
cd /usr/local/src
sudo chmod a+rw /usr/local/src # Dodgelly change the read write permissions

# Kaldi
git clone -b vosk --single-branch https://github.com/alphacep/kaldi
cd kaldi/tools/
sed -i 's:--enable-ngram-fsts:--enable-ngram-fsts --disable-bin:g' Makefile
make openfst cub
extras/install_openblas_clapack.sh
cd ../src
./configure --use-cuda=no --mathlib=OPENBLAS_CLAPACK --shared
sed -i 's:-msse -msse2:-msse -msse2:g' kaldi.mk || echo "ERROR"
sed -i 's: -O1 : -O3 :g' kaldi.mk  || echo "ERROR"
make online2 lm rnnlm

# Vosk
cd /usr/local/src
git clone https://github.com/alphacep/vosk-api.git
cd vosk-api/src
KALDI_ROOT=/usr/local/src/kaldi make
sudo cp libvosk.so /usr/local/lib/libvosk.so
sudo cp vosk_api.h /usr/local/include/vosk_api.h
cd ../c
make

# Model
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
sudo unzip vosk-model-small-en-us-0.15.zip
sudo mv vosk-model-small-en-us-0.15 model
