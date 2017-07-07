#!/bin/bash

# Apply the puppet file to the vm
sudo puppet apply --parser=future --verbose --debug --modulepath=puppet/modules:/etc/puppet/modules puppet/manifests/travis.pp

# For some reason it looks like we have to run update-alternative again on travis
sudo update-alternatives --remove-all gcc || true
sudo update-alternatives --remove-all g++ || true
sudo update-alternatives --remove-all gfortan || true
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 --slave /usr/bin/g++ g++ /usr/bin/g++-7 --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7
