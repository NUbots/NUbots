FROM archlinux:base-20210110.0.13332
RUN echo -e "Server=http://host.docker.internal:7878/\$repo/os/\$arch\nServer=https://archive.archlinux.org/repos/2021/01/10/\$repo/os/\$arch" | tee /etc/pacman.d/mirrorlist
RUN pacman -Syyuu --noconfirm --needed --overwrite \*
COPY usr/local/bin/install-package /usr/local/bin/install-package
RUN install-package \
    wget \
    sudo \
    python \
    python-pip \
    base-devel \
    ninja \
    cmake \
    meson \
    nodejs-lts-fermium \
    npm \
    yarn \
    git \
    ncurses
# Get python to look in /usr/local for packages
RUN echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
    > $(python -c "import site; print(site.getsitepackages()[0])")/local.pth
COPY etc/xdg/pip/pip.conf /etc/xdg/pip/pip.conf

# Make sure /usr/local is checked for libraries and binaries
COPY etc/ld.so.conf.d/usrlocal.conf /etc/ld.so.conf.d/usrlocal.conf
RUN ldconfig

# Make a symlink from /usr/local/lib to /usr/local/lib64 so library install location is irrelevant
RUN cd /usr/local && ln -sf lib lib64

COPY usr/local/toolchain/generate_toolchains.py /usr/local/generate_toolchains.py
COPY usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr

# Copy over a tool to install simple standard conforming libraries from source
COPY usr/local/bin/download-and-extract /usr/local/bin/download-and-extract
COPY usr/local/bin/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-header-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-python-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-from-source-with-patches

# Generate toolchain files for generic
COPY usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr


# Setup the sudo so it can be run without a password
COPY etc/sudoers.d/user /etc/sudoers.d/user
RUN chmod 440 /etc/sudoers.d/user

# Create the user, and setup sudo so no password is required
ARG user_uid=1000
ARG user_gid=$user_uid
RUN groupadd --gid $user_gid nubots \
    && useradd --uid $user_uid --gid $user_gid -m nubots
USER nubots

# Copy ssh keys over to the system
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/.ssh
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa /home/nubots/.ssh/id_rsa
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa.pub /home/nubots/.ssh/id_rsa.pub
COPY --chown=nubots:nubots home/nubots/.ssh/config /home/nubots/.ssh/config
COPY --chown=nubots:nubots home/nubots/.gdbinit /home/nubots/.gdbinit
RUN chmod 600 /home/nubots/.ssh/id_rsa /home/nubots/.ssh/config

# Setup the locations where we will mount our folders
RUN install -d -m 0755 /home/nubots/NUbots
RUN install -d -m 0777 /home/nubots/build
WORKDIR /home/nubots/NUbots

# fixme: I don't know how much of the above is important.
# There might be more stuff from the nubots image that needs to be apart of the base image
# to make sure the system is setup the same.
