FROM archlinux/base:latest

# Get packages
RUN pacman -Syu --noconfirm --needed \
    && pacman -S --noconfirm --needed \
    clang \
    colordiff \
    parallel \
    which \
    awk \
    && rm -rf /var/cache

# Create the home directory owned by nubots
RUN mkdir -p /home/nubots
WORKDIR /home/nubots
