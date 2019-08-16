FROM archlinux/base:latest

# Get packages
RUN pacman -Syu --noconfirm --needed \
    && pacman -S --noconfirm --needed clang colordiff \
    && rm -rf /var/cache
RUN groupadd -r nubots && useradd --no-log-init -r -g nubots nubots

# Create the home directory owned by nubots
RUN mkdir -p /home/nubots && chown -R nubots:nubots /home/nubots

WORKDIR /home/nubots
