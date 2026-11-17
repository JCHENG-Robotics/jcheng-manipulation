#!/bin/bash
# Install Docker Engine from the Ubuntu repository.
# Run this in your own terminal; it needs your sudo password.
set -euo pipefail

if command -v docker >/dev/null 2>&1; then
    echo "Docker is already installed: $(docker --version)"
else
    sudo apt-get update
    sudo apt-get install -y docker.io
    sudo systemctl enable --now docker
fi

if ! getent group docker >/dev/null 2>&1; then
    sudo groupadd docker
fi
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    echo "Added $USER to the docker group."
fi

echo
echo "Docker version:"
docker --version 2>/dev/null || sudo docker --version
echo
echo "If 'docker' still needs sudo, run 'newgrp docker' or log out/in once."
