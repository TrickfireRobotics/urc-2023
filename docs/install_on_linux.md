# Installing on Linux
If you already have any of the following software installed, you can skip those respective sections. We cannot guarantee that your currently installed versions are compatible with our development scheme, so update your software if something doesn't work.

This guide is for Ubuntu 20.04, but this software is available across most Linux distributions (but with different package managers and install steps).

## Docker Engine
Run the following in a terminal to install Docker Engine from Docker's APT repository:
```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
```
Docker runs as root by default and requires `sudo` for commands. We need to create a new user group named `docker` and add ourselves to it. This lets VS Code run without root permissions, too.
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```
Log out and back in to reevaluate your group membership. Be aware that this introduces a security hole into root priveleges from your user account through Docker Engine. We are aware of Docker's Rootless mode, but we have not yet found a way to make it fully compatible with VS Code's Remote Containers extension.

## Git
TODO

## Visual Studio Code
TODO