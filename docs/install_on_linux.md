# Installing on Linux
If you already have any of the following software installed, you can skip those respective sections. We cannot guarantee that your currently installed versions are compatible with our development scheme, so update your software if something doesn't work.

**This guide is for Ubuntu 20.04**, but this software is available across most Linux distributions (but with different package managers and install steps).

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
[Create a GitHub account](https://github.com/) if you don't have one already. GitHub is not Git - it's merely an online host for Git repositories.

Skip the rest of this section if you already have Git authentication set up with GitHub using an SSH key or other means.

Git is version control software - it comes with Ubuntu and many other distributions by default. If you don't have it, install it:
```bash
sudo apt update
sudo apt install git -y
```

Set up your name and email so your Git commits have your identity on them. `user.name` doesn't have to use your real name, but `user.email` must match your GitHub email:
```bash
git config --global user.name "YOUR NAME HERE"
git config --global user.email "YOUR GITHUB EMAIL HERE"
```

We need Git to authenticate with GitHub: follow the [GitHub guide for generating SSH keys](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent). Ignore the "hardware security key" section, that's meant for users with USB security dongles.

You must also add that SSH key to GitHub. If you didn't already follow the [guide to add SSH keys to GitHub](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account), complete those steps. Give your key an informative name like "Thinkpad T480 Ubuntu 20.04" so you aren't confused in a few years when you have accumulated a dozen different keys.

### Cloning this repository
Clone this repository into your home directory (or wherever you prefer).
```bash
cd ~
git clone --recurse-submodules https://github.com/TrickfireRobotics/urc-2023.git
```

## Visual Studio Code
Download [VS Code](https://code.visualstudio.com/) and install it:
```bash
sudo apt install ~/Downloads/<VSCODE-PACKAGE-NAME>.deb
```

If you already have VS Code, update it (**Help** > **Check for Updates...**).

Once installed, we need a few VS Code extensions. Open the Extensions sidebar (`Ctrl+Shift+X`) and install **Remote - Containers** by Microsoft. VS Code may auto-detect software and offer to install extensions for you.

### Open `urc-2023` in VS Code
You're almost there.

Open VS Code, select **File > Open Folder...**, and open the `urc-2023` repository you cloned earlier. If VS Code asks you if you trust the workspace authors, select **Yes, I trust the authors**.

After the `urc-2023` folder opens, a notification will tell you that it noticed a Dev Container configuration file. Click the **Reopen in Container** option. If you miss the notification, you can search **Remote-Containers: Reopen in Container** in the Command Palette (`Ctrl+Shift+P` then type the command).

**You're ready to develop now!** 🥳

This procedure is running the VS Code Server (the part which reads/writes files and runs commands) in the Docker development container. Your VS Code Client (the user interface) remains on Linux but talks to the Server in the container.

### Reopening VS Code
If you close VS Code and reopen it later, it will try to reopen `urc-2023` in a container.

### Disconnecting from the container
To leave the container (to work on other projects), run **Remote: Close Remote Connection** from the Command Palette or click the green Remote button in the bottom-left and select **Close Remote Connection**.

You can always re-enter the container with that green button (**Open Folder in container** > `urc-2023`).
