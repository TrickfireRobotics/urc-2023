# Installing on Windows

We need to get Docker working in Windows, but Docker containers are native to Linux. We get around that by using the [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/faq) (WSL2) as a backend for [Docker Desktop](https://www.docker.com/products/docker-desktop). Using that combination, we will connect VS Code through Docker Desktop to a container runnning in WSL2. We use Git for version control, which tracks file changes and syncs with our GitHub repository.

## Windows Subsystem for Linux
Update Windows 10 so your build number is at least 19041 (Settings > About > OS Build). Windows 11 users don't need to update anything. Open Powershell and run:
```
wsl --install
```
This will eventually open a WSL2 terminal. It will ask you to set a username and password for WSL2 - do it.

If you already have WSL 1 or 2, run this to make sure you're on Ubuntu using WSL2:
```
   wsl --update
   wsl --install --distribution Ubuntu
   wsl --set-version Ubuntu 2
   wsl --set-default Ubuntu
```
Restart your computer to finish installing WSL2.

## Docker Desktop
Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop). If you already have it, update it, and make sure it uses the WSL2 backend (Docker Desktop > Settings > Use the WSL 2 based engine). If Docker Desktop fails to start, try deleting `%appdata%/Docker`.

## Git for Windows
*If you know what you're doing, you can ignore these Git setup steps.*

Install [Git for Windows](https://git-scm.com/download/win). If you already have it, update it to at least version 2.33 (`git update-git-for-windows`), because it comes with Git Credential Manager Core, a tool that simplifies authentication with GitHub.

Open the WSL2 terminal (search for "WSL" or "Ubuntu" in the start menu) and enter this:
```
git config --global user.name "YOUR NAME HERE"
git config --global user.email "YOUR EMAIL HERE"

git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/libexec/git-core/git-credential-manager-core.exe"
```
That sets up your name and email with Git in WSL2 (not Git for Windows, a separate Git that came with Ubuntu). It then tells Git to call back to a credential helper located in Windows (GCM Core). VS Code will later forward your Git credentials from WSL2 to your Docker containers.

Finally, to download this repository into your WSL2 home directory:
```
cd ~
git clone --recurse-submodules https://github.com/TrickfireRobotics/nasa-rmc.git
```
The tilde `~` is shorthand for your `$HOME` directory (`/home/yourusername`) in Linux shells. That first command moves into your home directory. This is in a different filesystem than your Windows files.

## Visual Studio Code
Install [VS Code](https://code.visualstudio.com/). If you already have it, update it (Help > Check for Updates...).

We need a few VS Code extensions. Open the Extensions sidebar (`Ctrl+Shift+X`) and install **Remote - WSL** and **Remote - Containers** by Microsoft. VS Code may auto-detect software and offer to install extensions for you.

## Open `nasa-rmc` in VS Code
You're almost there.

In VS Code, open the Command Palette (`Ctrl+Shift+P`), search for **Remote-WSL: Open Folder in WSL...** and run that command. A folder browser will pop up - find and select `nasa-rmc`. This folder browser is showing your WSL2 filesystem, not your Windows filesystem.

After VS Code loads into WSL2, a notification will tell you that it noticed a Dev Container configuration file. Click the **Reopen in Container** option. If you miss the notification, you can search **Remote-Containers: Reopen in Container** in the Command Palette.

This procedure sent the VS Code Server (the part which reads/writes files and runs commands) from Windows to WSL2, then from WSL2 to a Docker container. Your VS Code Client (the user interface) remains on Windows but talks to the Server which currently runs in the container.

## Start developing!
Now that you're running VS Code inside a container, the entire team will have the same workflow, even if they're not on Windows. Open the integrated terminal (`` Ctrl+` `` or `Ctrl+J`) to see the Linux shell inside the container. Happy developing!