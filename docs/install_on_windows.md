# Installing on Windows

We need to get Docker working in Windows, but Docker containers are native to Linux. We get around that by using the [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/faq) (WSL2) as a backend for [Docker Desktop](https://www.docker.com/products/docker-desktop). Using that combination, we will connect VS Code through Docker Desktop to a container runnning in WSL2. We use Git for version control, which tracks file changes and syncs with our GitHub repository.

## Windows Subsystem for Linux
Update Windows 10 so your build number is at least 19041 (Settings > About > **OS Build**). Windows 11 users don't need to update anything. Open Command Prompt or Powershell as an administrator (right-click > **Run as administrator**) and run:
```
wsl --install
```
**Restart your computer.** After you restart and log back in, a terminal to Ubuntu will open. It will ask you to set a username and password - do it. This sets your login information for that Ubuntu distribution of WSL2. It's separate from your Windows login, but you can make it the same username/password if you want.

(TODO: move this into a footnote because people may run it even if they don't already have WSL1/2) If you already have WSL 1 or 2, run this to make sure you're on Ubuntu using WSL2:
```
   wsl --update
   wsl --install --distribution Ubuntu
   wsl --set-version Ubuntu 2
   wsl --set-default Ubuntu
```
By the way, WSL2 is a Linux system, but Ubuntu is a Linux "distribution". Distributions take the Linux kernel and bundle it with many software packages to deliver a complete operating system product.

## Git
Git is version control software - it comes with Ubuntu by default, and we're not using Git for Windows if you already have that installed. Open the WSL2 terminal (search for "Ubuntu" in the start menu) and enter the following:
```
git config --global user.name "YOUR NAME HERE"
git config --global user.email "YOUR EMAIL HERE"
```

TODO: Git/GitHub SSH setup steps

Finally, download this repository into your WSL2 home directory:
```
cd ~
git clone --recurse-submodules https://github.com/TrickfireRobotics/nasa-rmc.git
```
The tilde `~` is shorthand for your `$HOME` directory (`/home/yourusername`) in Linux shells. That first command moves into your home directory. This is in a different filesystem than your Windows files.

## Docker Desktop
Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop). If you already have it, update it, and make sure it uses the WSL2 backend (Docker Desktop > Settings > **Use the WSL 2 based engine**). If Docker Desktop fails to start, try deleting `%appdata%/Docker`.

Docker Desktop will offer you a tutorial - it's not necessary for setting up this repository. You can skip it if you want.

In general, you can't work with containers if Docker isn't running on Windows. There will be a whale icon in the Windows system tray if Docker is running. You can enable/disable Docker auto-starting when you log in (Docker Desktop > Settings > General > **Start Docker Desktop when you log in**).

## Visual Studio Code
Install [VS Code](https://code.visualstudio.com/). If you already have it, update it (Help > Check for Updates...).

We need a few VS Code extensions. Open the Extensions sidebar (`Ctrl+Shift+X`) and install **Remote - WSL** and **Remote - Containers** by Microsoft. VS Code may auto-detect software and offer to install extensions for you.

## Open `nasa-rmc` in VS Code
You're almost there.

In VS Code, open the Command Palette (`Ctrl+Shift+P`), search for **Remote-WSL: Open Folder in WSL...** and run that command. A folder browser will pop up - find and select `nasa-rmc`. This folder browser is showing your WSL2 filesystem, not your Windows filesystem.

TODO: for some reason, the folder browser opens up into Linux / root directory instead of /home/username.

TODO: select "yes" if it asks you if you trust the workspace author.

After VS Code loads into WSL2, a notification will tell you that it noticed a Dev Container configuration file. Click the **Reopen in Container** option. If you miss the notification, you can search **Remote-Containers: Reopen in Container** in the Command Palette.

You're ready to develop now! 🥳

This procedure sent the VS Code Server (the part which reads/writes files and runs commands) from Windows to WSL2, then from WSL2 to a Docker container. Your VS Code Client (the user interface) remains on Windows but talks to the Server which currently runs in the container.

### Reopening VS Code
If you close VS Code and reopen it later, it will try to reopen `nasa-rmc` in a container.
Make sure Docker is running (check for the whale icon in the system tray) or it will give you an error.

To disconnect from the container (to work on other projects), run **Remote: Close Remote Connection** from the Command Palette or click the green Remote button in the bottom-left and select **Close Remote Connection**.

You can always re-enter WSL2 then reopen in a container with this green button (**Open Folder in WSL** > `nasa-rmc` > **Reopen in Container**).