# Installing on Windows

We need to get Docker working in Windows, but Docker containers are native to Linux. We get around that by using the [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/faq) (WSL2) as a backend for [Docker Desktop](https://www.docker.com/products/docker-desktop). Using that combination, we will connect VS Code through Docker Desktop to a container runnning in WSL2. We use Git for version control, which tracks file changes and syncs with our GitHub repository. To simplify GitHub authentication, we have to connect Git in WSL2 to Git Credential Manager Core running in Windows.

Buckle up.

## Windows Subsystem for Linux
Update Windows 10 so your build number is at least 19041 (Settings > About > **OS Build**). Windows 11 users don't need to update anything. Open Command Prompt or Powershell as an administrator (right-click > **Run as administrator**) and run:
```
wsl --install
```
**Restart your computer.** After you restart and log back in, a terminal to Ubuntu will open. It will ask you to set a username and password - do it. This sets your login information for that Ubuntu distribution of WSL2. It's separate from your Windows login, but you can make it the same username/password if you want.

By the way, WSL2 is a Linux system, but Ubuntu is a Linux "distribution". Distributions take the Linux kernel and bundle it with many software packages to deliver a complete operating system product.

### What if I already have WSL1 or 2?
If you already have WSL 1 or 2, run this in Command Prompt/Powershell (as an administrator) to make sure you're on Ubuntu using WSL2:
```
   wsl --update
   wsl --install --distribution Ubuntu
   wsl --set-version Ubuntu 2
   wsl --set-default Ubuntu
```

## Git
[Create a GitHub account](https://github.com/) if you don't have one already. GitHub is not Git - it's merely an online host for Git repositories.

Git is version control software - it comes with Ubuntu by default. Open the WSL2 terminal (search for "Ubuntu" in the start menu) and run the following:
```
git config --global user.name "YOUR NAME HERE"
git config --global user.email "YOUR EMAIL HERE"
```

### I already have Git for Windows
Git on Ubuntu (in WSL2) is different from Git for Windows. You may remember doing the `git config` steps above in Windows, but don't skip those in WSL2 (the Git configurations are not shared between WSL2 and Windows).

We don't actually need most of Git for Windows. We only need to pass your GitHub credentials from Git Credential Manager Core to Git in WSL2. GCM Core requires Git for Windows version 2.33 or above (check with `git --version`). To update Git for Windows, open Git Bash and run:
```
git update-git-for-windows
```
Select the option for **Git Credential Manager Core** while installing.

Finally, open a WSL2 terminal (not Git Bash on Windows!) and run:
```
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/libexec/git-core/git-credential-manager-core.exe"
```
That tells Git in WSL2 to call back into the Windows filesystem (your Windows `C:\` drive filesystem is presented as `/mnt/c` in WSL2) to use GCM Core as a credential manager.

### I don't have Git for Windows
We don't need Git for Windows - we can install Git Credential Manager Core by itself. From the [releases page of GCM Core](https://github.com/microsoft/Git-Credential-Manager-Core/releases), install the latest version of the `gcmcoreuser-win-x86` .exe file (don't pick `gcmcore-win-x86`).

After installing it, open a WSL2 terminal and run:
```
git config --global credential.helper "$(wslpath "$(cmd.exe /c echo %LocalAppData%\\Programs\\Git Credential Manager Core\\git-credential-manager-core.exe 2>/dev/null)" | sed -e 's/\r//g' -e 's/ /\\ /g')"
echo >> ~/.bashrc
echo 'export GIT_EXEC_PATH="$(git --exec-path)"' >> ~/.bashrc
echo 'export WSLENV=$WSLENV:GIT_EXEC_PATH/wp' >> ~/.bashrc
source ~/.bashrc
```
That tells Git in WSL2 to use GCM Core in your Windows filesystem as a credential manager. A few extra configurations in `.bashrc` are necessary to help GCM Core call back into Git in WSL2.

### Final Git steps
In a WSL2 terminal, download this repository into your WSL2 home directory:
```
cd ~
git clone --recurse-submodules https://github.com/TrickfireRobotics/nasa-rmc.git
```
The tilde `~` is shorthand for your `$HOME` directory (`/home/yourusername`) in Linux shells. That first command `cd ~` moves into your home directory. This is in a different filesystem than your Windows files.

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

The folder browser might not put you into a familiar WSL2 directory. The correct absolute path should look like `Network > wsl$ > Ubuntu > home > your-linux-username > nasa-rmc`.

If VS Code asks you if you trust the workspace authors, select **Yes, I trust the authors**.

After VS Code loads into WSL2, a notification will tell you that it noticed a Dev Container configuration file. Click the **Reopen in Container** option. If you miss the notification, you can search **Remote-Containers: Reopen in Container** in the Command Palette.

**You're ready to develop now!** 🥳

This procedure sent the VS Code Server (the part which reads/writes files and runs commands) from Windows to WSL2, then from WSL2 to a Docker container. Your VS Code Client (the user interface) remains on Windows but talks to the Server which currently runs in the container.

### Reopening VS Code
If you close VS Code and reopen it later, it will try to reopen `nasa-rmc` in a container.
Make sure Docker is running (check for the whale icon 🐳 in the system tray) or it will give you an error.

### Disconnecting from the container
To leave the container (to work on other projects), run **Remote: Close Remote Connection** from the Command Palette or click the green Remote button in the bottom-left and select **Close Remote Connection**.

You can always re-enter WSL2 then reopen in a container with this green button (**Open Folder in WSL** > `nasa-rmc` > **Reopen in Container**).