# Installing on Windows

Docker containers run on Linux, but we get around that by using the [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/faq) (WSL2) as a backend for [Docker Desktop](https://www.docker.com/products/docker-desktop).

1. Update Windows 10 so your build number is at least 19041 (Settings > About > OS Build). Windows 11 users don't need to update anything.
2. Open Powershell and run:
   ```
   wsl --install
   ```
   If you already have WSL 1 or 2, run this to make sure you're on Ubuntu using WSL2:
   ```
    wsl --update
    wsl --install --distribution Ubuntu
    wsl --set-version Ubuntu 2
    wsl --set-default Ubuntu
   ```
3. Restart your computer.
4. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop). If you already have it, update it, and make sure it uses the WSL2 backend (Docker Desktop > Settings > Use the WSL 2 based engine). If Docker Desktop fails to start, try deleting `%appdata%/Docker`.