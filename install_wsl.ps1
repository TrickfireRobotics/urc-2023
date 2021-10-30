#!this-is-meant-for-windows-not-linux

$architecture = $env:PROCESSOR_ARCHITECTURE
if ($architecture -ne "AMD64") {
    Write-Error -Message "Your computer uses $architecture and Docker Desktop only works on AMD64 (64-bit Intel or AMD CPUs). Sorry." -Category NotImplemented
    exit 1
}

$windows_build = [System.Environment]::OSVersion.Version.Build
if ($windows_build -lt 19041) {
    Write-Error -Message "You need Windows 10 with build number 19041 at minimum or Windows 11 (you are on build $windows_build). Update Windows." -Category NotInstalled
    exit 1
}

wsl --list | Out-Null
if ($LASTEXITCODE -eq 0) {
    Write-Warning "The Windows Subsystem for Linux was already installed on your system."
    wsl --update
    wsl --install --distribution Ubuntu
    wsl --set-version Ubuntu 2
    wsl --set-default Ubuntu
} else {
    wsl --install
}

Write-Warning "You need to restart your computer."
Restart-Computer -Confirm
