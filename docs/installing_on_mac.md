# Installing on Mac (Intel/Apple chip)

## Docker Desktop


To install Docker on a MacBook Pro, you can follow these steps:

1. **Check System Requirements:** Ensure that your MacBook Pro meets the minimum system requirements for Docker. Visit the Docker documentation or official website to check the requirements for the specific version you intend to install.

-   Note: There are known issue when trying to access Docker through Mac products with M1 chips: these issues and solutions may be found on Docker's [Known issues page](https://docs.docker.com/desktop/troubleshoot/known-issues/) under the 'For Mac with Apple Silicon' tab.

2. **Download Docker for Mac:** Open a web browser and go to the Docker website (https://www.docker.com/products/docker-desktop). On the Docker Desktop page, click on the "Download for Mac" button to start downloading the Docker installation package.

3. **Install Docker:** Once the download is complete, locate the downloaded Docker installation package (usually in your Downloads folder) and double-click on it to start the installation process.

4. **Drag Docker to Applications:** After opening the Docker installation package, you'll see the Docker icon. Drag the Docker icon to the Applications folder to install Docker on your MacBook Pro.

5. **Launch Docker:** Open the Applications folder and locate the Docker application. Double-click on the Docker icon to launch it. Docker will start initializing and setting up the necessary components.

6. **Provide System Authentication:** During the installation process, you may be prompted to enter your system password to authorize Docker to make changes to your system.

7. **Wait for Initialization:** Docker may take a few minutes to complete the initialization process. It will set up the required components, including downloading the Docker Engine, creating the Docker virtual machine, and configuring networking.

8. **Enable HyperKit (optional):** Docker for Mac uses HyperKit as the virtualization technology. If you are prompted to enable HyperKit during the installation process, grant the necessary permissions to allow Docker to use it.

9. **Check Docker Status:** Once the Docker initialization is complete, you should see the Docker icon appear in the menu bar. Click on the Docker icon to check the status and ensure that Docker is running properly.

That's it! Docker should now be installed and running on your MacBook Pro. You can start using Docker by running Docker commands in the terminal or by using graphical user interfaces (GUIs) like Docker Desktop, which provides a user-friendly interface for managing Docker containers and images.

## Git & GitHhub Desktop

[Create a GitHub account](https://github.com/) if you don't have one already. GitHub is not Git - it's merely an online host for Git repositories.


To install Git on a Mac and configure your name and email, you can follow these steps:

1. Open the Terminal application. You can find it by searching for "Terminal" in Spotlight (press Command + Space and type "Terminal").

2. Check if Git is already installed on your Mac by typing the following command and pressing Enter:

   ```
   git --version
   ```

   If Git is not installed, you will see a message prompting you to install it.

3. If Git is not installed, you can install it using Homebrew, a popular package manager for macOS. To install Homebrew, run the following command in the Terminal:

   ```
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

   Follow the instructions in the Terminal to complete the Homebrew installation.

4. Once Homebrew is installed, you can use it to install Git. Run the following command in the Terminal:

   ```
   brew install git
   ```

   Homebrew will download and install Git on your Mac.

5. After Git is installed, you need to configure your name and email address. Run the following commands in the Terminal, replacing "Your Name" with your actual name and "your-email@example.com" with your email address:

   ```
   git config --global user.name "Your Name"
   git config --global user.email "your-email@example.com"
   ```

   These commands set your name and email address globally, so they will be used for all your Git repositories.

6. You can verify that the name and email are set correctly by running the following command:

   ```
   git config --global --get user.name
   git config --global --get user.email
   ```

   The Terminal will display the configured name and email.

That's it! You have successfully installed Git on your Mac and configured your name and email. You can now use Git for version control and collaborate on projects.

To install GitHub Desktop on a Mac, follow these steps:

1. Open your web browser and go to the GitHub Desktop download page for Mac, which you can find at: [https://desktop.github.com](https://desktop.github.com)

2. Click on the "Download for macOS" button. The download should start automatically. If not, you may need to click on a download link on the page.

3. Once the download is complete, locate the downloaded file (typically in your Downloads folder) and double-click on it to open the installer.

4. A window will appear with the GitHub Desktop logo and the Applications folder icon. Drag the GitHub Desktop icon onto the Applications folder icon to install it.

5. Wait for the installation process to complete. It may take a few moments.

6. Once the installation is finished, you can find GitHub Desktop in your Applications folder. Open it by clicking on its icon.

7. GitHub Desktop will launch, and you'll be prompted to sign in with your GitHub account. Enter your GitHub username and password, or choose the option to sign in using your browser.

8. After signing in, GitHub Desktop will guide you through some initial setup steps, such as configuring your Git preferences and selecting a default text editor. Follow the instructions on the screen to complete the setup.

9. Once the setup is complete, you can start using GitHub Desktop to clone repositories, create branches, commit changes, and more. You'll have access to your GitHub repositories and can manage them directly from the application's user interface.

That's it! You have successfully installed GitHub Desktop on your Mac. Enjoy using it for your version control and collaboration needs.

## Visual Studio Code (VS Code)

To install and run Visual Studio Code (VS Code) on a Mac, you can follow these steps:

1. Open your web browser and go to the Visual Studio Code download page for Mac, which you can find at: [https://code.visualstudio.com](https://code.visualstudio.com)

2. Click on the "Download for macOS" button. The download should start automatically. If not, you may need to click on a download link on the page.

3. Once the download is complete, locate the downloaded file (typically in your Downloads folder) and double-click on it to open the VS Code disk image (.dmg) file.

4. A new Finder window will open, displaying the Visual Studio Code application icon. Drag the VS Code icon onto the "Applications" folder icon to install it.

5. Wait for the copying process to complete. It may take a few moments.

6. Once the installation is finished, you can find Visual Studio Code in your Applications folder. Open it by clicking on its icon.

7. When you first launch Visual Studio Code, you might see a security warning saying that it is from an unidentified developer. In that case, right-click on the VS Code icon and select "Open" from the context menu. Then, confirm your action in the pop-up dialog.

8. Visual Studio Code will launch, and you can start using it for your coding projects. You'll see a welcome screen with various options and a search bar.

9. Customize your preferences, extensions, and settings in VS Code to enhance your development experience. You can explore the various features and settings available in the editor.

That's it! You have successfully installed and launched Visual Studio Code on your Mac. You can now use it as your code editor for various programming languages and development tasks.

### Open `urc-2023` in VS Code
You're almost there.

Open VS Code, select **File > Open Folder...**, and open the `urc-2023` repository you cloned earlier. If VS Code asks you if you trust the workspace authors, select **Yes, I trust the authors**.

After the `urc-2023` folder opens, a notification will tell you that it noticed a Dev Container configuration file. Click the **Reopen in Container** option. If you miss the notification, you can search **Dev-Containers: Reopen in Container** in the Command Palette (`command+shift+P` then type the command).

**You're ready to develop now!** 🥳

This procedure is running the VS Code Server (the part which reads/writes files and runs commands) in the Docker development container. Your VS Code Client (the user interface) remains on Linux but talks to the Server in the container.

### Reopening VS Code
If you close VS Code and reopen it later, it will try to reopen `urc-2023` in a container.

### Disconnecting from the container
To leave the container (to work on other projects), run **Remote: Close Remote Connection** from the Command Palette or click the green Remote button in the bottom-left and select **Close Remote Connection**.

You can always re-enter the container with that green button (**Open Folder in container** > `urc-2023`).
