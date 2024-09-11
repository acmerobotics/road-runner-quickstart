# Road Runner Quickstart

Check out the [docs](https://rr.brott.dev/docs/v1-0/tuning/).

Welcome to the AIM Robotics 23-23 season repository
Downloading Android Studio
To develop and run the code, you need to download Android Studio. You can download the latest version of Android Studio from the official website here.

After downloading Android Studio, follow the installation instructions for your operating system.

Cloning the Repository
Create a new directory for the repository on your local machine.

To clone the repository, click on the green "Code" button on the GitHub page and copy the URL.

In Android Studio, click Get from VCS and paste in the URL.

Confirm that the repositry is now in the local directory.

Connecting to the Global Repository
Before you can push changes to the global repository or pull changes from it, you need to connect your local repository to the global one. To do this, follow these steps:

Check if a remote already exists with the command:git remote -v

If so, move on past this section

Assuming there is no remote, open the terminal on your local machine.

Navigate to the directory where you cloned the repository by running this command: cd location/of/the/directory

Run the following command to add the global repository as a remote: git remote add origin AIM-Robotics-2023-2024

Verify that the remote has been added by running the following command: git remote -v

This should list the global repository as origin

Congratulations! Proceed to CONTRIBUTING.md to learn how to properly add to this project