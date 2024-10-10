# How to Properly Contribute to This Repository

## Creating a Branch

The first step in contributing is to create a branch that will contain all of the work for a specific feature.

If the branch is related to a GitHub issue, use the issue number in your branch name for better organization.

To create a branch for a new feature, run the following command:

```bash
git checkout -b <name-of-branch> // DO NOT INCLUDE THESE --> <>
```

This will create a new branch and switch to it.

## Fetching and Switching to a New Branch

When someone else creates a new branch (e.g., for an issue), you may need to fetch it. To do this:

1. Fetch all branches from the remote repository:
   ```bash
   git fetch
   ```

2. Switch to the new branch:
   ```bash
   git checkout <branch-name> // DO NOT INCLUDE THESE --> <>
   ```

## Committing and Pushing Upstream

While working, it's important to commit changes frequently and push them upstream. Follow these steps:

1. Add files to staging:
   ```bash
   git add <filename> // DO NOT INCLUDE THESE --> <>
   ```
   Or, add all files at once:
   ```bash
   git add .
   ```

2. Commit your changes with a descriptive message:
   ```bash
   git commit -m "MESSAGE GOES HERE"
   ```

3. Push your branch to the remote repository:
   ```bash
   git push -u origin <branch-name> // DO NOT INCLUDE THESE --> <>
   ```

## Pull Request

Once your feature is complete, create a pull request (PR) on GitHub:

1. Go to your branch on GitHub.
2. Click `Create pull request`.
3. **Ensure the base repository is the current season repo**
    1. **If not, the PR will compare to the wrong repository**
3. Write a clear description of the changes made in this branch.
4. Click `Create pull request`.

Your PR will be reviewed and either modified or accepted.

## Resetting for the Next Branch

Before starting a new feature, make sure your local repository is up to date. Follow these steps:

1. Switch to the master branch:
   ```bash
   git checkout master
   ```

2. Pull the latest `master` branch:
   ```bash
   git pull
   ```

3. Delete your old feature branch:
   ```bash
   git branch -d <branch-name> // DO NOT INCLUDE THESE --> <>
   ```

By following these steps, you'll always be working with the most up-to-date code.
