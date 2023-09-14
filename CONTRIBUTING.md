## Here you will find how to propery contribute to this repository

## Creating a branch

The first step is to create a branch that will contain all of a certain feature's work.

To do this run the following command:

`git checkout -b <name-of-branch>`

This will create a new branch, and switch to the newly created branch.

## Committing and pushing upstream

While working, it is imperative you consistently commit and push upstream.

Follow these few steps:

Make sure all files are added either by running `git add <filename>`

Alternatively, run `git add .` to add all files

Once added, commit the files and add a descriptive message like so:

`git commit -m "MESSAGE GOES HERE""`

Finally, push the commit upstream with the following command:

`git push -u origin <branch-name-here>`

## Pull Request

Once you finish the feature, create a pull request on the github website and write a description of the branch

Click `Create pull request` and voila, you have officially requested to merge your branch back into master

The PR will be reviewed and then either modified or accepted

## Resetting for the next branch

Before you create a new branch for a new feature, make sure to pull the latest master so you have the most updated repo

Then, after the acceptance of the PR, delete the old local branch like so:

`git branch -d <branch-name-here>`