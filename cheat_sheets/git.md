# git cheat sheet
```sh
sudo apt install git
```
There is text-mode interface for Git called Tig which may be helpful:
```sh
sudo apt install tig
```

## Table of contents
* [Setup](#Setup)
* [Create repositories](#Create-repositories)
* [Remote](#Remote)
* [Basics](#Basics)
* [Stage/Unstage](#Stage/Unstage)
* [Commits](#Commits)
* [Branches](#Branches)
* [Stash](#Stash)
* [`.gitignore`](#gitignore)
* [Releases](#Releases)
* [Submodules](#Submodules)
  * [Create submodules](#Create-submodules)
  * [Upload changes](#Upload-changes)
  * [Download changes](#Download-changes)


### Setup
```sh
git config --global user.name <username>
git config --global user.email <email>
# Save username and password in order not to have to type them in every push
git config credential.helper store
```

### Create repositories
```sh
# Create new one
git init <project>
# Download an existing one
git clone <url>
```

### Remote
```sh
# To import code to a new repository on GitHub use origin as <remote>
git remote add <remote> <url>
git remote show <remote>
```

### Basics
```sh
git status
git diff <file>
git add <file>
git commit -m "<message>"
git push
git pull
```
*`git pull` is equivalent to `git fetch && git merge`.*

### Stage/Unstage
```sh
# Discard all before staging
git checkout .
# Discard line by line before staging
git checkout -p
# Discard untracked files
git clean -f
# Stage all the changes
git add .
# Stage the desired lines
git add -p
# Unstage all
git reset @ .
```

### Commits
```sh
# List commits (see the tree with --graph)
git log
# Fix last commit message
git commit --amend -m "<fixed_message>"
# Stage more lines to the last commit
git commit --amend --no-edit
# If the commit is already pushed
git push -f
# Revert with a new commit
git revert <commit>
# Reset to change commit message
git reset --soft <commit>
# Reset to change files to commit
git reset <commit>
# Reset to remove the commit from everywhere
git reset --hard <commit>
# Or number of commits to remove
git reset --hard HEAD~<numcommints>
# Force push
git push origin HEAD --force
# Cherry-pick a commit from somewhere else to current branch
git cherry-pick <commit>
# Ignore already committed file, then add it to .gitignore
git rm --cached <file>
```
*Create a branch from a specific commit and give a name:
`git checkout <commit> && git checkout -b <branch>`.*

*`git reset --hard <commit>` can be replaced with `git reset --hard HEAD~<number>`, where `<number>` is the number of commits to go back.*

*To co-author a commit add `co-authored-by: <username> <<email>>` to the commit message.*

*`HEAD` is equivalent to `@`*

### Branches
```sh
# List branches
git branch -a
# Create and switch to the new one ()
git checkout -b <branch>
# Switch of branch
git checkout <branch>
# Switch to the previous branch
git checkout -
# Rename a branch
git branch -m <old> <new>
# Locally ( -D force delete)
git branch -d <branch>
# Remove branch remotely
git push origin  --delete <old>
# In the first push
git push -u origin <branch>
# pull keeping the commits to be pushed on top of the remote tree
git pull --rebase origin <master>
# avoid overwriting commits after rebase
git push --force-with-lease
# Merge into the current branch (usually master) the specified one
git merge <branch>
# Rebase into the current branch the specified one (usually master)
git rebase <branch>
```
*`git checkout -b <branch>` is equivalent to `git branch <branch> && git checkout <branch>`.*

*`git pull --rebase` is equivalent to `git fetch && git rebase`*


### Stash
```sh
# Save
git stash
# Use and remove newest
git stash pop <stash>
# Use and keep newest
git stash apply <stash>
git stash list
# Drop newest
git stash drop <stash>
git stash clear
```

### `.gitignore`
It is a text file that tells Git which files or folders has to be ignored in a project. [Git ignore patterns](https://www.atlassian.com/git/tutorials/saving-changes/gitignore#git-ignore-patterns) may be helpful for writing your own `.gitignore` file.

The **local** `.gitignore` file is usually placed in the root directory of the project and the **global** one can be created as follow:
```sh
touch ~/.gitignore_global
git config --global core.excludesfile ~/.gitignore_global
```

### Releases
```sh
git tag
# Show releases with comments
git tag -l -n1
git tag <version>
# Release version with comment
git tag -a <version> -m '<message>'
```

### Submodules
#### Create submodules
```sh
# Create a new one
git submodule add -b <branch> <url>
git submodule init
# Create submodule
git submodule add <url>
git submodule init
# Download an existing one
git clone recursive <url>
# If previously --recursive not used
git submodule update --init --recursive
```
#### Upload changes
```sh
# Push to the submodule
git add <submodule>/<file>
git commit
git push origin HEAR:master
# Push to the repository
git add <submodule>
git commit
git push
```
#### Download changes
```sh
# Pull from the repository
git pull --recurse-submodules
# Pull from each submodule
git submodule update --recursive --remote
```
