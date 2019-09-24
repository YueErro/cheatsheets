# GitHub cheat sheet

```sh
sudo apt install git
```

## Table of contents
* [Setup](#Setup)
* [Create repositories](#Create-repositories)
* [Remote](#Remote)
* [Basics](#Basics)
* [Submodules](#Submodules)
  * [Create submodules](#Create-submodules)
  * [Upload changes](#Upload-changes)
  * [Download changes](#Download-changes)
* [Commits](#Commits)
* [Branches](#Branches)
* [Stash](#Stash)
* [Releases](#Releases)

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
# Add all the changes
git add .
git commit -m "<message>"
git push
git pull
```
*`git pull` is equivalent to `git fetch && git merge`.*

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
git submodule update --remote
```

### Commits
```sh
git log
# Revert with a new commit
git revert <commit>
# Reset to change commit message
git reset --soft <commit>
# Reset to change files to commit
git reset <commit>
# Reset to remove the commit from everywhere
git reset --hard <commit>
# Cherry-pick a commit from somewhere else to current branch
git cherry-pick <commit>
# Ignore already committed file, then add it to .gitignore
git rm --cached <file>
```
*Create a branch from a specific commit and give a name:
`git checkout <commit> && git checkout -b <branch>`.*

*`git reset --hard <commit>` can be replaced with `git reset --hard HEAD~<number>`, where `<number>` is the number of commits to go back.*

*To co-author a commit add `co-authored-by: <username> <<email>>` to the commit message.*

### Branches
```sh
git branch -a
# Create and switch to the new one
git checkout -b <branch>
# Locally ( -D force delete)
git branch -d <branch>
# Remotely (: force delete instead of --delete)
git push <remote> --delete <branch>
# Merge into the current branch (usually master) the specified one
git merge <branch>
# Rebase into the current branch the specified one (usually master)
git rebase <branch>
```
*`git checkout -b <branch>` is equivalent to `git branch <branch> && git checkout <branch>`.*


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

### Releases
```sh
git tag
# Show releases with comments
git tag -l -n1
git tag <version>
# Release version with comment
git tag -a <version> -m '<message'
```
