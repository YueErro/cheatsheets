# git cheat sheet

```sh
sudo apt-get install git
```

There is text-mode interface for Git called Tig which may be helpful:

```sh
sudo apt-get install tig
```

There is a graphical repository browser called Gitk which may be helpful as well:

```sh
sudo apt-get install gitk
```

## Table of contents

- [git cheat sheet](#git-cheat-sheet)
  - [Table of contents](#table-of-contents)
    - [Setup](#setup)
    - [Create repositories](#create-repositories)
    - [Remote](#remote)
    - [Basics](#basics)
    - [Stage/Unstage](#stageunstage)
    - [Commits](#commits)
      - [Commit messages](#commit-messages)
        - [Structure](#structure)
        - [Type](#type)
        - [Subject](#subject)
        - [Body](#body)
        - [Footer](#footer)
        - [Example](#example)
    - [Branches](#branches)
    - [Stash](#stash)
    - [`.gitignore`](#gitignore)
    - [Tags](#tags)
    - [Submodules](#submodules)
      - [Create submodules](#create-submodules)
      - [Upload changes](#upload-changes)
      - [Download changes](#download-changes)

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

*`git pull` over all subdirectories: `ls | xargs -I{} git -C {} pull`*

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
# Change author of the tag
git checkout <tag>
git tag -d <tag>
git commit --amend --no-edit --allow-empty --author="<username> <<e-mail>>"
export GIT_COMMITTER_DATE="$(git show --format=%aD | head -1)"; git tag <tag>
git push origin :refs/tags/<tag>
git push --tags
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
git reset --hard HEAD~<numcommits>
# Force push
git push origin HEAD --force
# Cherry-pick a commit from somewhere else to current branch
git cherry-pick <commit>
# Cherry-pick before committing in order to inspect and modify, the commit message will have been stored
git cherry-pick -n <commit>
# Ignore already committed file, then add it to .gitignore
git rm --cached <file>
```

*Create a branch from a specific commit and give a name:
`git checkout <commit> && git checkout -b <branch>`.*

*`git reset --hard <commit>` can be replaced with `git reset --hard HEAD~<number>`, where `<number>` is the number of commits to go back.*

*To co-author a commit add `co-authored-by: <username> <<email>>` to the commit message.*

*`HEAD` is equivalent to `@`*

#### Commit messages

[Udacity](https://udacity.github.io/git-styleguide/) urges students to refer to a Git Commit Message Style that we will try to follow.

##### Structure

A commit messages consists of three distinct parts separated by a blank line: the title, an optional body and an optional footer:

```md
type: subject

body

footer
```

##### Type

The type is contained within the title and can be one of these types:

- **feat**: a new feature
- **fix**: a bug fix
- **perf**: improve code performance
- **docs**: changes to documentation
- **style**: formatting, missing semi colons, etc; no code change
- **refactor**: refactoring production code
- **test**: adding tests, refactoring test; no production code change
- **chore**: updating build tasks, package manager configs, etc; no production code change

##### Subject

Subjects should be no greater than 50 characters, should begin with a capital letter and do not end with a period.

Use an imperative tone to describe what a commit does, rather than what it did

##### Body

Not all commits are complex enough to warrant a body, therefore it is optional and only used when a commit requires a bit of explanation and context. Use the body to explain the **what** and **why** of a commit, not the how.

When writing a body, the blank line between the title and the body is required and you should limit the length of each line to no more than 72 characters.

##### Footer

The footer is optional and is used to reference issue tracker IDs.

##### Example

```md
feat: Summarize changes in around 50 characters or less

More detailed explanatory text, if necessary. Wrap it to about 72
characters or so. In some contexts, the first line is treated as the
subject of the commit and the rest of the text as the body. The
blank line separating the summary from the body is critical (unless
you omit the body entirely); various tools like `log`, `shortlog`
and `rebase` can get confused if you run the two together.

Explain the problem that this commit is solving. Focus on why you
are making this change as opposed to how (the code explains that).
Are there side effects or other unintuitive consequences of this
change? Here's the place to explain them.

Further paragraphs come after blank lines.

 - Bullet points are okay, too

 - Typically a hyphen or asterisk is used for the bullet, preceded
   by a single space, with blank lines in between, but conventions
   vary here

If you use an issue tracker, put references to them at the bottom,
like this:

Resolves: #123
See also: #456, #789
```

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
# Remove local branches not existing remotely
git remote prune origin
# Remove local branches not existing remotely in every fetch/pull
git config --global fetch.prune true
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

### Tags

```sh
# Show tags with comments
git tag -l -n1
# Create local tag
git tag <version>
# Create tag with comment
git tag -a <version> -m '<message>'
# Push the tag remotely
git push origin tag <version>
```

### Submodules

#### Create submodules

```sh
# Create a new one
git submodule add -b <branch> <url>
# Clone submodules after having cloned the repository containing them
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
