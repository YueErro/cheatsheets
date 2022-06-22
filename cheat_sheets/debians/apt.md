# apt cheat sheet

## Table of contents
* [Update and Upgrade](#Update-and-Upgrade)
* [Search](#Search)
* [Install and Remove](#Install-and-Remove)
* [Clean](#Clean)

### Update and Upgrade
```sh
sudo apt-get update
# not to upgrade especific pkg
sudo apt-mark hold <pkg1> <pkg2>
# get a list of holded pkgs
sudo apt-mark showhold
# to undo the previous command
sudo apt-mark unhold <pkg1> <pkg2>
# upgrade without asking comfirmation --> -y
sudo apt-get upgrade <pkg1> <pkg2> -y
```

### Search
```sh
apt-cache search <pkg_desciption_term>
apt-cache pkgnames <pkg_term>
# metadata
apt-cache show <pkg>
# all dependencies
apt-cache showpkg <pkg>
# what it depends on
apt-cache depends <pkg>
# what depends on it
apt-cache rdepends <pkg>
```

### Install and Remove
```sh
# --reinstall only that pkg, not dependencies
sudo apt-get install <pkg1> <pkg2>
# without removeing its configuration files
sudo apt-get remove <pkg>
# removing configuration files as well
sudo apt-get purge <pkg>
```

### Clean
```sh
# clear out the local repository
sudo apt-get clean
# only the ones that can no longer be downloaded from their sources
sudo apt-get autoclean
# the dependencies of an already removed pkg
sudo apt-get autoremove
```
