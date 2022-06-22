# dpkg cheat sheet

## Table of contents
* [Install and Uninstall](#Install-and-Uninstall)
* [Additional information](#Additional-information)
* [Installed information](#Installed-information)

### Install and Uninstall
```sh
# install
sudo dpkg -i <.deb>
# remove everything
sudo dpkg --purge <deb>
# force whole removal
sudo dpkg --purge --force-all <deb>
```

### Additional information
```sh
# informaiton
dpkg -I <.deb>
# display content
dpkg -c <.deb>
# unzip
dpkg --extract <.deb> <name>
```

### Installed information
```sh
# list all
dpkg -l
# localize
dpkg -L <deb>
# status
dpkg -s <deb>
# search
dpkg -S <deb>
```