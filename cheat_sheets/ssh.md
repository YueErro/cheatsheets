# ssh cheat sheet
```sh
sudo apt install openssh-client
sudo apt install openssh-server
```

## Table of contents
* [Connection](#Connection)
* [Data transfer](#Data-transfer)
* [Useful tips](#Useful-tips)
  * [ssh keys](#ssh-keys)
  * [Simplify connection](#Simplify-connection)

### Connection
```sh
# Connect
ssh <username>@<hostnameorip>
# Disconnect
exit
# or
ctrl + d
```
### Data transfer
```sh
# Use -r for a recursive copy of a folder
scp <file> <username>@<hostnameorip>:<path>
```
### Useful tips

#### ssh keys
```sh
# Generate a new public/private rsa key pair
ssh-keygen
# Use public key to authenticate
ssh-copy-id <username>@<hostnameorip>
# Connect as usual
ssh <username>@<hostnameorip>
```

#### Simplify connection
```sh
vi ~/.ssh/config
```
```sh
Host <name>
  Hostname <hostnameorip>
  User <username>
  IdentityFile <~/.ssh/id_rsa>
  Port <22>
```
```sh
ssh <name>
```
