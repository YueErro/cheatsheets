# ssh cheat sheet
```sh
sudo apt-get install openssh-client
sudo apt-get install openssh-server
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
ssh <username>@<IP>
# Disconnect
exit
# or
ctrl + d
```
### Data transfer
Common `scp` flags:
* `-P`: Specifies the remote host ssh port
* `-C`: Forces to compress the data sent to the destination machine
* `-r`: Copies directories recursively

```sh
# From local to remote
scp <file> <username>@<IP>:<remotepath>
# From remote to local
scp <username>@<IP>:<remotepath> <localpaht>
```

### Useful tips
#### ssh keys
```sh
# Generate a new public/private rsa key pair
ssh-keygen
# Use public key to authenticate and do not ask again for the password
ssh-copy-id -i $HOME/.ssh/id_rsa.pub <username>@<IP>
# Connect as usual
ssh <username>@<IP>
```

#### Simplify connection
```sh
vi ~/.ssh/config
```
```sh
Host <name>
  Hostname <IP>
  User <username>
  IdentityFile <$HOME/.ssh/id_rsa>
  Port <22>
```
```sh
ssh <name>
```
