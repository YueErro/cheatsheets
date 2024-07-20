# ssh cheat sheet

```sh
# Bash
sudo apt-get install openssh-client
sudo apt-get install openssh-server
```

```powershell
# Powershell (admin)
Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
# Start the sshd service manually
Start-Service sshd
# Start the sshd service automatically
Set-Service -Name sshd -StartupType 'Automatic'
```

## Table of contents

- [ssh cheat sheet](#ssh-cheat-sheet)
  - [Table of contents](#table-of-contents)
    - [Connection](#connection)
    - [Data transfer](#data-transfer)
    - [Useful tips](#useful-tips)
      - [ssh keys](#ssh-keys)
      - [Simplify connection](#simplify-connection)

### Connection

```sh
# If there is no ping it won't connect
ping <IP/hostname>
# Stop ping
ctrl + c
# To ping just X times
ping -c <5> <IP/hostname>
# Connect
ssh <username>@<IP>
# If the dns is able to resolve
ssh <username>@<hostname>
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
scp <file> <username>@<IP/hostname>:<remote_path>
# From remote to local
scp <username>@<IP/hostname>:<remote_path> <local_path>
```

### Useful tips

#### ssh keys

```sh
# Generate a new public/private rsa key pair
ssh-keygen
# Use public key to authenticate and do not ask again for the password
ssh-copy-id -i $HOME/.ssh/id_rsa.pub <username>@<IP/hostname>
# Connect as usual
ssh <username>@<IP/hostname>
```

#### Simplify connection

```sh
vi ~/.ssh/config
```

```vim
Host <name>
  Hostname <IP/hostname>
  User <username>
  IdentityFile <$HOME/.ssh/id_rsa.pub>
  Port <22>
  RemoteCommand <command> && bash
  RequestTTY yes
```

```sh
ssh <name>
```
