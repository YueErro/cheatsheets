# tcpdump cheat sheet

```sh
sudo apt install tcpdump
```

## Table of contents

* [Configure permissions](#configure-permissions)
* [Capture packets](#capture-packets)
  * [Capture by host](#capture-by-host)
  * [Capture by port](#capture-by-port)
  * [Capture by protocol](#capture-by-protocol)
  * [Capture to a file](#capture-to-a-file)

### Configure permissions

```sh
# Create a group
sudo groupadd pcap
# Add your user to the group
sudo usermod -aG pcap $USER
# Change group
sudo chgrp pcap /usr/bin/tcpdump
# Set permissions
sudo chmod 750 /usr/bin/tcpdump
# Give the necessary permissions
sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/tcpdump
```

*The tcpdump might be in `/usr/sbin/tcpdump`.*

### Capture packets

```sh
# Show available interfaces
tcpdump -D
# Capture everything in a particular interface
tcpdump -i <interface>
# Show IP instead of name
tcpdump -i <interface> -n
# Capture a limit number of packets
tcpdump -i <interface> -c <5>
# Show human readable time instead of timestamp
tcpdump -i <interface> -l
# Show content in ASCII
tcpdump -i <interface> -A
# Show at the end of each packet
tcpdump -i <interface> -U
# More command options
tcpdump --help
```

#### Capture by host

```sh
# Capture a particular IP, either in the source or destination fields
tcpdump -i <interface> host <IP>
# Capture a particular IP in source
tcpdump -i <interface> src host <IP>
# Capture a particular IP in destination
tcpdump -i <interface> dst host <IP>
# Capture a particular IP in source and destination
tcpdump -i <interface> src and dst host <IP>
```

#### Capture by port

```sh
# Capture by particular port
tcpdump -i <interface> port <80>
# Capture a particular port in source
tcpdump -i <interface> src port <80>
# Capture a particular port in destination
tcpdump -i <interface> dst port <80>
# Capture a particular port in source and destination
tcpdump -i <interface> src and dst port <80>
```

#### Capture by protocol

```sh
# Capture by protocol
tcpdump -i <interface> <protocol>
```

*It can be combined with the other capture method using `and` and/or `or`*.

#### Capture to a file

```sh
# Write to a file
tcpdump -i <interface> -w <filepath>
# Read the capture file
tcpdump -r <filepath>
```
