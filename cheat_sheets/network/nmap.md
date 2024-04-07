# nmap cheat sheet

```sh
sudo apt install nmap
```

## Table of contents

* [Ping scanning](#ping-scanning)
* [Port scanning](#port-scanning)
* [Additional useful commands](#additional-useful-commands)

### Ping scanning

```sh
nmap -sn <IP_range>
nmap -sn <IP>
```

### Port scanning

```sh
# TCP SYN scan, a bit less reliable than the TCP and UDP scan
nmap -sS <IP/IP_range>
# TCP connect scan
nmap -sT <IP/IP_range>
# UDP scan
nmap -sU <IP/IP_range>
# SCTP INIT scanning external network
nmap -sY <IP/IP_range>
# TCP NULL regardless of the firewall
nmap -sS <IP/IP_range>
# Automatically the most popular ports
nmap --top-ports <25> <IP>
```

### Additional useful commands

```sh
# help
nmap -h
# colorize output with grc before any nmap command
sudo apt install grc
```
