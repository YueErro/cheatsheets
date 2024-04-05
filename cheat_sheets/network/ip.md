# ip cheat sheet

## Table of contents

* [Addresses](#addresses)
* [Links](#links)
* [Routing table](#routing-table)
* [Neighbors](#neighbors)
* [Additional useful commands](#additional-useful-commands)

### Addresses

```sh
# Whole address information of all the devices
ip a
# Whole address information of a particular device
ip a ls <device>
# Only the IPv4 addresses
ip -4 a
# Only the IPv6 addresses
ip -6 a
# Add IP to an interface
ip a add <IP> dev <interface>
# Delete IP of an interface
ip a del <IP> dev <interface>
# More command options
ip a help
```

### Links

```sh
# Link layer information of all available devices
ip l
# Link layer information of a particular device
ip l ls <device>
# Statistics of all network interfaces
ip -s l
# Set network interface up
ip l set <interface> up
# Set network interface down
ip l set <interface> down
# More command options
ip l help
```

### Routing table

```sh
# Information of all the route entries
ip r
# Add a new entry in the routing table
ip r add <IP> dev <interface>
# Add a new rout via gateway
ip r add <IP> via <gatewayIP>
# Delete entry from the routing table
ip r del <IP> dev <interface>
# More command options
ip r help
```

### Neighbors

```sh
# List the neighbor device objects
ip n
```

For a more detailed information of the neighbor device objects `nmap` command can be used, see [nmap.md](https://github.com/YueErro/cheatsheets/blob/master/cheat_sheets/network/nmap.md)

### Additional useful commands

```sh
# Colorize ip command output
ip -c <command>
# More ip command options
ip help
# Get information of all the wireless networking interfaces
iwconfig
# More iwconfig command options
iwconfig --help
```