# NUClearNet

This module is responsible for connecting the system to the NUClear Network.
In the configuration you must set a name, broadcast address and port.
By setting these three the network will connect and be able to communicate with other NUClearNet systems that are on the same network.

## Name

If the name is left as an empty string in the configuration, the system will use the current host name as the name of the NUClearNet node.

## Address

The NUClearNet system can take an IP address or a dns name as the address. The address is able to resolve to either IPv4 or IPv6 address.
There are three main ways you can configure the network and use it.

### Unicast Mode

In Unicast mode, you connect from this device to another device on a specific other system. This address may also be `127.0.0.1` if you wish to only connect to other NUClearNet binaries on the same system.
In Unicast mode, you set the IP address here to the IP address of the other system you wish to connect to and on that system set the IP address of this system.
This form of networking is useful when you are debugging and wish to make a small isolated NUClearNet system.

### Broadcast Mode

In broadcast mode, you set the address to the broadcast IP of your network (e.g. `192.168.1.255` for the network `192.168.1.0/24`). In this mode you will connect to all other systems that are running on that same network. In general you should prefer multicast mode over this mode so that you only send packets to systems that are specifically part of the NUClearNet node rather than sending to all computers on the network and using up the bandwidth.

### Multicast Mode

In multicast mode you set either an IPv4 or IPv6 multicast address. Then provided your network correctly routes multicast packets it will connect to all other NUClearNet nodes that share the same multicast address. The default address for this is `239.226.152.162`

## Port

The default port for NUClearNet is 7447. You can use different ports if you want to make several distinct NUClearNet networks on the same device.
