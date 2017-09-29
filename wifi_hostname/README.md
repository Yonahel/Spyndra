# Set up Wifi Hostname
Tired of looking up IP address? This manual will help you set up an alias hostname. If everything works out, you will be able to ssh into Spyndra by a line of command.

## Prerequisites
Here we are using avahi-daemon for zero-configuration in raspberrypi. Check whether avahi-daemon is installed by the command.

```
  $ dpkg-query -W avahi-daemon  
  
  # EXPECTED OUTPUT:
  # avahi-daemon  0.6.31-4ubuntu1.1
```

## Installing

1. Download the script from Spyndra repository.

```
  $ git clone https://github.com/Yonahel/Spyndra.git
```

2. Run bash script by root directory.

```
  $ cd ~/Downloads/Spyndra/dev/
  $ sudo ./setup.sh
```

3. You will have to reboot the Raspberry Pi after installation. After reboot, check out the new hostname.

```
  $ hostname
  
  # EXPECTED OUTPUT: 
  # spyndra
```

4. After you reboot the Raspberry Pi, please ping spyndra.local to make sure everything is ready.

First make sure you computer and Raspberry Pi are sharing the same Wifi. Then ping spyndra.local.

```
  $ ping spyndra.local

  # EXPECTED OUTPUT
  # PING spyndra.local (192.168.0.104) 56(84) bytes of data.
  # 64 bytes from 192.168.0.104: icmp_seq=1 ttl=64 time=6.17 ms
  # 64 bytes from 192.168.0.104: icmp_seq=2 ttl=64 time=9.01 ms
  # 64 bytes from 192.168.0.104: icmp_seq=3 ttl=64 time=6.62 ms
```

## Examples
Here are a few examples of how `spyndra.local` makes your life easier.

1. SSH into Spyndra.

```
$ ./wifi_hostname/ssh_spyndra.sh
```

2. File transfer through scp.

```
$ ./wifi_hostname/scpfiles.sh SOURCE_FILE DESTINATION
```

Example 1: getting a file from Spyndra to your desktop.

```
$ ./wifi_hostname/scpfiles.sh /home/pi/.bashrc ~/Desktop
```

Example 2: sending a file to the desktop of Spyndra.

```
$ ./wifi_hostname/scpfiles.sh README.txt /home/pi/Desktop
```
