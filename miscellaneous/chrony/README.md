# how to install chrony for syncing time on drones

## README

Chrony **should and will not do automatic step synchronization**, that will break ROS. If you want to do a step sync (the time difference is too large), issue it manually by:
```bash
sudo chronyc -a makestep
```

BTW, the computers should (probably) be in the same timezone.

## Possible system's clock rate change

Chrony synchronizes the time by adjusting the system's clock rate by up to ~8% (by default). This means that if chrony is active and the declared time server is not in sync (the makestep command has not been called), use of chrony leads to nonnegligible constant change of system's clock rate. Be aware that if chrony is installed, it will start automatically at boot whenever NTP-based automatic clock synchronization is enabled. So do not forget to disable chrony or NTP-based time synchronization if you do not plan to use it. To disable NTP-based synchronization, run:
```bash
sudo timedatectl set-ntp off
```
To temporary disable chrony, run:
```bash
sudo service chrony restart
```

## install chrony client

```bash
sudo apt-get -y install chrony
sudo cp chrony-client.conf /etc/chrony/chrony.conf
sudo vim /etc/chrony/chrony.conf
```
Edit the lookup server by adding a new line to list of NTP servers.
```
server [SERVER IP ADDRESS - e.g. 192.168.69.5/SERVER HOSTNAME] offline iburst
```
Make sure to have the `SERVER HOSTNAME` in the `/etc/hosts` if you want to use the hostname.

```bash
sudo service chrony restart
```

## how to install chrony server

```bash
sudo apt-get -y install chrony
sudo cp chrony-server.conf /etc/chrony/chrony.conf
sudo vim /etc/chrony/chrony.conf
```
Edit the `allow` ip address in the configuration file that defines the server's accessibility by NTP clients as follows
```
allow [SERVER IP ADDRESS (replace the last digit with zero) - e.g. 192.168.69.0.]/24
```
```bash
sudo service chrony restart
```

## testing chrony

```bash
chronyc tracking
chronyc sources
```
