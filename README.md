# RAPTRS

## Getting Started

### Development Environment
The following steps are used to create a development environment:
1. Install [Debian Stretch](https://www.debian.org/) to a Virtual Machine (VM) or development computer.
2. Ensure that [build-essential](https://packages.debian.org/stretch/build-essential) and [crossbuild-essential-armhf](crossbuild-essential-armhf) are installed.
3. Clone this [repository](https://github.com/bolderflight/RAPTRS) to your VM or development computer.

### BeagleBone Black Image
1. Download the BeagleBone Black [Debian 9.5 IOT image](https://debian.beagleboard.org/images/bone-debian-9.5-iot-armhf-2018-10-07-4gb.img.xz).
2. Install [xz-utils](https://packages.debian.org/stretch/xz-utils) and extract the compressed image.
```
$ xz -d bone-debian-9.5-iot-armhf-2018-10-07-4gb.img.xz
```
3. Insert a micro-sd and write the image to the micro-sd card, where /dev/sdX is the drive for your micro-sd. You can use a utility like [GParted](https://gparted.org/) to inspect your machine's drives and ensure that you are writing to the correct location.
```
$ sudo dd if=bone-debian-9.5-iot-armhf-2018-10-07-4gb.img of=/dev/sdX
```
4. Insert the micro-sd in the BeagleBone Black and power on the device while holding down the "User Boot" button.
![BeagleBone Black User Boot](https://learn.adafruit.com/assets/8680 "BeagleBone Black User Boot")
5. Login using the default user name: _debian_ and password: _temppwd_.

## Feedback
Pull requests improving our software or documentation are always appreciated.


