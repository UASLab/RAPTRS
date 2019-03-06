# RAPTRS

## Getting Started

### Development Environment
The following steps are used to create a development environment:
1. Install [Debian Stretch](https://www.debian.org/) to a Virtual Machine (VM) or development computer.
2. Ensure that [build-essential](https://packages.debian.org/stretch/build-essential) and [crossbuild-essential-armhf](crossbuild-essential-armhf) are installed.
3. Clone this [repository](https://github.com/bolderflight/RAPTRS) to your VM or development computer.
4. Install [Arudino](https://www.arduino.cc/en/Main/Software) and [Teensyduino](https://www.pjrc.com/teensy/td_download.html)

### BeagleBone Black Image
1. Download the BeagleBone Black [image](https://debian.beagleboard.org/images/bone-debian-9.5-iot-armhf-2018-10-07-4gb.img.xz).
2. Install [xz-utils](https://packages.debian.org/stretch/xz-utils) and extract the compressed image.
```
$ xz -d bone-debian-9.5-iot-armhf-2018-10-07-4gb.img.xz
```
3. Insert a micro-sd and write the image to the micro-sd card, where /dev/sdX is the drive for your micro-sd. You can use a utility like [GParted](https://gparted.org/) to inspect your machine's drives and ensure that you are writing to the correct location.
```
$ sudo dd if=bone-debian-9.5-iot-armhf-2018-10-07-4gb.img of=/dev/sdX
```
4. Insert the micro-sd in the BeagleBone Black and power on the device while holding down the "User Boot" button. Power can be supplied by plugging the BeagleBone Black into your computer with a USB micro plug. You should see all 4 LEDs lightup before lighting sequentially. This is flashing the image to the BeagleBone Black EMMC non-volatile memory. When the lights all turn off, you can remove the micro-sd and boot the BeagleBone Black.

![BeagleBone Black User Boot](https://cdn-learn.adafruit.com/assets/assets/000/008/680/small240/beaglebone_BeagleBoneBlack.jpeg)

5. Once the BeagleBone Black has booted, you should see it connect to your development computer as a network device. SSH to the BeagleBone Black using a terminal using its default user name, _debian_, and IP address, _192.168.7.2_. The default password is: _temppwd_.
```
$ ssh debian@192.168.7.2
```

### Building Software
Use the makefile in /RAPTRS/software to build flight software binaries, which are built to /RAPTRS/software/bin. The following commands are available:
   * _make_: builds all of the available binaries
   * _make clean_: deletes all binaries and deletes cached object files
   * _make flight_: builds the flight software for the BeagleBone Black
   * _make datalog_: builds the BeagleBone Black datalog-server software
   * _make telem_: builds the soc telem-server software
   * _make fmu_: builds the fmu software
   * _make node_: builds the node software
   * _make upload_fmu_: uploads the fmu software
   * _make upload_node_: uploads the node software
   
Additionally, within /RAPTRS/software/src there is an Arduino program called _write-addr_, which writes the BFS-Bus address for the FMU and Nodes using the Arduino serial monitor and following the prompts. This must be run prior to uploading FMU or Node software.

## Feedback
Pull requests improving our software or documentation are always appreciated.


