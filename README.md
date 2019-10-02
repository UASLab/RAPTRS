# RAPTRS

## Getting Started

### Development Environment
The following steps are used to create a development environment:
1. Install [Debian Stretch](https://www.debian.org/) to a Virtual Machine (VM) or development computer.
2. Ensure that [build-essential](https://packages.debian.org/stretch/build-essential) and [crossbuild-essential-armhf](https://packages.debian.org/buster/crossbuild-essential-armhf) are installed.
3. Clone this [repository](https://github.com/bolderflight/RAPTRS) to your VM or development computer.
4. Install [Arudino](https://www.arduino.cc/en/Main/Software) and [Teensyduino](https://www.pjrc.com/teensy/td_download.html)

### BeagleBone Black Image
1. Download the BeagleBone Black [image](https://www.dropbox.com/s/plqvbqe4fgw76vp/SOC.img.xz?dl=0).
2. Install [xz-utils](https://packages.debian.org/stretch/xz-utils) and extract the compressed image.
```
$ xz -d SOC.img.xz
```
3. Insert a micro-sd and write the image to the micro-sd card, where /dev/sdX is the drive for your micro-sd. You can use a utility like [GParted](https://gparted.org/) to inspect your machine's drives and ensure that you are writing to the correct location.
```
$ sudo dd if=SOC.img of=/dev/sdX
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

While the FMU and Node software is flashed by _make upload_fmu_ and _make upload_node_, the flight and datalog-server software should be transfered to the BeagleBone Black using SFTP or SCP.

### Configuration
Aircraft configuration is done via a JSON file that is read by the BeagleBone Black and used to configure the aircraft sensors, sensor-processing and estimation algorithms, control laws, mission manager, and effectors. Details are [provided](https://github.com/bolderflight/RAPTRS/blob/master/CONFIGURATION.md), for each of the configurable items. A couple of examples are also provided for an [Ultra Stick 25e](https://www.dropbox.com/s/shd0q6is4jtnv1q/thor.json?dl=0) and a [quadrotor](https://www.dropbox.com/s/kqxwsg8gf16s80u/f450.json?dl=0) UAS.

### Flight
On the BeagleBone Black:
1. Setup the UARTS by running this [script](https://www.dropbox.com/s/4zfucbmtxqe7tgx/setup-uarts.sh?dl=0)
2. Start the datalog server
```
$ nohup ./datalog-server &
```
3. Start the flight code
```
$ nohup ./flight config.json &
```
4. You can check that the datalog file (_dataX.bin_) is growing and disconnect from the BeagleBone Black.
5. After the flight, download the datalog file and convert to HDF5 using 'bin2hdf_clo.py' in /RAPTRS/analysis-tools. Note that this script requires python3 and h5py installed.
```
$ python3 bin2hdf_clo.py data0.bin
```

## Feedback
Pull requests improving our software or documentation are always appreciated.


