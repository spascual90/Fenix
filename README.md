# Fenix Autopilot
## What is Fenix Autopilot?
The goals of Fenix project are,
- to develop and test a prototype of open-source DIY navigation system for mini-cruisers.  
- to gather individual efforts to a joint solution.
- get users to adopt the system into their own boats with minimum technical knowledge.

## How to contribute?
https://emacua.fandom.com/wiki/EMacua_Wiki

## Released versions
- V0.1 Initial version

## How to start?
First of all integrate your hardware.
If you are a user, upload the SW to Arduino, the App to mobile and configure your autopilot.

If you are a developer, additionally you will have to setup a development environment.
## HW integration
You can find in SCHEMATICS folder,
- Build of materials: the basic shop list to build Fenix autopilot
- Fritzing file
- Guide on how to integrate the components
<img alt="README-e8ba2418.jpg" src="assets/README-e8ba2418.jpg" width="" height="" >

## Users: How to start?
### Uploading SW to Arduino
You can find in HEX folder,
- HEX Arduino file
- Xloader, an application to upload HEX to Arduino

### Installing Virtuino App
You can find in repository Virtuino-for-Fenix all elements required
- Enter into Play Store and install Virtuino Viewer
- Copy project file to the Virtuino folder in your mobile
- Configure Bluetooth

### Autopilot configuration
- Connect your laptop to Fenix autopilot USB.
- Open Arduino IDE or putty.exe available in Serial IF folder
- In putty select the Arduino settings
- Open a new terminal
- Follow autopilot information to configure your IMU and linear actuator
- All implemented messages are described in Serial IF folder. A checksum calculator is included.
- Configure other parameters of the autopilot

## Developers: How to start?
### Installation of development environment
Install Sloeber, the Eclipse Arduino IDE https://eclipse.baeyens.it

Downlad Fenix Project from Github https://github.com/spascual90/Fenix

Compile and upload sketch to Arduino
