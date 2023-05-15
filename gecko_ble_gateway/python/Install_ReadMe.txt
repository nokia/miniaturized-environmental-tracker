Raspberry Pi Bluetooth Install Instructions:

The following is copied from https://scribles.net/updating-bluez-on-raspberry-pi-from-5-43-to-5-50/

1.) Download Bluez User Space: http://www.bluez.org/download/
2.) tar xvf bluez-5.50.tar.xz
3.) sudo apt-get update
4.) sudo apt-get install libdbus-1-dev libglib2.0-dev libudev-dev libical-dev libreadline-dev -y
5.) cd bluez-5.50
6.) ./configure --prefix=/usr --mandir=/usr/share/man --sysconfdir=/etc --localstatedir=/var --enable-experimental
7.) make -j4
8.) sudo make install
9.) sudo reboot
10.) Check to see if it worked:
	bluetoothctl -v

Install VNC
1.) sudo apt-get update
2.) sudo apt-get install realvnc-vnc-server realvnc-vnc-viewer
3.) Menu > Preferences > Raspberry Pi Configuration > Interfaces > VNC Enable
4.) From VNC Server App:
	Menu > Options > Security 
		- Encryption Prefer On
		- VNC password
5.) Stop VNC Server
6.) Restart VNC Server (same as #3)

Install WWS Interface
1.) pip3 install pika

Bluetooth Manager (optional)
1.) sudo apt-get install d-feet

Onscreen Keyboard (optional)
1.) sudo apt-get install matchbook-keyboard

Install Bluetooth Dongle (I’m not really sure if this is necessary….)
1.) sudo apt-get install blueman   