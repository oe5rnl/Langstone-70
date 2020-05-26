#!/bin/bash
# Langstone Install script 
# Buster Version G4EML 27/01/20
# 70cm 500 kHz by oe5rnl


# Update the package manager
sudo dpkg --configure -a
sudo apt-get -y update

# Uninstall the apt-listchanges package to allow silent install of ca certificates (201704030)
# http://unix.stackexchange.com/questions/124468/how-do-i-resolve-an-apparent-hanging-update-process
sudo apt-get -y remove apt-listchanges

# -------- Upgrade distribution ------

# Update the distribution
sudo apt-get -y dist-upgrade


# Install the packages that we need
sudo apt-get -y install git
sudo apt-get -y install libxml2 libxml2-dev bison flex libcdk5-dev cmake
sudo apt-get -y install libaio-dev libusb-1.0-0-dev libserialport-dev libxml2-dev libavahi-client-dev 
sudo apt-get -y install gr-iio
sudo apt-get -y install gnuradio
sudo apt-get -y install raspi-gpio



# install WiringPi
cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb
cd ~

#install libiio
git clone https://github.com/analogdevicesinc/libiio.git
cd libiio
cmake ./
make all
sudo make install

cd ~
# Set auto login to command line.

sudo raspi-config nonint do_boot_behaviour B2

# install the Langstone Files

git clone https://github.com/oe5rnl/Langstone-70.git
cd Langstone
chmod +x build
chmod +x run
chmod +x stop
chmod +x update
./build


#make Langstone autostart on boot

if !(grep Langstone ~/.bashrc) then
  echo if test -z \"\$SSH_CLIENT\" >> ~/.bashrc 
  echo then >> ~/.bashrc
  echo /home/pi/Langstone-70/run >> ~/.bashrc
  echo fi >> ~/.bashrc
fi

#Configure the boot parameters

if !(grep lcd_rotate /boot/config.txt) then
  sudo sh -c "echo lcd_rotate=2 >> /boot/config.txt"
fi
if !(grep disable_splash /boot/config.txt) then
  sudo sh -c "echo disable_splash=1 >> /boot/config.txt"
fi
if !(grep global_cursor_default /boot/cmdline.txt) then
  sudo sed -i '1s,$, vt.global_cursor_default=0,' /boot/cmdline.txt
fi


#Reboot and start
sudo reboot




