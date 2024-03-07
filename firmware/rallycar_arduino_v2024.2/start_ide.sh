#!/bin/bash
ARDUINO_VERSION=2.3.2_Linux_64bit

if [[ ! -f ~/arduino-ide/arduino-ide ]]; then
  if [[ $(uname -m) != "x86_64" ]] ; then
    echo "ERROR: Auto installation of IDE is configured only for X86_64 systems. For your system, please manually download and extract Arduino to ~/arduino-ide/ folder, and re-run this script."
    exit -1
  fi
  wget -O arduino_ide.zip https://downloads.arduino.cc/arduino-ide/arduino-ide_${ARDUINO_VERSION}.zip
  unzip arduino_ide.zip -d ~/
  mv ~/arduino-ide_${ARDUINO_VERSION}/ ~/arduino-ide/
  rm arduino_ide.zip
fi

wget -O rosserial_arduino_lib.zip https://codeload.github.com/frankjoshua/rosserial_arduino_lib/zip/refs/heads/master
unzip -o rosserial_arduino_lib.zip -d ~/Arduino/libraries
mv ~/Arduino/libraries/rosserial_arduino_lib-master ~/Arduino/libraries/rosserial_arduino_lib
rm rosserial_arduino_lib.zip
~/arduino-ide/arduino-ide .
