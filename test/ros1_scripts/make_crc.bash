#!/bin/bash
echo -e "make_crc.bash: build and generate the crc code for sick_lidar_localization."
echo -e "Checkout https://github.com/madler/crcany.git"
if [ -d ../../../crcany ] ; then
  ./cleanup.bash
  pushd ../../../crcany
  source /opt/ros/melodic/setup.bash
  make
  make test
  ls src
  ./src/crc_test
  popd
  if ! [ -d ../../src/crc     ] ; then mkdir -p ../../src/crc     ; fi
  if ! [ -d ../../include/crc ] ; then mkdir -p ../../include/crc ; fi
  if [ -f ../../../crcany/src/crc16ccitt_false.h ] && [ -f ../../../crcany/src/crc16ccitt_false.c ] ; then
    cat ../../include/crc/crc_note.h           >  ../../src/crc/crc16ccitt_false.cpp
    cat ../../include/crc/crc_note.h           >  ../../include/crc/crc16ccitt_false.h
    cat ../../../crcany/src/crc16ccitt_false.c >> ../../src/crc/crc16ccitt_false.cpp
    cat ../../../crcany/src/crc16ccitt_false.h >> ../../include/crc/crc16ccitt_false.h
  else
    echo -e "make crcany failed: crc16ccitt_false.c or crc16ccitt_false.h not found."
  fi
else
  echo -e "Folder ../../../crcany not found"
  echo -e "Please checkout https://github.com/madler/crcany.git:"
  echo -e "  pushd ../../../"
  echo -e "  git clone https://github.com/madler/crcany.git"
  echo -e "  popd"
  echo -e "and retry."
fi
echo -e "make_crc.bash finished."

