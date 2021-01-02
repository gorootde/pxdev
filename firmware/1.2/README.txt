/*
# This file is part of the PiXtend(R) Project.
#
# For more information about PiXtend(R) and this program,
# see <http://www.pixtend.de> or <http://www.pixtend.com>
#
# Copyright (C) 2014-2015 Christian Strobel, Tobias Gall
# Qube Solutions UG (haftungsbeschränkt), Luitgardweg 18
# 71083 Herrenberg, Germany 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

PiXtend v1.2 controller firmware & source:
------------------------------------------

This package provides the sourcecode and firmware for the
microcontroller on the PiXtend V1.2 board. The firmware file can be 
flashed to the microcontroller by avrdude.

For more Information, please visit our homepage and download
the Application Note APP-PX-540_Microcontroller_Firmware_Update:
http://www.pixtend.de/pixtend/downloads/

The FUSE-Bits have to be set to the following values:
FUSE_LOW: 0x3F
FUSE_HIGH: 0xC9


This package contains the following files:
------------------------------------------

LICENSE.txt		Copy of the GPL v3 license
main.c			Sourcecode
main.hex		Firmware file
Makefile		Makefile for avrdude toolchain
PiXtend.pnproj		Projectfile for Programmers Notepad (WinAVR toolchain)


Release History:
----------------
12.4 - Dez. 2014: First release for PiXtend V1.2 Board
12.5 - Nov. 2015: Imporved DHT11/DHT22/AM2302 support for long wires up to 25 meters
