/*
# This file is based on pxauto of the PiXtend(R) Project.
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

#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include <pixtend.h>

enum menu_page
{
	HOME = 0,
	DIN = 1,
	AIN = 2,
	GPIO = 3,
	DOUT = 4,
	AOUT = 5,
	PWM = 6,
	CTRL = 7,
	STAT = 8
};

int autoMode();

void update_header();
void update_menu();
void update_HOME();
void update_AIN();
void update_DIN();
void update_GPIO();
void update_AOUT();
void update_DOUT();
void update_PWM();
void update_CTRL();
void update_STAT();
