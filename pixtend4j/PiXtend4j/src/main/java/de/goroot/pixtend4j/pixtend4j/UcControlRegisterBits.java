/*
 * Copyright (C) 2015 Michael Kolb <dev(at)db1smk(dot)com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package de.goroot.pixtend4j.pixtend4j;

/**
 * Bits of the Microcontroller-Control register
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
public enum UcControlRegisterBits {

    /**
     * Set this bit to enable watchdog in automatic mode
     */
    WATCHDOG_ENABLE(0),
    /**
     * Set this to exit init state and enter automatic mode
     */
    RUN_BIT(4);

    private final int bitIndex;

    UcControlRegisterBits(int bitIndex) {
        this.bitIndex = bitIndex;
    }

    int getBitIndex() {
        return bitIndex;
    }
}
