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
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
class PiXtendManualModeImpl extends AbstractPiXtendImpl {

    @Override
    protected short readAnalogIn(int channel) {
        return getNativeLib().Spi_Get_Ain(channel);
    }

    @Override
    protected void writeDigitalOutputs(byte value) {
        getNativeLib().Spi_Set_Dout(value);
    }

    @Override
    protected int readDigitalInputs() {
        return getNativeLib().Spi_Get_Din();
    }

    @Override
    protected void writeAnalogInputControlRegisters(byte register1, byte register2) {
        getNativeLib().Spi_Set_AiControl(register1, register2);
    }

    @Override
    protected void writeAnalogOut(int channel, short value) {
        getNativeLib().Spi_Set_Aout(channel, value);
    }

    @Override
    protected void writeRelaisValues(byte values) {
        getNativeLib().Spi_Set_Relays(values);
    }

    @Override
    protected int readGpio() {
        return getNativeLib().Spi_Get_Gpio();
    }

    @Override
    protected void writeGpioValues(byte values) {
        getNativeLib().Spi_Set_Gpio(values);
    }

    @Override
    protected void writeGpioControlRegister(byte register) {
        getNativeLib().Spi_Set_GpioControl(register);
    }

    @Override
    protected void writeServoValue(int channel, byte value) {
        getNativeLib().Spi_Set_Servo(channel, value);
    }

    @Override
    protected void writePwmValue(int channel, short value) {
        getNativeLib().Spi_Set_Pwm(channel, value);
    }

    @Override
    protected short readHum(int channel) {
        return getNativeLib().Spi_Get_Hum(channel);
    }

    @Override
    protected short readTemp(int channel) {
        return getNativeLib().Spi_Get_Temp(channel);
    }

    @Override
    protected void writeRpiStatusRegister(byte value) {
        getNativeLib().Spi_Set_RaspStat(value);
    }

    @Override
    protected short readUcVersion() {
        return getNativeLib().Spi_Get_uC_Version();
    }

    @Override
    protected int readUcStatusRegister() {
        return getNativeLib().Spi_Get_uC_Status();
    }

    @Override
    protected void writeSerialMode(int i) {
        getNativeLib().Change_Serial_Mode((byte) 1);
    }

    @Override
    protected void writePwmControlRegisters(byte register1, byte register2, byte register3) {
        getNativeLib().Spi_Set_PwmControl(register1, register2, register3);
    }

    @Override
    protected void writeUcControlRegister(byte register) {
        getNativeLib().Spi_Set_UcControl(register);
    }

}
