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

import com.sun.jna.Native;

/**
 * Baseclass for actual PiXtend interfacing implementations. Input value range
 * checks take place here
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
abstract class AbstractPiXtendImpl implements PiXtend {

    private static final int UC_CONTROL_BIT_RUN_INDEX = 4;
    private static final int UC_CONTROL_BIT_WATCHDOG_INDEX = 0;

    private final JnaInterface nativeLib;

    AbstractPiXtendImpl() {
        nativeLib = (JnaInterface) Native.loadLibrary("pixtend", JnaInterface.class);
        nativeLib.Spi_Setup(0);
        nativeLib.Spi_Setup(1);
    }

    protected JnaInterface getNativeLib() {
        return nativeLib;
    }

    @Override
    public final void setUcControlRegister(UcControlRegisterBits... bits) {
        byte register = 0;
        for (UcControlRegisterBits bit : bits) {
            register = setBit(register, bit.getBitIndex());
        }
        writeUcControlRegister(register);
    }

    /**
     * Get a single digital input
     *
     * @param channel Input index (0-7)
     * @return True if the input is high, false otherwise
     */
    @Override
    public final boolean getDigitalInput(int channel) {
        if (channel < 0 || channel > 7) {
            throw new IllegalArgumentException("channel has to be between 0 and 7");
        }
        return isBitSet(getDigitalInputs(), channel);
    }

    private static boolean isBitSet(int value, int index) {
        return (value & (1L << index)) != 0;
    }

    protected static byte setBit(byte b, int index) {
        return (byte) (b | (1 << index));
    }

    protected static byte unsetBit(byte b, int index) {
        return (byte) (b & ~(1 << index));
    }

    protected static double convertTempHumSensorValue(short value, SensorType sensorType) {

        switch (sensorType) {
            case DHT11:
                return value / 10.0;
            case DHT22:
            default:
                return (value >> 8) * 1.0;
        }
    }

    /**
     * Get single analog input
     *
     * @param channel Input index (0-3)
     * @return The value of the analog input
     */
    @Override
    public final short getAnalogInput(int channel) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("channel has to be between 0 and 3");
        }
        return readAnalogIn(channel);
    }

    /**
     * Set all digital outputs. Each output state is set to the corresponding
     * bit value
     *
     * @param value
     */
    @Override
    public final void setDigitalOutputs(byte value) {
        writeDigitalOutputs(value);
    }

    /**
     * Get all digital inputs. Each input state is reflected in the
     * corresponding bit value
     *
     * @return
     */
    @Override
    public final int getDigitalInputs() {
        return readDigitalInputs();
    }

    /**
     * Read analog input voltage
     *
     * @param channel Channel to read the voltage from (0-3)
     * @param jumperSetting Jumper setting of the corresponding analog input
     * jumper
     * @return Voltage
     */
    @Override
    public final double getAnalogInputVoltage(int channel, AnalogInReferenceVoltage jumperSetting) {
        short value = getAnalogInput(channel);
        switch (jumperSetting) {
            case VOLT_10:
                return value * 10.0 / 1024.0;
            case VOLT_5:
            default:
                return value * 5.0 / 1024.0;
        }
    }

    /**
     * Read analog input current
     *
     * @param channel Index of the analog input (2-3)
     * @return Current in milliamps
     */
    @Override
    public final double getAnalogInputCurrent(int channel) {
        if (channel < 2 || channel > 3) {
            throw new UnsupportedOperationException("current is only available for channels 2 and 3");
        }
        return getAnalogInput(channel) * 0.024194115990990990990990990991;

    }

    /**
     * Set analog input control registers
     *
     * @param register1
     * @param register2
     */
    @Override
    public final void setAnalogInputControlRegisters(byte register1, byte register2) {
        writeAnalogInputControlRegisters(register1, register2);
    }

    /**
     * Set analog output
     *
     * @param channel Analog output index (0-1)
     * @param value Value to set
     */
    @Override
    public final void setAnalogOutput(int channel, short value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("index has to be between 0 and 1");
        }
        if (value < 0 || value > 1023) {
            throw new IllegalArgumentException("value has to be between 0 and 1023");
        }
        writeAnalogOut(channel, value);
    }

    /**
     * Set all relays states
     *
     * @param values (each bit represents a single relays state)
     */
    @Override
    public final void setRelaisValues(byte values) {
        writeRelaisValues(values);
    }

    /**
     * Read all GPIO values
     *
     * @return each bit represents a GPIO state
     */
    @Override
    public final byte getGpioValues() {
        return (byte) readGpio();
    }

    /**
     * Set all GPIO values
     *
     * @param values each bit represents a GPIO state
     */
    @Override
    public final void setGpioValues(byte values) {
        writeGpioValues(values);
    }

    /**
     * Set GPIO control register
     *
     * @param register
     */
    @Override
    public final void setGpioControlRegister(byte register) {
        writeGpioControlRegister(register);
    }

    /**
     * Set servo value
     *
     * @param channel Servo channel to set the value (0-1)
     * @param value Value to set
     */
    @Override
    public final void setServoValue(int channel, byte value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("Channel has to be between 0 and 1");
        }
        writeServoValue(channel, value);
    }

    /**
     * Set PWM value of a channel
     *
     * @param channel Channel to set (0-1)
     * @param value Value to set the channel to (0-65535)
     */
    @Override
    public final void setPwmValue(int channel, short value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("Channel has to be between 0 and 1");
        }

        if (value < 0 || value > 65535) {
            throw new IllegalArgumentException("Value has to be between 0 and 65535");
        }

        writePwmValue(channel, value);
    }

    /**
     * Set the PWM control registers
     *
     * @param register1
     * @param register2
     * @param register3
     */
    @Override
    public final void setPwmControlRegisters(byte register1, byte register2, byte register3) {
        writePwmControlRegisters(register1, register2, register3);
    }

    /**
     * Set the serial hardware mode
     *
     * @param mode RS232 or RS485
     */
    @Override
    public final void setSerialHardwareMode(SerialHardwareMode mode) {
        switch (mode) {
            case RS232:
                writeSerialMode(0);
            //getNativeLib().Change_Serial_Mode((byte) 0);
            case RS485:
            default:
                writeSerialMode(1);
        }
    }

    /**
     * Reset the microcontroller
     */
    @Override
    public final void resetUc() {
        getNativeLib().Spi_uC_Reset();
    }

    /**
     * Get the microcontroller status register
     *
     * @return
     */
    @Override
    public final int getUcStatusRegister() {
        return readUcStatusRegister();
    }

    /**
     * Get the microcontroller's firmware version
     *
     * @return
     */
    @Override
    public final String getUcVersion() {
        short rawVersion = readUcVersion();
        int major = rawVersion >> 8;
        int minor = rawVersion & 0xFF;
        return String.format("%d.%d", major, minor);
    }

    /**
     * Set the status register value
     *
     * @param value
     */
    @Override
    public final void setRaspberryPiStatusRegister(byte value) {
        writeRpiStatusRegister(value);
    }

    /**
     * Read temperature sensor
     *
     * @param channel Channel (0-3)
     * @param sensorType Type of sensor
     * @return temperature value
     */
    @Override
    public final double readTemperature(int channel, SensorType sensorType) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("Channel has to be between 0 and 3");
        }
        short value = readTemp(channel);
        return convertTempHumSensorValue(value, sensorType);
    }

    /**
     * Read temperature sensor humidity
     *
     * @param channel Channel (0-3)
     * @param sensorType Type of sensor
     * @return temperature value
     */
    @Override
    public final double readHumidity(int channel, SensorType sensorType) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("Channel has to be between 0 and 3");
        }
        short value = readHum(channel);
        return convertTempHumSensorValue(value, sensorType);
    }

    protected abstract int readDigitalInputs();

    protected abstract short readAnalogIn(int channel);

    protected abstract void writeDigitalOutputs(byte value);

    protected abstract void writeAnalogInputControlRegisters(byte register1, byte register2);

    protected abstract void writeAnalogOut(int channel, short value);

    protected abstract void writeRelaisValues(byte values);

    protected abstract int readGpio();

    protected abstract void writeGpioValues(byte values);

    protected abstract void writeGpioControlRegister(byte register);

    protected abstract void writeServoValue(int channel, byte value);

    protected abstract void writePwmValue(int channel, short value);

    protected abstract short readHum(int channel);

    protected abstract short readTemp(int channel);

    protected abstract void writeRpiStatusRegister(byte value);

    protected abstract short readUcVersion();

    protected abstract int readUcStatusRegister();

    protected abstract void writeSerialMode(int i);

    protected abstract void writePwmControlRegisters(byte register1, byte register2, byte register3);

    protected abstract void writeUcControlRegister(byte register);
}
