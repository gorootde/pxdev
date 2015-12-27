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
import static de.goroot.pixtend4j.pixtend4j.AnalogInReferenceVoltage.VOLT_10;
import static de.goroot.pixtend4j.pixtend4j.AnalogInReferenceVoltage.VOLT_5;
import static de.goroot.pixtend4j.pixtend4j.SensorType.DHT11;
import static de.goroot.pixtend4j.pixtend4j.SensorType.DHT22;
import static de.goroot.pixtend4j.pixtend4j.SerialHardwareMode.RS232;
import static de.goroot.pixtend4j.pixtend4j.SerialHardwareMode.RS485;

/**
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
class PiXtendImpl implements PiXtend {

    private final JnaInterface nativeLib;

    PiXtendImpl() {
        nativeLib = (JnaInterface) Native.loadLibrary("pixtend", JnaInterface.class);
        nativeLib.Spi_Setup(0);
        nativeLib.Spi_Setup(1);
    }

    /**
     * Set all digital outputs. Each output state is set to the corresponding
     * bit value
     *
     * @param value
     */
    @Override
    public void setDigitalOutputs(byte value) {
        nativeLib.Spi_Set_Dout(value);
    }

    /**
     * Get all digital inputs. Each input state is reflected in the
     * corresponding bit value
     *
     * @return
     */
    @Override
    public int getDigitalInputs() {
        return nativeLib.Spi_Get_Din();
    }

    /**
     * Get a single digital input
     *
     * @param channel Input index (0-7)
     * @return True if the input is high, false otherwise
     */
    @Override
    public boolean getDigitalInput(int channel) {
        if (channel < 0 || channel > 7) {
            throw new IllegalArgumentException("channel has to be between 0 and 7");
        }
        return isBitSet(getDigitalInputs(), channel);
    }

    /**
     * Get single analog input
     *
     * @param channel Input index (0-3)
     * @return The value of the analog input
     */
    @Override
    public short getAnalogInput(int channel) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("channel has to be between 0 and 3");
        }
        return nativeLib.Spi_Get_Ain(channel);
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
    public double getAnalogInputVoltage(int channel, AnalogInReferenceVoltage jumperSetting) {
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
    public double getAnalogInputCurrent(int channel) {
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
    public void setAnalogInputConrolRegisters(byte register1, byte register2) {
        nativeLib.Spi_Set_AiControl(register1, register2);
    }

    /**
     * Set analog output
     *
     * @param channel Analog output index (0-1)
     * @param value Value to set
     */
    @Override
    public void setAnalogOutput(int channel, short value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("index has to be between 0 and 1");
        }
        if (value < 0 || value > 1023) {
            throw new IllegalArgumentException("value has to be between 0 and 1023");
        }
        nativeLib.Spi_Set_Aout(channel, value);
    }

    /**
     * Set all relays states
     *
     * @param values (each bit represents a single relays state)
     */
    @Override
    public void setRelaisValues(byte values) {
        nativeLib.Spi_Set_Relays(values);
    }

    /**
     * Read all GPIO values
     *
     * @return each bit represents a GPIO state
     */
    @Override
    public byte getGpioValues() {
        return (byte) nativeLib.Spi_Get_Gpio();
    }

    /**
     * Set all GPIO values
     *
     * @param values each bit represents a GPIO state
     */
    @Override
    public void setGpioValues(byte values) {
        nativeLib.Spi_Set_Gpio(values);
    }

    /**
     * Set GPIO control register
     *
     * @param register
     */
    @Override
    public void setGpioControlRegister(byte register) {
        nativeLib.Spi_Set_GpioControl(register);
    }

    /**
     * Set servo value
     *
     * @param channel Servo channel to set the value (0-1)
     * @param value Value to set
     */
    @Override
    public void setServoValue(int channel, byte value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("Channel has to be between 0 and 1");
        }
        nativeLib.Spi_Set_Servo(channel, value);
    }

    /**
     * Set PWM value of a channel
     *
     * @param channel Channel to set (0-1)
     * @param value Value to set the channel to (0-65535)
     */
    @Override
    public void setPwmValue(int channel, short value) {
        if (channel < 0 || channel > 1) {
            throw new IllegalArgumentException("Channel has to be between 0 and 1");
        }

        if (value < 0 || value > 65535) {
            throw new IllegalArgumentException("Value has to be between 0 and 65535");
        }

        nativeLib.Spi_Set_Pwm(channel, value);
    }

    /**
     * Set the PWM control registers
     *
     * @param register1
     * @param register2
     * @param register3
     */
    @Override
    public void setPwmControlRegisters(byte register1, byte register2, byte register3) {
        nativeLib.Spi_Set_PwmControl(register1, register2, register3);
    }

    /**
     * Set the serial hardware mode
     *
     * @param mode RS232 or RS485
     */
    @Override
    public void setSerialHardwareMode(SerialHardwareMode mode) {
        switch (mode) {
            case RS232:
                nativeLib.Change_Serial_Mode((byte) 0);
            case RS485:
            default:
                nativeLib.Change_Serial_Mode((byte) 1);
        }
    }

    /**
     * Reset the microcontroller
     */
    @Override
    public void resetUc() {
        nativeLib.Spi_uC_Reset();
    }

    /**
     * Get the microcontroller status register
     *
     * @return
     */
    @Override
    public int getUcStatusRegister() {
        return nativeLib.Spi_Get_uC_Status();
    }

    /**
     * Get the microcontroller's firmware version
     *
     * @return
     */
    @Override
    public String getUcVersion() {
        short rawVersion = nativeLib.Spi_Get_uC_Version();
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
    public void setRaspberryPiStatusRegister(byte value) {
        nativeLib.Spi_Set_RaspStat(value);
    }

    /**
     * Read temperature sensor
     *
     * @param channel Channel (0-3)
     * @param sensorType Type of sensor
     * @return temperature value
     */
    @Override
    public double readTemperature(int channel, SensorType sensorType) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("Channel has to be between 0 and 3");
        }
        short value = nativeLib.Spi_Get_Temp(channel);
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
    public double readHumidity(int channel, SensorType sensorType) {
        if (channel < 0 || channel > 3) {
            throw new IllegalArgumentException("Channel has to be between 0 and 3");
        }
        short value = nativeLib.Spi_Get_Hum(channel);
        return convertTempHumSensorValue(value, sensorType);
    }

    private static boolean isBitSet(int value, int index) {
        return (value & (1L << index)) != 0;
    }

    private static double convertTempHumSensorValue(short value, SensorType sensorType) {

        switch (sensorType) {
            case DHT11:
                return value / 10.0;
            case DHT22:
            default:
                return (value >> 8) * 1.0;
        }
    }

}
