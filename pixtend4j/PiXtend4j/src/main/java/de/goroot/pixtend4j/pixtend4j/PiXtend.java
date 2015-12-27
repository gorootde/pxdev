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
 * Pixtend Interface
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
public interface PiXtend {

    /**
     * Get single analog input
     *
     * @param channel Input index (0-3)
     * @return The value of the analog input
     */
    public short getAnalogInput(int channel);

    /**
     * Read analog input current
     *
     * @param channel Index of the analog input (2-3)
     * @return Current in milliamps
     */
    public double getAnalogInputCurrent(int channel);

    /**
     * Read analog input voltage
     *
     * @param channel Channel to read the voltage from (0-3)
     * @param jumperSetting Jumper setting of the corresponding analog input
     * jumper
     * @return Voltage
     */
    public double getAnalogInputVoltage(int channel, AnalogInReferenceVoltage jumperSetting);

    /**
     * Get a single digital input
     *
     * @param channel Input index (0-7)
     * @return True if the input is high, false otherwise
     */
    public boolean getDigitalInput(int channel);

    /**
     * Get all digital inputs. Each input state is reflected in the
     * corresponding bit value
     *
     * @return
     */
    public int getDigitalInputs();

    /**
     * Read all GPIO values
     *
     * @return each bit represents a GPIO state
     */
    public byte getGpioValues();

    /**
     * Get the microcontroller status register
     *
     * @return
     */
    public int getUcStatusRegister();

    /**
     * Get the microcontroller's firmware version
     *
     * @return
     */
    public String getUcVersion();

    /**
     * Read temperature sensor humidity
     *
     * @param channel Channel (0-3)
     * @param sensorType Type of sensor
     * @return temperature value
     */
    public double readHumidity(int channel, SensorType sensorType);

    /**
     * Read temperature sensor
     *
     * @param channel Channel (0-3)
     * @param sensorType Type of sensor
     * @return temperature value
     */
    public double readTemperature(int channel, SensorType sensorType);

    /**
     * Reset the microcontroller
     */
    public void resetUc();

    /**
     * Set analog input control registers
     *
     * @param register1
     * @param register2
     */
    public void setAnalogInputConrolRegisters(byte register1, byte register2);

    /**
     * Set analog output
     *
     * @param channel Analog output index (0-1)
     * @param value Value to set
     */
    public void setAnalogOutput(int channel, short value);

    /**
     * Set all digital outputs. Each output state is set to the corresponding
     * bit value
     *
     * @param value
     */
    public void setDigitalOutputs(byte value);

    /**
     * Set GPIO control register
     *
     * @param register
     */
    public void setGpioControlRegister(byte register);

    /**
     * Set all GPIO values
     *
     * @param values each bit represents a GPIO state
     */
    public void setGpioValues(byte values);

    /**
     * Set the PWM control registers
     *
     * @param register1
     * @param register2
     * @param register3
     */
    public void setPwmControlRegisters(byte register1, byte register2, byte register3);

    /**
     * Set PWM value of a channel
     *
     * @param channel Channel to set (0-1)
     * @param value Value to set the channel to (0-65535)
     */
    public void setPwmValue(int channel, short value);

    /**
     * Set the status register value
     *
     * @param value
     */
    public void setRaspberryPiStatusRegister(byte value);

    /**
     * Set all relays states
     *
     * @param values (each bit represents a single relays state)
     */
    public void setRelaisValues(byte values);

    /**
     * Set the serial hardware mode
     *
     * @param mode RS232 or RS485
     */
    public void setSerialHardwareMode(SerialHardwareMode mode);

    /**
     * Set servo value
     *
     * @param channel Servo channel to set the value (0-1)
     * @param value Value to set
     */
    public void setServoValue(int channel, byte value);

}
