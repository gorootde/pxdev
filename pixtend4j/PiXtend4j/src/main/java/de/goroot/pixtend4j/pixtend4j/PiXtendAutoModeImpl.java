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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Timer;
import java.util.TimerTask;

/**
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
class PiXtendAutoModeImpl extends AbstractPiXtendImpl {

	private final JnaInterface.pixtIn input = new JnaInterface.pixtIn();
	private final JnaInterface.pixtOut output = new JnaInterface.pixtOut();
	private final JnaInterface.pixtOutDAC dacOut = new JnaInterface.pixtOutDAC();

	private final Timer tmer = new Timer(true);
	private final TimerTask task = new TimerTask() {
		@Override
		public void run() {
			update();
		}
	};

	public PiXtendAutoModeImpl() {
		super();

		setUcControlRegister(UcControlRegisterBits.RUN_BIT, UcControlRegisterBits.WATCHDOG_ENABLE);

		tmer.scheduleAtFixedRate(task, 200, 200);

	}

	private synchronized void update() {
		getNativeLib().Spi_AutoMode(output, input);
		getNativeLib().Spi_AutoModeDAC(dacOut);
	}

	@Override
	protected synchronized int readDigitalInputs() {
		return input.byDigIn;
	}

	@Override
	protected synchronized short readAnalogIn(int channel) {
		switch (channel) {
		case 0:
			return input.wAi0;
		case 1:
			return input.wAi1;
		case 2:
			return input.wAi2;
		case 3:
			return input.wAi3;
		default:
			throw new IllegalArgumentException("Unknown Channel " + channel);
		}
	}

	@Override
	protected synchronized void writeDigitalOutputs(byte value) {
		output.byDigOut = value;
	}

	@Override
	protected synchronized void writeAnalogInputControlRegisters(byte register1, byte register2) {
		output.byAiCtrl0 = register1;
		output.byAiCtrl1 = register2;
	}

	@Override
	protected synchronized void writeAnalogOut(int channel, short value) {
		switch (channel) {
		case 0:
			dacOut.wAOut0 = value;
			break;
		case 1:
			dacOut.wAOut1 = value;
			break;
		default:
			throw new IllegalArgumentException("Unknown Channel " + channel);
		}
	}

	@Override
	protected synchronized void writeRelaisValues(byte values) {
		output.byRelayOut = values;
	}

	@Override
	protected synchronized int readGpio() {
		return input.byGpioIn;
	}

	@Override
	protected synchronized void writeGpioValues(byte values) {
		output.byGpioOut = values;
	}

	@Override
	protected synchronized void writeGpioControlRegister(byte register) {
		output.byGpioCtrl = register;
	}

	@Override
	protected synchronized void writeServoValue(int channel, byte value) {
		throw new UnsupportedOperationException("not supported");
	}

	@Override
	protected synchronized void writePwmValue(int channel, short value) {
		switch (channel) {
		case 0:
			output.wPwm0 = value;
			break;
		case 1:
			output.wPwm1 = value;
			break;
		default:
			throw new IllegalArgumentException("Unknown Channel " + channel);
		}
	}

	@Override
	protected synchronized short readHum(int channel) {
		switch (channel) {
		case 0:
			return input.wHumid0;
		case 1:
			return input.wHumid1;
		case 2:
			return input.wHumid2;
		case 3:
			return input.wHumid3;
		default:
			throw new IllegalArgumentException("Unknown Channel " + channel);
		}
	}

	@Override
	protected synchronized short readTemp(int channel) {
		switch (channel) {
		case 0:
			return input.wTemp0;
		case 1:
			return input.wTemp1;
		case 2:
			return input.wTemp2;
		case 3:
			return input.wTemp3;
		default:
			throw new IllegalArgumentException("Unknown Channel " + channel);
		}
	}

	@Override
	protected synchronized void writeRpiStatusRegister(byte value) {
		output.byPiStatus = value;
	}

	@Override
	protected synchronized short readUcVersion() {
		ByteBuffer bb = ByteBuffer.allocate(2);
		bb.order(ByteOrder.LITTLE_ENDIAN);
		bb.put(input.byUcVersionL);
		bb.put(input.byUcVersionH);
		return bb.getShort(0);
	}

	@Override
	protected synchronized int readUcStatusRegister() {
		return input.byUcStatus;
	}

	@Override
	protected synchronized void writeSerialMode(int i) {
		throw new UnsupportedOperationException("not supported");
	}

	@Override
	protected synchronized void writePwmControlRegisters(byte register1, byte register2, byte register3) {
		output.byPwm0Ctrl0 = register1;
		output.byPwm0Ctrl1 = register2;
		output.byPwm0Ctrl2 = register3;

	}

	@Override
	protected void writeUcControlRegister(byte register) {
		output.byUcCtrl = register;
	}

}
