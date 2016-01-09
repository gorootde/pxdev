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

import com.sun.jna.Library;
import com.sun.jna.Structure;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * pixtend Library JNA interface
 *
 * @author Michael Kolb <dev(at)db1smk(dot)com>
 */
interface JnaInterface extends Library {

    public class pixtOut extends Structure {

        public byte byDigOut;
        public byte byRelayOut;
        public byte byGpioOut;
        public short wPwm0;
        public short wPwm1;
        public byte byPwm0Ctrl0;
        public byte byPwm0Ctrl1;
        public byte byPwm0Ctrl2;
        public byte byGpioCtrl;
        public byte byUcCtrl;
        public byte byAiCtrl0;
        public byte byAiCtrl1;
        public byte byPiStatus;
        public byte byAux0;

        @Override
        protected List getFieldOrder() {
            List<Field> fields = Arrays.asList(getClass().getDeclaredFields());
            List<String> result = new LinkedList<>();
            fields.stream().map(field -> field.getName()).forEach(result::add);
            return result;
        }

    }

    public class pixtOutDAC extends Structure {

        public short wAOut0;
        public short wAOut1;

        @Override
        protected List getFieldOrder() {
            List<Field> fields = Arrays.asList(getClass().getDeclaredFields());
            List<String> result = new LinkedList<>();
            fields.stream().map(field -> field.getName()).forEach(result::add);
            return result;

        }
    }

    public class pixtIn extends Structure {

        public byte byDigIn;
        public short wAi0;
        public short wAi1;
        public short wAi2;
        public short wAi3;
        public byte byGpioIn;
        public short wTemp0;
        public short wTemp1;
        public short wTemp2;
        public short wTemp3;
        public short wHumid0;
        public short wHumid1;
        public short wHumid2;
        public short wHumid3;
        public byte byUcVersionL;
        public byte byUcVersionH;
        public byte byUcStatus;
        public float rAi0;
        public float rAi1;
        public float rAi2;
        public float rAi3;
        public float rTemp0;
        public float rTemp1;
        public float rTemp2;
        public float rTemp3;
        public float rHumid0;
        public float rHumid1;
        public float rHumid2;
        public float rHumid3;

        @Override
        protected List getFieldOrder() {
            List<Field> fields = Arrays.asList(getClass().getDeclaredFields());
            List<String> result = new LinkedList<>();
            fields.stream().map(field -> field.getName()).forEach(result::add);
            return result;
        }
    }

    public short crc16_calc(short crc, byte data);

    public int Spi_AutoMode(pixtOut OutputData, pixtIn InputData);

    public int Spi_AutoModeDAC(pixtOutDAC OutputDataDAC);

    public int Spi_Set_Dout(int value);

    public int Spi_Get_Din();

    public short Spi_Get_Ain(int Idx);

    public int Spi_Set_Aout(int channel, short value);

    public int Spi_Set_Relays(int value);

    public short Spi_Get_Temp(int Idx);

    public short Spi_Get_Hum(int Idx);

    public int Spi_Set_Servo(int channel, int value);

    public int Spi_Set_Pwm(int channel, short value);

    public int Spi_Set_PwmControl(int value0, int value1, int value2);

    public int Spi_Set_GpioControl(int value);

    public int Spi_Set_UcControl(int value);

    public int Spi_Set_AiControl(int value0, int value1);

    public int Spi_Set_RaspStat(int value);

    public int Spi_Setup(int spi_device);

    public int Spi_uC_Reset();

    public int Spi_Get_uC_Status();

    public short Spi_Get_uC_Version();

    public int Change_Gpio_Mode(byte pin, byte mode);

    public int Change_Serial_Mode(byte mode);

    public int Spi_Set_Gpio(int value);

    public int Spi_Get_Gpio();
}
