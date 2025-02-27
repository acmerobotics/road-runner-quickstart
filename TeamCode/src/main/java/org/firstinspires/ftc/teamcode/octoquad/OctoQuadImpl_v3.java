/*
 * Copyright (c) 2022 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.octoquad;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

@I2cDeviceType
@DeviceProperties(xmlTag = "OctoQuadFTC_FW_v3", name = "OctoQuadFTC MK2")
public class OctoQuadImpl_v3 extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> implements OctoQuad, CachingOctoQuad
{
    private static final int I2C_ADDRESS = 0x30;
    private static final int SUPPORTED_FW_VERSION_MAJ = 3;
    private static final int SUPPORTED_FW_VERSION_MIN = 0;
    private static final ByteOrder OCTOQUAD_ENDIAN = ByteOrder.LITTLE_ENDIAN;

    private static final byte CMD_SET_PARAM = 1;
    private static final byte CMD_READ_PARAM = 2;
    private static final byte CMD_WRITE_PARAMS_TO_FLASH = 3;

    private static final byte CMD_RESET_EVERYTHING = 20;
    private static final byte CMD_RESET_ENCODERS = 21;

    private static final byte CMD_RESET_LOCALIZER = 40;

    private static final byte PARAM_ENCODER_DIRECTIONS = 0;
    private static final byte PARAM_I2C_RECOVERY_MODE = 1;
    private static final byte PARAM_CHANNEL_BANK_CONFIG = 2;
    private static final byte PARAM_CHANNEL_VEL_INTVL = 3;
    private static final byte PARAM_CHANNEL_PULSE_WIDTH_MIN_MAX = 4;
    private static final byte PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP = 5;

    private static final byte PARAM_LOCALIZER_X_TICKS_PER_MM = 50;
    private static final byte PARAM_LOCALIZER_Y_TICKS_PER_MM = 51;
    private static final byte PARAM_LOCALIZER_TCP_OFFSET_X_MM = 52;
    private static final byte PARAM_LOCALIZER_TCP_OFFSET_Y_MM = 53;
    private static final byte PARAM_LOCALIZER_IMU_HEADING_SCALAR = 54;
    private static final byte PARAM_LOCALIZER_PORT_X = 55;
    private static final byte PARAM_LOCALIZER_PORT_Y = 56;
    private static final byte PARAM_LOCALIZER_VEL_INTVL = 57;

    private static final float SCALAR_LOCALIZER_HEADING_VELOCITY = 1/600f;
    private static final float SCALAR_LOCALIZER_HEADING = 1/5000f;

    private boolean isInitialized = false;

    public class OctoQuadException extends RuntimeException
    {
        public OctoQuadException(String msg)
        {
            super(msg);
        }
    }

    public OctoQuadImpl_v3(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    protected boolean doInitialize()
    {
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        isInitialized = false;
        verifyInitialization();
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.DigitalChickenLabs;
    }

    @Override
    public String getDeviceName()
    {
        return "OctoQuadFTC";
    }

    enum RegisterType
    {
        uint8_t(1),
        int32_t(4),
        int16_t(2),
        uint16_t(2),
        float32(4);

        public final int length;

        RegisterType(int length)
        {
            this.length = length;
        }
    }

    enum Register
    {
        CHIP_ID                           (0x00, RegisterType.uint8_t),
        FIRMWARE_VERSION_MAJOR            (0x01, RegisterType.uint8_t),
        FIRMWARE_VERSION_MINOR            (0x02, RegisterType.uint8_t),
        FIRMWARE_VERSION_ENGINEERING      (0x03, RegisterType.uint8_t),
        COMMAND                           (0x04, RegisterType.uint8_t),
        COMMAND_DAT_0                     (0x05, RegisterType.uint8_t),
        COMMAND_DAT_1                     (0x06, RegisterType.uint8_t),
        COMMAND_DAT_2                     (0x07, RegisterType.uint8_t),
        COMMAND_DAT_3                     (0x08, RegisterType.uint8_t),
        COMMAND_DAT_4                     (0x09, RegisterType.uint8_t),
        COMMAND_DAT_5                     (0x0A, RegisterType.uint8_t),
        COMMAND_DAT_6                     (0x0B, RegisterType.uint8_t),

        LOCALIZER_YAW_AXIS                (0x0C, RegisterType.uint8_t),
        LOCALIZER_STATUS                  (0x0D, RegisterType.uint8_t),
        LOCALIZER_VX                      (0x0E, RegisterType.int16_t),
        LOCALIZER_VY                      (0x10, RegisterType.int16_t),
        LOCALIZER_VH                      (0x12, RegisterType.int16_t),
        LOCALIZER_X                       (0x14, RegisterType.int16_t),
        LOCALIZER_Y                       (0x16, RegisterType.int16_t),
        LOCALIZER_H                       (0x18, RegisterType.int16_t),
        LOCALIZER_CRC16                   (0x1A, RegisterType.uint16_t), // LOCALIZER_STATUS --> LOCALIZER_H

        ENCODER_0_POSITION                (0x1C, RegisterType.int32_t),
        ENCODER_1_POSITION                (0x20, RegisterType.int32_t),
        ENCODER_2_POSITION                (0x24, RegisterType.int32_t),
        ENCODER_3_POSITION                (0x28, RegisterType.int32_t),
        ENCODER_4_POSITION                (0x2C, RegisterType.int32_t),
        ENCODER_5_POSITION                (0x30, RegisterType.int32_t),
        ENCODER_6_POSITION                (0x34, RegisterType.int32_t),
        ENCODER_7_POSITION                (0x38, RegisterType.int32_t),

        ENCODER_0_VELOCITY                (0x3C, RegisterType.int16_t),
        ENCODER_1_VELOCITY                (0x3E, RegisterType.int16_t),
        ENCODER_2_VELOCITY                (0x40, RegisterType.int16_t),
        ENCODER_3_VELOCITY                (0x42, RegisterType.int16_t),
        ENCODER_4_VELOCITY                (0x44, RegisterType.int16_t),
        ENCODER_5_VELOCITY                (0x46, RegisterType.int16_t),
        ENCODER_6_VELOCITY                (0x48, RegisterType.int16_t),
        ENCODER_7_VELOCITY                (0x4A, RegisterType.int16_t),

        ENCODER_DATA_CRC16                (0x4C, RegisterType.uint16_t); // ENC0 --> VEL7

        public final byte addr;
        public final int length;

        Register(int addr, RegisterType type)
        {
            this.addr = (byte) addr;
            this.length = type.length;
        }

        public static final Register[] all = Register.values();
    }

    // --------------------------------------------------------------------------------------------------------------------------------
    // PUBLIC OCTOQUAD API
    //---------------------------------------------------------------------------------------------------------------------------------

    public byte getChipId()
    {
        return readRegister(Register.CHIP_ID)[0];
    }

    public FirmwareVersion getFirmwareVersion()
    {
        byte[] fw = readContiguousRegisters(Register.FIRMWARE_VERSION_MAJOR, Register.FIRMWARE_VERSION_ENGINEERING);

        int maj = fw[0] & 0xFF;
        int min = fw[1] & 0xFF;
        int eng = fw[2] & 0xFF;

        return new FirmwareVersion(maj, min, eng);
    }

    public String getFirmwareVersionString()
    {
        return getFirmwareVersion().toString();
    }

    public int readSinglePosition(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        Register register = Register.all[Register.ENCODER_0_POSITION.ordinal()+idx];
        return intFromBytes(readRegister(register));
    }

    public void readAllPositions(int[] out)
    {
        verifyInitialization();

        if(out.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("out.length != 8");
        }

        byte[] bytes = readContiguousRegisters(Register.ENCODER_0_POSITION, Register.ENCODER_7_POSITION);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out[i] = buffer.getInt();
        }
    }

    public int[] readAllPositions()
    {
        verifyInitialization();

        int[] block = new int[NUM_ENCODERS];
        readAllPositions(block);
        return block;
    }

    public int[] readPositionRange(int idxFirst, int idxLast)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idxFirst, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(idxLast, ENCODER_FIRST, ENCODER_LAST);

        Register registerFirst = Register.all[Register.ENCODER_0_POSITION.ordinal()+idxFirst];
        Register registerLast = Register.all[Register.ENCODER_0_POSITION.ordinal()+idxLast];

        byte[] data = readContiguousRegisters(registerFirst, registerLast);
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        int numEncodersRead = idxLast-idxFirst+1;
        int[] encoderCounts = new int[numEncodersRead];

        for(int i = 0; i < numEncodersRead; i++)
        {
            encoderCounts[i] = buffer.getInt();
        }

        return encoderCounts;
    }

    // ALSO USED BY CACHING API !
    public void resetSinglePosition(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        byte dat = (byte) (1 << idx);
        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_RESET_ENCODERS, dat});

        if (cachingMode == CachingMode.AUTO)
        {
            refreshCache();
        }
    }

    // ALSO USED BY CACHING API !
    public void resetAllPositions()
    {
        verifyInitialization();
        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[] {CMD_RESET_ENCODERS, (byte)0xFF});

        if (cachingMode == CachingMode.AUTO)
        {
            refreshCache();
        }
    }

    public void resetMultiplePositions(boolean[] resets)
    {
        verifyInitialization();

        if(resets.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("resets.length != 8");
        }

        byte dat = 0;

        for(int i = ENCODER_FIRST; i <= ENCODER_LAST; i++)
        {
            dat |= resets[i] ? (byte)(1 << i) : 0;
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[] {CMD_RESET_ENCODERS, dat});
    }

    public void resetMultiplePositions(int... indices)
    {
        verifyInitialization();

        for(int idx : indices)
        {
            Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        }

        byte dat = 0;

        for(int idx : indices)
        {
            dat |= 1 << idx;
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[] {CMD_RESET_ENCODERS, dat});
    }

    public void setSingleEncoderDirection(int idx, EncoderDirection direction)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_ENCODER_DIRECTIONS});
        byte directionRegisterData = readRegister(Register.COMMAND_DAT_0)[0];

        if(direction == EncoderDirection.REVERSE)
        {
            directionRegisterData |= (byte) (1 << idx);
        }
        else
        {
            directionRegisterData &= (byte) ~(1 << idx);
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_ENCODER_DIRECTIONS, directionRegisterData});
    }

    public EncoderDirection getSingleEncoderDirection(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_ENCODER_DIRECTIONS});
        byte directions = readRegister(Register.COMMAND_DAT_0)[0];

        boolean reversed = (directions & (1 << idx)) != 0;
        return  reversed ? EncoderDirection.REVERSE : EncoderDirection.FORWARD;
    }

    public void setAllEncoderDirections(boolean[] reverse)
    {
        verifyInitialization();

        if(reverse.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("reverse.length != 8");
        }

        byte directionRegisterData = 0;

        for(int i = ENCODER_FIRST; i <= ENCODER_LAST; i++)
        {
            if(reverse[i])
            {
                directionRegisterData |= (byte) (1 << i);
            }
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_ENCODER_DIRECTIONS, directionRegisterData});
    }

    public short readSingleVelocity(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        Register register = Register.all[Register.ENCODER_0_VELOCITY.ordinal()+idx];
        return shortFromBytes(readRegister(register));
    }

    public void readAllVelocities(short[] out)
    {
        verifyInitialization();

        if(out.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("out.length != 8");
        }

        byte[] bytes = readContiguousRegisters(Register.ENCODER_0_VELOCITY, Register.ENCODER_7_VELOCITY);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out[i] = buffer.getShort();
        }
    }

    public short[] readAllVelocities()
    {
        verifyInitialization();

        short[] block = new short[NUM_ENCODERS];
        readAllVelocities(block);
        return block;
    }

    public short[] readVelocityRange(int idxFirst, int idxLast)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idxFirst, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(idxLast, ENCODER_FIRST, ENCODER_LAST);

        Register registerFirst = Register.all[Register.ENCODER_0_VELOCITY.ordinal()+idxFirst];
        Register registerLast = Register.all[Register.ENCODER_0_VELOCITY.ordinal()+idxLast];

        byte[] data = readContiguousRegisters(registerFirst, registerLast);
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        int numVelocitiesRead = idxLast-idxFirst+1;
        short[] velocities = new short[numVelocitiesRead];

        for(int i = 0; i < numVelocitiesRead; i++)
        {
            velocities[i] = buffer.getShort();
        }

        return velocities;
    }

    private void unpackAllEncoderData(ByteBuffer buffer, EncoderDataBlock out)
    {
        buffer.mark(); // mark our current position
        byte[] asArray = new byte[(RegisterType.int32_t.length + RegisterType.int16_t.length) * NUM_ENCODERS]; // only the encoder data itself
        buffer.get(asArray); // read data as array (needed for CRC)
        buffer.reset(); // rewind back to mark

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out.positions[i] = buffer.getInt();
        }

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out.velocities[i] = buffer.getShort();
        }

        short crc = buffer.getShort();
        short calculatedCrc = calc_crc16_profibus(asArray);

        out.crcOk = calculatedCrc == crc;

        if (!out.crcOk)
        {
            RobotLog.ee("OctoQuadImpl_v3", String.format("Encoder data CRC error!! Expect = 0x%x Actual = 0x%x", calculatedCrc, crc));
        }
    }

    public void readAllEncoderData(EncoderDataBlock out)
    {
        verifyInitialization();

        if(out.positions.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("out.counts.length != 8");
        }

        if(out.velocities.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("out.velocities.length != 8");
        }

        byte[] bytes = readContiguousRegisters(Register.ENCODER_0_POSITION, Register.ENCODER_DATA_CRC16);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        unpackAllEncoderData(buffer, out);
    }

    public EncoderDataBlock readAllEncoderData()
    {
        verifyInitialization();

        EncoderDataBlock block = new EncoderDataBlock();
        readAllEncoderData(block);

        return block;
    }

    public void setSingleVelocitySampleInterval(int idx, int intvlms)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(intvlms, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_2, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_VEL_INTVL, (byte)idx, (byte)intvlms});
    }

    public void setAllVelocitySampleIntervals(int intvlms)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(intvlms, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);

        for(int i = ENCODER_FIRST; i <= ENCODER_LAST; i++)
        {
            writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_2, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_VEL_INTVL, (byte)i, (byte)intvlms});
        }
    }

    public void setAllVelocitySampleIntervals(int[] intvlms)
    {
        verifyInitialization();

        if(intvlms.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("intvls.length != 8");
        }

        for(int i : intvlms)
        {
            Range.throwIfRangeIsInvalid(i, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
        }

        for(int i = ENCODER_FIRST; i <= ENCODER_LAST; i++)
        {
            writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_2, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_VEL_INTVL, (byte)i, (byte)intvlms[i]});
        }
    }

    public int getSingleVelocitySampleInterval(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_VEL_INTVL, (byte)idx});
        byte ms = readRegister(Register.COMMAND_DAT_0)[0];
        return ms & 0xFF;
    }

    public int[] getAllVelocitySampleIntervals()
    {
        verifyInitialization();

        int[] ret = new int[NUM_ENCODERS];

        for(int i = ENCODER_FIRST; i <= ENCODER_FIRST; i++)
        {
            writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_VEL_INTVL, (byte)i});
            byte ms = readRegister(Register.COMMAND_DAT_0)[0];
            ret[i] = ms & 0xFF;
        }

        return ret;
    }

    public void setSingleChannelPulseWidthParams(int idx, int min, int max)
    {
        setSingleChannelPulseWidthParams(idx, new ChannelPulseWidthParams(min, max));
    }

    public void setSingleChannelPulseWidthParams(int idx, ChannelPulseWidthParams params)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(params.min_length_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
        Range.throwIfRangeIsInvalid(params.max_length_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        if(params.max_length_us <= params.min_length_us)
        {
            throw new RuntimeException("params.max_length_us <= params.min_length_us");
        }

        ByteBuffer outgoing = ByteBuffer.allocate(7);
        outgoing.order(OCTOQUAD_ENDIAN);
        outgoing.put(CMD_SET_PARAM);
        outgoing.put(PARAM_CHANNEL_PULSE_WIDTH_MIN_MAX);
        outgoing.put((byte)idx);
        outgoing.putShort((short)params.min_length_us);
        outgoing.putShort((short)params.max_length_us);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_5, outgoing.array());
    }

    public ChannelPulseWidthParams getSingleChannelPulseWidthParams(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_PULSE_WIDTH_MIN_MAX, (byte)idx});
        byte[] result = readContiguousRegisters(Register.COMMAND_DAT_0, Register.COMMAND_DAT_3);

        ByteBuffer buffer = ByteBuffer.wrap(result);
        buffer.order(OCTOQUAD_ENDIAN);

        ChannelPulseWidthParams params = new ChannelPulseWidthParams();
        params.min_length_us = buffer.getShort() & 0xFFFF;
        params.max_length_us = buffer.getShort() & 0xFFFF;

        return params;
    }

    public void setSingleChannelPulseWidthTracksWrap(int idx, boolean trackWrap)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP});
        byte absWrapTrackRegisterData = readRegister(Register.COMMAND_DAT_0)[0];

        if(trackWrap)
        {
            absWrapTrackRegisterData |= (byte) (1 << idx);
        }
        else
        {
            absWrapTrackRegisterData &= (byte) ~(1 << idx);
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP, absWrapTrackRegisterData});
    }

    public boolean getSingleChannelPulseWidthTracksWrap(int idx)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP});
        byte tracking = readRegister(Register.COMMAND_DAT_0)[0];

        return (tracking & (1 << idx)) != 0;
    }

    public void setAllChannelsPulseWidthTracksWrap(boolean[] trackWrap)
    {
        verifyInitialization();

        if(trackWrap.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("trackWrap.length != 8");
        }

        byte absWrapTrackRegisterData = 0;

        for(int i = ENCODER_FIRST; i <= ENCODER_LAST; i++)
        {
            if(trackWrap[i])
            {
                absWrapTrackRegisterData |= (byte) (1 << i);
            }
        }

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP, absWrapTrackRegisterData});
    }

    public void setLocalizerCountsPerMM_X(float ticksPerMM_x)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(ticksPerMM_x, 0, Float.MAX_VALUE);

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);
        buf.put(CMD_SET_PARAM);
        buf.put(PARAM_LOCALIZER_X_TICKS_PER_MM);
        buf.putFloat(ticksPerMM_x);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_4, buf.array());
    }

    public void setLocalizerCountsPerMM_Y(float ticksPerMM_y)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(ticksPerMM_y, 0, Float.MAX_VALUE);

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);
        buf.put(CMD_SET_PARAM);
        buf.put(PARAM_LOCALIZER_Y_TICKS_PER_MM);
        buf.putFloat(ticksPerMM_y);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_4, buf.array());
    }

    public void setLocalizerTcpOffsetMM_X(float tcpOffsetMM_X)
    {
        verifyInitialization();

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);
        buf.put(CMD_SET_PARAM);
        buf.put(PARAM_LOCALIZER_TCP_OFFSET_X_MM);
        buf.putFloat(tcpOffsetMM_X);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_4, buf.array());
    }

    public void setLocalizerTcpOffsetMM_Y(float tcpOffsetMM_Y)
    {
        verifyInitialization();

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);
        buf.put(CMD_SET_PARAM);
        buf.put(PARAM_LOCALIZER_TCP_OFFSET_Y_MM);
        buf.putFloat(tcpOffsetMM_Y);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_4, buf.array());
    }

    public void setLocalizerImuHeadingScalar(float headingScalar)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(headingScalar, 0, Float.MAX_VALUE);

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);
        buf.put(CMD_SET_PARAM);
        buf.put(PARAM_LOCALIZER_IMU_HEADING_SCALAR);
        buf.putFloat(headingScalar);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_4, buf.array());
    }

    public void setLocalizerPortX(int port)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(port, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_LOCALIZER_PORT_X, (byte)port});
    }

    public void setLocalizerPortY(int port)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(port, ENCODER_FIRST, ENCODER_LAST);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_LOCALIZER_PORT_Y, (byte)port});
    }

    public void setLocalizerVelocityIntervalMS(int ms)
    {
        verifyInitialization();

        Range.throwIfRangeIsInvalid(ms, 1, 255);

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_LOCALIZER_VEL_INTVL, (byte)ms});
    }

    public LocalizerStatus getLocalizerStatus()
    {
        verifyInitialization();
        int state =  readRegister(Register.LOCALIZER_STATUS)[0] & 0xFF;

        // Prevent crash if value out of expected range
        if (state < LocalizerStatus.values().length)
        {
            return LocalizerStatus.values()[state];
        }
        else
        {
            return LocalizerStatus.INVALID;
        }
    }

    public LocalizerYawAxis getLocalizerHeadingAxisChoice()
    {
        verifyInitialization();
        int code = readRegister(Register.LOCALIZER_YAW_AXIS)[0] & 0xFF;

        // Prevent crash if value out of expected range
        if (code < LocalizerYawAxis.values().length)
        {
            return LocalizerYawAxis.values()[code];
        }
        else
        {
            return LocalizerYawAxis.UNDECIDED;
        }
    }

    public void resetLocalizer()
    {
        verifyInitialization();
        writeRegister(Register.COMMAND, new byte[] {CMD_RESET_LOCALIZER});
    }

    private void unpackLocalizerData(ByteBuffer buf, LocalizerDataBlock out)
    {
        buf.mark(); // mark our current position
        byte[] asArray = new byte[RegisterType.uint16_t.length*6 + RegisterType.uint8_t.length];
        buf.get(asArray); // read data as array (needed for CRC)
        buf.reset(); // rewind back to mark

        int localizerStatusCode = buf.get() & 0xFF;

        // Prevent crash if value out of expected range
        if (localizerStatusCode < LocalizerStatus.values().length)
        {
            out.localizerStatus = LocalizerStatus.values()[localizerStatusCode];
        }
        else
        {
            out.localizerStatus = LocalizerStatus.INVALID;
        }

        out.velX_mmS = buf.getShort();
        out.velY_mmS = buf.getShort();
        out.velHeading_radS = buf.getShort() * SCALAR_LOCALIZER_HEADING_VELOCITY;
        out.posX_mm = buf.getShort();
        out.posY_mm = buf.getShort();
        out.heading_rad = buf.getShort() * SCALAR_LOCALIZER_HEADING;

        short crc = buf.getShort();
        short calculatedCrc = calc_crc16_profibus(asArray);

        out.crcOk = calculatedCrc == crc;

        if (!out.crcOk)
        {
            RobotLog.ee("OctoQuadImpl_v3", String.format("Localizer data CRC error!! Expect = 0x%x Actual = 0x%x", calculatedCrc, crc));

            StringBuilder bld = new StringBuilder();
            for (byte b : asArray)
            {
                bld.append(String.format(" 0x%x", b));
            }
            bld.append("\r\n");
            System.err.print(bld);
        }
    }

    public void readLocalizerData(LocalizerDataBlock out)
    {
        verifyInitialization();

        byte[] bytes = readContiguousRegisters(Register.LOCALIZER_STATUS, Register.LOCALIZER_CRC16);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        unpackLocalizerData(buffer, out);
    }

    public LocalizerDataBlock readLocalizerData()
    {
        LocalizerDataBlock block = new LocalizerDataBlock();
        readLocalizerData(block);
        return block;
    }

    public void readLocalizerDataAndEncoderPositions(LocalizerDataBlock localizerOut, int[] positionsOut)
    {
        verifyInitialization();

        if(positionsOut.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("positionsOut.length != 8");
        }

        byte[] bytes = readContiguousRegisters(Register.LOCALIZER_STATUS, Register.ENCODER_7_POSITION);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        unpackLocalizerData(buffer, localizerOut);

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            positionsOut[i] = buffer.getInt();
        }
    }

    public void readLocalizerDataAndAllEncoderData(LocalizerDataBlock localizerOut, EncoderDataBlock encoderOut)
    {
        verifyInitialization();

        byte[] bytes = readContiguousRegisters(Register.LOCALIZER_STATUS, Register.ENCODER_DATA_CRC16);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        unpackLocalizerData(buffer, localizerOut);
        unpackAllEncoderData(buffer, encoderOut);
    }

    public void setAllLocalizerParameters(
            int portX,
            int portY,
            float ticksPerMM_x,
            float ticksPerMM_y,
            float tcpOffsetMM_X,
            float tcpOffsetMM_Y,
            float headingScalar,
            int velocityIntervalMs)
    {
        setLocalizerPortX(portX);
        setLocalizerPortY(portY);
        setLocalizerCountsPerMM_X(ticksPerMM_x);
        setLocalizerCountsPerMM_Y(ticksPerMM_y);
        setLocalizerImuHeadingScalar(headingScalar);
        setLocalizerVelocityIntervalMS(velocityIntervalMs);
    }

    public void setLocalizerPose(int posX_mm, int posY_mm, float heading_rad)
    {
        verifyInitialization();

        ByteBuffer buf = ByteBuffer.allocate(6);
        buf.order(OCTOQUAD_ENDIAN);

        buf.putShort((short)posX_mm);
        buf.putShort((short)posY_mm);
        buf.putShort((short) (heading_rad / SCALAR_LOCALIZER_HEADING));

        writeContiguousRegisters(Register.LOCALIZER_X, Register.LOCALIZER_H, buf.array());
    }

    public void setLocalizerHeading(float headingRad)
    {
        verifyInitialization();

        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(OCTOQUAD_ENDIAN);
        buf.putShort((short) (headingRad / SCALAR_LOCALIZER_HEADING));
        writeRegister(Register.LOCALIZER_H, buf.array());
    }

    public void resetEverything()
    {
        verifyInitialization();

        writeRegister(Register.COMMAND, new byte[]{CMD_RESET_EVERYTHING});
    }

    public void setChannelBankConfig(ChannelBankConfig config)
    {
        verifyInitialization();

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_CHANNEL_BANK_CONFIG, config.bVal});
    }

    public ChannelBankConfig getChannelBankConfig()
    {
        verifyInitialization();

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_CHANNEL_BANK_CONFIG});
        byte result = readRegister(Register.COMMAND_DAT_0)[0];

        for(ChannelBankConfig c : ChannelBankConfig.values())
        {
            if(c.bVal == result)
            {
                return c;
            }
        }

        return ChannelBankConfig.ALL_QUADRATURE;
    }

    public void setI2cRecoveryMode(I2cRecoveryMode mode)
    {
        verifyInitialization();

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_1, new byte[]{CMD_SET_PARAM, PARAM_I2C_RECOVERY_MODE, mode.bVal});
    }

    public I2cRecoveryMode getI2cRecoveryMode()
    {
        verifyInitialization();

        writeContiguousRegisters(Register.COMMAND, Register.COMMAND_DAT_0, new byte[]{CMD_READ_PARAM, PARAM_I2C_RECOVERY_MODE});
        byte result = readRegister(Register.COMMAND_DAT_0)[0];

        for(I2cRecoveryMode m : I2cRecoveryMode.values())
        {
            if(m.bVal == result)
            {
                return m;
            }
        }

        return I2cRecoveryMode.NONE;
    }

    public void saveParametersToFlash()
    {
        verifyInitialization();

        writeRegister(Register.COMMAND, new byte[] {CMD_WRITE_PARAMS_TO_FLASH});
        try
        {
            Thread.sleep(100);
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
            e.printStackTrace();
        }
    }

    // --------------------------------------------------------------------------------------------------------------------------------
    // PUBLIC CACHING OCTOQUAD API
    //---------------------------------------------------------------------------------------------------------------------------------

    protected CachingMode cachingMode = CachingMode.AUTO;
    protected EncoderDataBlock cachedData = new EncoderDataBlock();
    protected boolean[] posHasBeenRead = new boolean[NUM_ENCODERS];
    protected boolean[] velHasBeenRead = new boolean[NUM_ENCODERS];

    public void setCachingMode(CachingMode mode)
    {
        this.cachingMode = mode;
        if (cachingMode != CachingMode.NONE)
        {
            refreshCache();
        }
    }

    public void refreshCache()
    {
        readAllEncoderData(cachedData);
        Arrays.fill(posHasBeenRead, false);
        Arrays.fill(velHasBeenRead, false);
    }

    public int readSinglePosition_Caching(int idx)
    {
        // If we're caching we're gonna want to read from the cache
        if (cachingMode == CachingMode.AUTO || cachingMode == CachingMode.MANUAL)
        {
            // Update cache if this is the 2nd read
            if (cachingMode == CachingMode.AUTO && posHasBeenRead[idx])
            {
                refreshCache();
            }

            posHasBeenRead[idx] = true;
            return cachedData.positions[idx];
        }
        // Not caching; read direct
        else
        {
            return readSinglePosition(idx);
        }
    }

    public short readSingleVelocity_Caching(int idx)
    {
        // If we're caching we're gonna want to read from the cache
        if (cachingMode == CachingMode.AUTO || cachingMode == CachingMode.MANUAL)
        {
            // Update cache if this is the 2nd read
            if (cachingMode == CachingMode.AUTO && velHasBeenRead[idx])
            {
                refreshCache();
            }

            velHasBeenRead[idx] = true;
            return cachedData.velocities[idx];
        }
        // Not caching; read direct
        else
        {
            return readSingleVelocity(idx);
        }
    }

    // --------------------------------------------------------------------------------------------------------------------------------
    // INTERNAL
    //---------------------------------------------------------------------------------------------------------------------------------

    private void verifyInitialization()
    {
        if(!isInitialized)
        {
            byte chipId = getChipId();
            if(chipId != OCTOQUAD_CHIP_ID)
            {
                RobotLog.addGlobalWarningMessage("OctoQuad does not report correct CHIP_ID value; (got 0x%X; expected 0x%X) this likely indicates I2C comms are not working", chipId, OCTOQUAD_CHIP_ID);
            }

            FirmwareVersion fw = getFirmwareVersion();

            if(fw.maj != SUPPORTED_FW_VERSION_MAJ)
            {
                RobotLog.addGlobalWarningMessage("OctoQuad is running a different major firmware version than this driver was built for (current=%d; expected=%d) IT IS HIGHLY LIKELY THAT NOTHING WILL WORK! You should flash the firmware to a compatible version (Refer to Section 6 in the OctoQuad datasheet).", fw.maj, SUPPORTED_FW_VERSION_MAJ);
            }
            else
            {
                if(fw.min < SUPPORTED_FW_VERSION_MIN)
                {
                    RobotLog.addGlobalWarningMessage("OctoQuad is running an older minor firmware revision than this driver was built for; certain features may not work (current=%d; expected=%d). You should update the firmware on your OctoQuad (Refer to Section 6 in the OctoQuad datasheet).", fw.min, SUPPORTED_FW_VERSION_MIN);
                }
                else if(fw.min > SUPPORTED_FW_VERSION_MIN)
                {
                    RobotLog.addGlobalWarningMessage("OctoQuad is running a newer minor firmware revision than this driver was built for; (current=%d; expected=%d). You will not be able to access new features in the updated firmware without an updated I2C driver.", fw.min, SUPPORTED_FW_VERSION_MIN);
                }
            }

            isInitialized = true;
        }
    }

    private static int intFromBytes(byte[] bytes)
    {
        ByteBuffer byteBuffer = ByteBuffer.wrap(bytes);
        byteBuffer.order(OCTOQUAD_ENDIAN);
        return byteBuffer.getInt();
    }

    private static short shortFromBytes(byte[] bytes)
    {
        ByteBuffer byteBuffer = ByteBuffer.wrap(bytes);
        byteBuffer.order(OCTOQUAD_ENDIAN);
        return byteBuffer.getShort();
    }

    private byte[] readRegister(Register reg)
    {
        return deviceClient.read(reg.addr, reg.length);
    }

    private byte[] readContiguousRegisters(Register first, Register last)
    {
        int addrStart = first.addr;
        int addrEnd = last.addr + last.length;
        int bytesToRead = addrEnd-addrStart;

        return deviceClient.read(addrStart, bytesToRead);
    }

    private void writeRegister(Register reg, byte[] bytes)
    {
        if(reg.length != bytes.length)
        {
            throw new IllegalArgumentException("reg.length != bytes.length");
        }

        deviceClient.write(reg.addr, bytes);
    }

    private void writeContiguousRegisters(Register first, Register last, byte[] dat)
    {
        int addrStart = first.addr;
        int addrEnd = last.addr + last.length;
        int bytesToWrite = addrEnd-addrStart;

        if(bytesToWrite != dat.length)
        {
            throw new IllegalArgumentException("bytesToWrite != dat.length");
        }

        deviceClient.write(addrStart, dat);
    }

    // --------------------------------------------------------------------------------------------------------------------------------
    // PROFIBUS CRC
    //---------------------------------------------------------------------------------------------------------------------------------

    private static final short crc16_profibus_init = (short) 0xFFFF;
    private static final short crc16_profibus_xor_out = (short) 0xFFFF;
    private static final short[] crc16_profibus_table = {
            (short) 0x0000, (short) 0x1DCF, (short) 0x3B9E, (short) 0x2651,   (short) 0x773C, (short) 0x6AF3, (short) 0x4CA2, (short) 0x516D,
            (short) 0xEE78, (short) 0xF3B7, (short) 0xD5E6, (short) 0xC829,   (short) 0x9944, (short) 0x848B, (short) 0xA2DA, (short) 0xBF15,
            (short) 0xC13F, (short) 0xDCF0, (short) 0xFAA1, (short) 0xE76E,   (short) 0xB603, (short) 0xABCC, (short) 0x8D9D, (short) 0x9052,
            (short) 0x2F47, (short) 0x3288, (short) 0x14D9, (short) 0x0916,   (short) 0x587B, (short) 0x45B4, (short) 0x63E5, (short) 0x7E2A,
            (short) 0x9FB1, (short) 0x827E, (short) 0xA42F, (short) 0xB9E0,   (short) 0xE88D, (short) 0xF542, (short) 0xD313, (short) 0xCEDC,
            (short) 0x71C9, (short) 0x6C06, (short) 0x4A57, (short) 0x5798,   (short) 0x06F5, (short) 0x1B3A, (short) 0x3D6B, (short) 0x20A4,
            (short) 0x5E8E, (short) 0x4341, (short) 0x6510, (short) 0x78DF,   (short) 0x29B2, (short) 0x347D, (short) 0x122C, (short) 0x0FE3,
            (short) 0xB0F6, (short) 0xAD39, (short) 0x8B68, (short) 0x96A7,   (short) 0xC7CA, (short) 0xDA05, (short) 0xFC54, (short) 0xE19B,
            (short) 0x22AD, (short) 0x3F62, (short) 0x1933, (short) 0x04FC,   (short) 0x5591, (short) 0x485E, (short) 0x6E0F, (short) 0x73C0,
            (short) 0xCCD5, (short) 0xD11A, (short) 0xF74B, (short) 0xEA84,   (short) 0xBBE9, (short) 0xA626, (short) 0x8077, (short) 0x9DB8,
            (short) 0xE392, (short) 0xFE5D, (short) 0xD80C, (short) 0xC5C3,   (short) 0x94AE, (short) 0x8961, (short) 0xAF30, (short) 0xB2FF,
            (short) 0x0DEA, (short) 0x1025, (short) 0x3674, (short) 0x2BBB,   (short) 0x7AD6, (short) 0x6719, (short) 0x4148, (short) 0x5C87,
            (short) 0xBD1C, (short) 0xA0D3, (short) 0x8682, (short) 0x9B4D,   (short) 0xCA20, (short) 0xD7EF, (short) 0xF1BE, (short) 0xEC71,
            (short) 0x5364, (short) 0x4EAB, (short) 0x68FA, (short) 0x7535,   (short) 0x2458, (short) 0x3997, (short) 0x1FC6, (short) 0x0209,
            (short) 0x7C23, (short) 0x61EC, (short) 0x47BD, (short) 0x5A72,   (short) 0x0B1F, (short) 0x16D0, (short) 0x3081, (short) 0x2D4E,
            (short) 0x925B, (short) 0x8F94, (short) 0xA9C5, (short) 0xB40A,   (short) 0xE567, (short) 0xF8A8, (short) 0xDEF9, (short) 0xC336,
            (short) 0x455A, (short) 0x5895, (short) 0x7EC4, (short) 0x630B,   (short) 0x3266, (short) 0x2FA9, (short) 0x09F8, (short) 0x1437,
            (short) 0xAB22, (short) 0xB6ED, (short) 0x90BC, (short) 0x8D73,   (short) 0xDC1E, (short) 0xC1D1, (short) 0xE780, (short) 0xFA4F,
            (short) 0x8465, (short) 0x99AA, (short) 0xBFFB, (short) 0xA234,   (short) 0xF359, (short) 0xEE96, (short) 0xC8C7, (short) 0xD508,
            (short) 0x6A1D, (short) 0x77D2, (short) 0x5183, (short) 0x4C4C,   (short) 0x1D21, (short) 0x00EE, (short) 0x26BF, (short) 0x3B70,
            (short) 0xDAEB, (short) 0xC724, (short) 0xE175, (short) 0xFCBA,   (short) 0xADD7, (short) 0xB018, (short) 0x9649, (short) 0x8B86,
            (short) 0x3493, (short) 0x295C, (short) 0x0F0D, (short) 0x12C2,   (short) 0x43AF, (short) 0x5E60, (short) 0x7831, (short) 0x65FE,
            (short) 0x1BD4, (short) 0x061B, (short) 0x204A, (short) 0x3D85,   (short) 0x6CE8, (short) 0x7127, (short) 0x5776, (short) 0x4AB9,
            (short) 0xF5AC, (short) 0xE863, (short) 0xCE32, (short) 0xD3FD,   (short) 0x8290, (short) 0x9F5F, (short) 0xB90E, (short) 0xA4C1,
            (short) 0x67F7, (short) 0x7A38, (short) 0x5C69, (short) 0x41A6,   (short) 0x10CB, (short) 0x0D04, (short) 0x2B55, (short) 0x369A,
            (short) 0x898F, (short) 0x9440, (short) 0xB211, (short) 0xAFDE,   (short) 0xFEB3, (short) 0xE37C, (short) 0xC52D, (short) 0xD8E2,
            (short) 0xA6C8, (short) 0xBB07, (short) 0x9D56, (short) 0x8099,   (short) 0xD1F4, (short) 0xCC3B, (short) 0xEA6A, (short) 0xF7A5,
            (short) 0x48B0, (short) 0x557F, (short) 0x732E, (short) 0x6EE1,   (short) 0x3F8C, (short) 0x2243, (short) 0x0412, (short) 0x19DD,
            (short) 0xF846, (short) 0xE589, (short) 0xC3D8, (short) 0xDE17,   (short) 0x8F7A, (short) 0x92B5, (short) 0xB4E4, (short) 0xA92B,
            (short) 0x163E, (short) 0x0BF1, (short) 0x2DA0, (short) 0x306F,   (short) 0x6102, (short) 0x7CCD, (short) 0x5A9C, (short) 0x4753,
            (short) 0x3979, (short) 0x24B6, (short) 0x02E7, (short) 0x1F28,   (short) 0x4E45, (short) 0x538A, (short) 0x75DB, (short) 0x6814,
            (short) 0xD701, (short) 0xCACE, (short) 0xEC9F, (short) 0xF150,   (short) 0xA03D, (short) 0xBDF2, (short) 0x9BA3, (short) 0x866C
    };

    private static short calc_crc16_profibus(byte[] data, int len)
    {
        short crc = crc16_profibus_init;

        for (int i = 0; i < len; i++)
        {
            // Need to & 0xFF because java 16-bit is signed
            crc = (short) ((crc << 8) ^ crc16_profibus_table[((crc >> 8) ^ data[i & 0xFF]) & 0xFF]);
        }

        return (short) (crc ^ crc16_profibus_xor_out);
    }

    private static short calc_crc16_profibus(byte[] data)
    {
        return calc_crc16_profibus(data, data.length);
    }
}

