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
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

//@I2cDeviceType
//@DeviceProperties(xmlTag = "OctoQuadFTC", name = "OctoQuadFTC")
public class OctoQuadImpl extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> implements OctoQuad, CachingOctoQuad
{
    private static final int I2C_ADDRESS = 0x30;
    private static final int SUPPORTED_FW_VERSION_MAJ = 2;
    private static final int SUPPORTED_FW_VERSION_MIN = 1;
    private static final ByteOrder OCTOQUAD_ENDIAN = ByteOrder.LITTLE_ENDIAN;

    private static final byte CMD_SET_PARAM = 1;
    private static final byte CMD_READ_PARAM = 2;
    private static final byte CMD_WRITE_PARAMS_TO_FLASH = 3;

    private static final byte CMD_RESET_EVERYTHING = 20;
    private static final byte CMD_RESET_ENCODERS = 21;

    private static final byte PARAM_ENCODER_DIRECTIONS = 0;
    private static final byte PARAM_I2C_RECOVERY_MODE = 1;
    private static final byte PARAM_CHANNEL_BANK_CONFIG = 2;
    private static final byte PARAM_CHANNEL_VEL_INTVL = 3;
    private static final byte PARAM_CHANNEL_PULSE_WIDTH_MIN_MAX = 4;
    private static final byte PARAM_CHANNEL_PULSE_WIDTH_TRACKS_WRAP = 5;

    private boolean isInitialized = false;

    public class OctoQuadException extends RuntimeException
    {
        public OctoQuadException(String msg)
        {
            super(msg);
        }
    }

    public OctoQuadImpl(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
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
        int16_t(2);

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

        ENCODER_0_POSITION                (0x0C, RegisterType.int32_t),
        ENCODER_1_POSITION                (0x10, RegisterType.int32_t),
        ENCODER_2_POSITION                (0x14, RegisterType.int32_t),
        ENCODER_3_POSITION                (0x18, RegisterType.int32_t),
        ENCODER_4_POSITION                (0x1C, RegisterType.int32_t),
        ENCODER_5_POSITION                (0x20, RegisterType.int32_t),
        ENCODER_6_POSITION                (0x24, RegisterType.int32_t),
        ENCODER_7_POSITION                (0x28, RegisterType.int32_t),

        ENCODER_0_VELOCITY                (0x2C, RegisterType.int16_t),
        ENCODER_1_VELOCITY                (0x2E, RegisterType.int16_t),
        ENCODER_2_VELOCITY                (0x30, RegisterType.int16_t),
        ENCODER_3_VELOCITY                (0x32, RegisterType.int16_t),
        ENCODER_4_VELOCITY                (0x34, RegisterType.int16_t),
        ENCODER_5_VELOCITY                (0x36, RegisterType.int16_t),
        ENCODER_6_VELOCITY                (0x38, RegisterType.int16_t),
        ENCODER_7_VELOCITY                (0x3A, RegisterType.int16_t);

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

        byte[] bytes = readContiguousRegisters(Register.ENCODER_0_POSITION, Register.ENCODER_7_VELOCITY);

        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        buffer.order(OCTOQUAD_ENDIAN);

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out.positions[i] = buffer.getInt();
        }

        for(int i = 0; i < NUM_ENCODERS; i++)
        {
            out.velocities[i] = buffer.getShort();
        }
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

    static final String unsupportedApiMsg = "This API call is only supported on the OctoQuad FTC Edition MK2 with firmware v3";
    @Override public LocalizerDataBlock readLocalizerData() {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void readLocalizerData(LocalizerDataBlock out) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void readLocalizerDataAndEncoderPositions(LocalizerDataBlock localizerOut, int[] positionsOut) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void readLocalizerDataAndAllEncoderData(LocalizerDataBlock localizerOut, EncoderDataBlock encoderOut) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerPose(int posX_mm, int posY_mm, float heading_rad) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerHeading(float heading_rad) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerPortX(int port) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerPortY(int port)  {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerCountsPerMM_X(float ticksPerMM_x) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerCountsPerMM_Y(float ticksPerMM_y) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerTcpOffsetMM_X(float tcpOffsetMM_X) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerTcpOffsetMM_Y(float tcpOffsetMM_Y) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerImuHeadingScalar(float headingScalar) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setLocalizerVelocityIntervalMS(int ms) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void setAllLocalizerParameters(int portX, int portY, float ticksPerMM_x, float ticksPerMM_y, float tcpOffsetMM_X, float tcpOffsetMM_Y, float headingScalar, int velocityIntervalMs) {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public LocalizerStatus getLocalizerStatus() {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public void resetLocalizer() {throw new UnsupportedOperationException(unsupportedApiMsg);}
    @Override public LocalizerYawAxis getLocalizerHeadingAxisChoice() {throw new UnsupportedOperationException(unsupportedApiMsg);}

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
}

