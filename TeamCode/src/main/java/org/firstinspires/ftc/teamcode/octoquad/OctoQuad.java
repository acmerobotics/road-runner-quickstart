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



public interface OctoQuad extends OctoQuadBase
{
    /**
     * Class to represent an OctoQuad firmware version
     */
    class FirmwareVersion
    {
        public final int maj;
        public final int min;
        public final int eng;

        public FirmwareVersion(int maj, int min, int eng)
        {
            this.maj = maj;
            this.min = min;
            this.eng = eng;
        }

        @Override
        public String toString()
        {
            return String.format("%d.%d.%d", maj, min, eng);
        }
    }

    /**
     * Get the firmware version running on the OctoQuad
     * @return the firmware version running on the OctoQuad
     */
    FirmwareVersion getFirmwareVersion();

    /**
     * Read a single position from the OctoQuad
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     * @param idx the index of the encoder to read
     * @return the position for the specified encoder
     */
    int readSinglePosition(int idx);

    /**
     * Reads all positions from the OctoQuad, writing the data into
     * an existing int[] object. The previous values are destroyed.
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     * @param out the int[] object to fill with new data
     */
    void readAllPositions(int[] out);

    /**
     * Reads all positions from the OctoQuad
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     * @return an int[] object with the new data
     */
    int[] readAllPositions();

    /**
     * Read a selected range of encoders
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     * @param idxFirst the first encoder (inclusive)
     * @param idxLast the last encoder (inclusive)
     * @return an array containing the requested encoder positions
     */
    int[] readPositionRange(int idxFirst, int idxLast);

    /*
     * More Reset methods in Base interface
     */

    /**
     * Reset multiple encoders in the OctoQuad firmware in one command
     * @param resets the encoders to be reset
     */
    void resetMultiplePositions(boolean[] resets);

    /**
     * Reset multiple encoders in the OctoQuad firmware in one command
     * @param indices the indices of the encoders to reset
     */
    void resetMultiplePositions(int... indices);

    /*
     * More direction methods in Base interface
     */

    /**
     * Set the direction for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param reverse 8-length direction array
     */
    void setAllEncoderDirections(boolean[] reverse);

    /**
     * Read a single velocity from the OctoQuad
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     * @param idx the index of the encoder to read
     * @return the velocity for the specified encoder
     */
    short readSingleVelocity(int idx);

    /**
     * Reads all velocities from the OctoQuad, writing the data into
     * an existing short[] object. The previous values are destroyed.
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     * @param out the short[] object to fill with new data
     */
    void readAllVelocities(short[] out);

    /**
     * Reads all velocities from the OctoQuad
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     * @return a short[] object with the new data
     */
    short[] readAllVelocities();

    /**
     * Read a selected range of encoder velocities
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     * @param idxFirst the first encoder (inclusive)
     * @param idxLast the last encoder (inclusive)
     * @return an array containing the requested velocities
     */
    short[] readVelocityRange(int idxFirst, int idxLast);

    class EncoderDataBlock
    {
        public int[] positions = new int[NUM_ENCODERS];
        public short[] velocities = new short[NUM_ENCODERS];
        public boolean crcOk;

        /**
         * Check whether it is likely that this data is valid by checking if the CRC
         * on the returned is bad. For example, if there is an I2C bus stall or bit flip,
         * you could avoid acting on corrupted data.
         * @return whether it is likely that this data is valid
         */
        public boolean isDataValid()
        {
            return crcOk;
        }
    }

    /**
     * Reads all encoder data from the OctoQuad, writing the data into
     * an existing {@link EncoderDataBlock} object. The previous values are destroyed.
     * @param out the {@link EncoderDataBlock} object to fill with new data
     */
    void readAllEncoderData(EncoderDataBlock out);

    /**
     * Reads all encoder data from the OctoQuad
     * @return a {@link EncoderDataBlock} object with the new data
     */
    EncoderDataBlock readAllEncoderData();

    /*
     * More velocity sample interval methods in base interface
     */

    /**
     * Set the velocity sample intervals for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param intvlms the sample intervals in milliseconds
     */
    void setAllVelocitySampleIntervals(int[] intvlms);

    /**
     * Reads all velocity sample intervals from the OctoQuad
     * @return all velocity sample intervals from the OctoQuad
     */
    int[] getAllVelocitySampleIntervals();

    class ChannelPulseWidthParams
    {
        public int min_length_us;
        public int max_length_us;

        public ChannelPulseWidthParams() {};

        public ChannelPulseWidthParams(int min_length_us, int max_length_us)
        {
            this.min_length_us = min_length_us;
            this.max_length_us = max_length_us;
        }
    }

    /**
     * Configure the minimum/maximum pulse width reported by an absolute encoder
     * which is connected to a given channel, to allow the ability to provide
     * accurate velocity data.
     * These parameters will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the channel in question
     * @param params minimum/maximum pulse width
     */
    void setSingleChannelPulseWidthParams(int idx, ChannelPulseWidthParams params);

    /**
     * Queries the OctoQuad to determine the currently set minimum/maxiumum pulse
     * width for an encoder channel, to allow sane velocity data.
     * @param idx the channel in question
     * @return minimum/maximum pulse width
     */
    ChannelPulseWidthParams getSingleChannelPulseWidthParams(int idx);

    /**
     * Configure whether in PWM mode, a channel will report the raw PWM length
     * in microseconds, or whether it will perform "wrap tracking" for use with
     * an absolute encoder to turn the absolute position into a continuous value
     *
     * This is useful if you want your absolute encoder to track position across
     * multiple rotations. NB: in order to get sane data, you MUST set the channel
     * min/max pulse width parameter. Do not assume these values are the same for each
     * encoder, even if they are from the same production run! REV Through Bore encoders
     * have been observed to vary +/- 10uS from the spec'd values. You need to
     * actually test each encoder manually by turning it very slowly until it gets to
     * the wrap around point, and making note of the max/min values that it flips between,
     * and then program those values into the respective channel's min/max pulse width parameter.
     *
     * Note that when using a PWM channel in "wrap tracking" mode, issuing a reset to
     * that channel does NOT clear the position register to zero. Rather, it zeros the
     * "accumulator" (i.e. the number of "wraps") such that immediately after issuing a
     * reset command, the position register will contain the raw pulse width in microseconds.
     * This behavior is chosen because it maintains the ability of the absolute encoder to
     * actually report its absolute position. If resetting the channel were to really zero
     * out the reported position register, using an absolute encoder would not behave any
     * differently than using a relative encoder.
     *
     * @param trackWrap whether to track wrap around
     */
    void setAllChannelsPulseWidthTracksWrap(boolean[] trackWrap);

    // ---------------------------------------------------------------------------------------------
    // ABSOLUTE LOCALIZER API - Supported on OctoQuad FTC Ed. MK2 with FW v3 only!
    // ---------------------------------------------------------------------------------------------

    /**
     * Bulk read all localizer data and encoder positions in one operation for maximum efficiency,
     * writing the data into existing objects. The previous values are destroyed.
     * @param localizerOut the {@link LocalizerDataBlock} object to fill with new localizer data
     * @param positionsOut the array to fill with new encoder position data
     */
    void readLocalizerDataAndEncoderPositions(LocalizerDataBlock localizerOut, int[] positionsOut);

    /**
     * Bulk read all localizer data and encoder data in one operation for maximum efficiency,
     * writing the data into existing objects. The previous values are destroyed.
     * @param localizerOut the {@link LocalizerDataBlock} object to fill with new localizer data
     * @param encoderOut the {@link EncoderDataBlock} object to fill with new encoder data
     */
    void readLocalizerDataAndAllEncoderData(LocalizerDataBlock localizerOut, EncoderDataBlock encoderOut);
}

