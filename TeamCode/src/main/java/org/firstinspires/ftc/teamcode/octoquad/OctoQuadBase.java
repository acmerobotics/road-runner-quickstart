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

public interface OctoQuadBase
{
    byte OCTOQUAD_CHIP_ID = 0x51;
    int ENCODER_FIRST = 0;
    int ENCODER_LAST = 7;
    int NUM_ENCODERS = 8;
    int MIN_VELOCITY_MEASUREMENT_INTERVAL_MS = 1;
    int MAX_VELOCITY_MEASUREMENT_INTERVAL_MS = 255;
    int MIN_PULSE_WIDTH_US = 0;  //  The symbol for microseconds is μs, but is sometimes simplified to us.
    int MAX_PULSE_WIDTH_US = 0xFFFF;

    /**
     * Reads the CHIP_ID register of the OctoQuad
     * @return the value in the CHIP_ID register of the OctoQuad
     */
    byte getChipId();

    /**
     * Get the firmware version running on the OctoQuad
     * @return the firmware version running on the OctoQuad
     */
    String getFirmwareVersionString();

    /**
     * Reset a single encoder in the OctoQuad firmware
     * @param idx the index of the encoder to reset
     */
    void resetSinglePosition(int idx);

    /**
     * Reset all encoder counts in the OctoQuad firmware
     */
    void resetAllPositions();

    enum EncoderDirection
    {
        FORWARD,
        REVERSE
    }

    /**
     * Set the direction for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the index of the encoder
     * @param dir direction
     */
    void setSingleEncoderDirection(int idx, EncoderDirection dir);

    /**
     * Get the direction for a single encoder
     * @param idx the index of the encoder
     * @return direction of the encoder in question
     */
    EncoderDirection getSingleEncoderDirection(int idx);

    /**
     * Set the velocity sample interval for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the index of the encoder in question
     * @param intvlms the sample interval in milliseconds
     */
    void setSingleVelocitySampleInterval(int idx, int intvlms);

    /**
     * Set the velocity sample interval for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param intvlms the sample interval in milliseconds
     */
    void setAllVelocitySampleIntervals(int intvlms);

    /**
     * Read a single velocity sample interval
     * @param idx the index of the encoder in question
     * @return the velocity sample interval
     */
    int getSingleVelocitySampleInterval(int idx);

    /**
     * Configure the minimum/maximum pulse width reported by an absolute encoder
     * which is connected to a given channel, to allow the ability to provide
     * accurate velocity data.
     * These parameters will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the channel in question
     * @param min_length_us minimum pulse width
     * @param max_length_us maximum pulse width
     */
    void setSingleChannelPulseWidthParams(int idx, int min_length_us, int max_length_us);

    /**
     * Configure whether in PWM mode, a channel will report the raw PWM length
     * in microseconds, or whether it will perform "wrap tracking" for use with
     * an absolute encoder to turn the absolute position into a continuous value.
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
     * @param idx the channel in question
     * @param trackWrap whether to track wrap around
     */
    void setSingleChannelPulseWidthTracksWrap(int idx, boolean trackWrap);

    /**
     * Get whether PWM wrap tracking is enabled for a single channel
     * @param idx the channel in question
     * @return whether PWM wrap tracking is enabled
     */
    boolean getSingleChannelPulseWidthTracksWrap(int idx);

    /**
     * Run the firmware's internal reset routine
     */
    void resetEverything();

    enum ChannelBankConfig
    {
        /**
         * Both channel banks are configured for Quadrature input
         */
        ALL_QUADRATURE(0),

        /**
         * Both channel banks are configured for pulse width input
         */
        ALL_PULSE_WIDTH(1),

        /**
         * Bank 1 (channels 0-3) is configured for Quadrature input;
         * Bank 2 (channels 4-7) is configured for pulse width input.
         */
        BANK1_QUADRATURE_BANK2_PULSE_WIDTH(2);

        public byte bVal;

        ChannelBankConfig(int bVal)
        {
            this.bVal = (byte) bVal;
        }
    }

    /**
     * Configures the OctoQuad's channel banks
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param config the channel bank configuration to use
     */
    void setChannelBankConfig(ChannelBankConfig config);

    /**
     * Queries the OctoQuad to determine the current channel bank configuration
     * @return the current channel bank configuration
     */
    ChannelBankConfig getChannelBankConfig();

    enum I2cRecoveryMode
    {
        /**
         * Does not perform any active attempts to recover a wedged I2C bus
         */
        NONE(0),

        /**
         * The OctoQuad will reset its I2C peripheral if 50ms elapses between
         * byte transmissions or between bytes and start/stop conditions
         */
        MODE_1_PERIPH_RST_ON_FRAME_ERR(1),

        /**
         * Mode 1 actions + the OctoQuad will toggle the clock line briefly,
         * once, after 1500ms of no communications.
         */
        MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL(2);

        public byte bVal;

        I2cRecoveryMode(int bVal)
        {
            this.bVal = (byte) bVal;
        }
    }

    /**
     * Configures the OctoQuad to use the specified I2C recovery mode.
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param mode the recovery mode to use
     */
    void setI2cRecoveryMode(I2cRecoveryMode mode);

    /**
     * Queries the OctoQuad to determine the currently configured I2C recovery mode
     * @return the currently configured I2C recovery mode
     */
    I2cRecoveryMode getI2cRecoveryMode();

    /**
     * Stores the current state of parameters to flash, to be applied at next boot
     */
    void saveParametersToFlash();

    // ---------------------------------------------------------------------------------------------
    // ABSOLUTE LOCALIZER API - Supported on OctoQuad FTC Ed. MK2 with FW v3 only!
    // ---------------------------------------------------------------------------------------------

    /**
     * A data block holding all localizer data needed for navigation; this block may be bulk
     * read in one I2C read operation.
     */
    class LocalizerDataBlock
    {
        public LocalizerStatus localizerStatus;
        public boolean crcOk;
        public float heading_rad;
        public short posX_mm;
        public short posY_mm;
        public short velX_mmS;
        public short velY_mmS;
        public float velHeading_radS;

        /**
         * Check whether it is likely that this data is valid. The localizer status is read
         * along with the data, and if the status is not RUNNING_FUSION, then the data invalid.
         * Additionally, if the CRC on the returned is bad, (e.g. if there is an I2C bus stall
         * or bit flip), you could avoid acting on that corrupted data.
         * @return whether it is likely that this data is valid
         */
        public boolean isDataValid()
        {
            return crcOk && localizerStatus == LocalizerStatus.RUNNING_FUSION;
        }
    }

    /**
     * Enum representing the status of the absolute localizer algorithm
     * You can poll this to determine when IMU calibration has finished.
     */
    enum LocalizerStatus
    {
        INVALID(0),
        NOT_INITIALIZED(1),
        WARMING_UP_IMU(2),
        CALIBRATING_IMU(3),
        RUNNING_FUSION(4),
        FAULT_NO_IMU(5);

        final int code;

        LocalizerStatus(int code)
        {
            this.code = code;
        }
    }

    /**
     * The absolute localizer automagically selects the appropriate axis on the IMU to use
     * for yaw, regardless of physical mounting orientation. (However, you must mount in one
     * of the 6 axis-orthogonal orientations). You can poll this status to determine which
     * axis was chosen.
     */
    enum LocalizerYawAxis
    {
        UNDECIDED(0),
        X(1),
        X_INV(2),
        Y(3),
        Y_INV(4),
        Z(5),
        Z_INV(6);

        int code;

        LocalizerYawAxis(int code)
        {
            this.code = code;
        }
    }

    /**
     * Query which IMU axis the localizer decided to use for heading
     * @return which IMU axis the localizer decided to use for heading
     */
    LocalizerYawAxis getLocalizerHeadingAxisChoice();

    /**
     * Set the port index to be used by the absolute localizer routine for measuring X movement
     * @param port the port index to be used by the absolute localizer routine for measuring X movement
     */
    void setLocalizerPortX(int port);

    /**
     * Set the port index to be used by the absolute localizer routine for measuring Y movement
     * @param port the port index to be used by the absolute localizer routine for measuring Y movement
     */
    void setLocalizerPortY(int port);

    /**
     * Set the scalar for converting encoder counts on the X port to millimeters of travel
     * You SHOULD NOT calculate this - this is a real world not a theoretical one - you should
     * measure this by pushing your robot N meters and dividing the counts by (N*1000)
     * @param ticksPerMM_x scalar for converting encoder counts on the X port to millimeters of travel
     */
    void setLocalizerCountsPerMM_X(float ticksPerMM_x);

    /**
     * Set the scalar for converting encoder counts on the Y port to millimeters of travel
     * You SHOULD NOT calculate this - this is a real world not a theoretical one - you should
     * measure this by pushing your robot N meters and dividing the counts by (N*1000)
     * @param ticksPerMM_y scalar for converting encoder counts on the X port to millimeters of travel
     */
    void setLocalizerCountsPerMM_Y(float ticksPerMM_y);

    /**
     * The real TCP (Tracking Center Point) of your robot is the point on the robot, where,
     * if the robot rotates about that point, neither the X nor Y tracking wheels will rotate.
     * This point can be determined by drawing imaginary lines parallel to, and through the middle
     * of, your tracking wheels, and finding where those lines intersect.
     *
     * Without setting any TCP offset, if the robot rotates, the reported XY vales from the localizer
     * will change during the rotation, unless the rotation is about the real TCP. Most users will
     * find this behavior rather unhelpful, since usually a robot will robot about its geometric center,
     * and not about the real TCP.
     *
     * You can move the location of the localize's "virtual" TCP away from the true location (e.g. to
     * the center of your robot) by applying an offset. This value does not need to be extremely accurate
     * since it will not affect the accuracy or repeatability of the localizer algorithm.
     * @param tcpOffsetMM_X offset to move the virtual TCP
     */
    void setLocalizerTcpOffsetMM_X(float tcpOffsetMM_X);

    /**
     * The real TCP (Tracking Center Point) of your robot is the point on the robot, where,
     * if the robot rotates about that point, neither the X nor Y tracking wheels will rotate.
     * This point can be determined by drawing imaginary lines parallel to, and through the middle
     * of, your tracking wheels, and finding where those lines intersect.
     *
     * Without setting any TCP offset, if the robot rotates, the reported XY vales from the localizer
     * will change during the rotation, unless the rotation is about the real TCP. Most users will
     * find this behavior rather unhelpful, since usually a robot will robot about its geometric center,
     * and not about the real TCP.
     *
     * You can move the location of the localize's "virtual" TCP away from the true location (e.g. to
     * the center of your robot) by applying an offset. This value does not need to be extremely accurate
     * since it will not affect the accuracy or repeatability of the localizer algorithm.
     * @param tcpOffsetMM_Y offset to move the virtual TCP
     */
    void setLocalizerTcpOffsetMM_Y(float tcpOffsetMM_Y);

    /**
     * Set a scale factor to apply to the IMU heading to improve accuracy
     * The recommended way to tune this is to set the scalar to 1.0, place the robot against a hard
     * surface, rotate the robot 10 times (3600 degrees) and see how far off the reported heading is
     * from what it should be.
     * @param headingScalar scale factor to apply to the IMU heading to improve accuracy
     */
    void setLocalizerImuHeadingScalar(float headingScalar);

    /**
     * Set the period of translational velocity calculation.
     * Longer periods give higher resolution with more latency,
     * shorter periods give lower resolution with less latency.
     * @param ms 1-255ms
     */
    void setLocalizerVelocityIntervalMS(int ms);

    /**
     * Set all localizer parameters with one function call
     * @param portX
     * @param portY
     * @param ticksPerMM_x
     * @param ticksPerMM_y
     * @param tcpOffsetMM_X
     * @param tcpOffsetMM_Y
     * @param headingScalar
     * @param velocityIntervalMs
     */
    void setAllLocalizerParameters(
            int portX,
            int portY,
            float ticksPerMM_x,
            float ticksPerMM_y,
            float tcpOffsetMM_X,
            float tcpOffsetMM_Y,
            float headingScalar,
            int velocityIntervalMs
    );

    /**
     * Bulk read all localizer data in one operation for maximum efficiency, writing the data into
     * an existing {@link LocalizerDataBlock} object. The previous values are destroyed.
     * @param out the {@link LocalizerDataBlock} object to fill with new data
     */
    void readLocalizerData(LocalizerDataBlock out);

    /**
     * Bulk read all localizer data in one operation for maximum efficiency
     * @return newly read localizer data
     */
    LocalizerDataBlock readLocalizerData();

    /**
     * "Teleport" the localizer to a new location. This may be useful, for instance, for updating
     * your position based on vision targeting.
     * @param posX_mm x position in millimeters
     * @param posY_mm y position in millimeters
     * @param heading_rad heading in radians
     */
    void setLocalizerPose(int posX_mm, int posY_mm, float heading_rad);

    /**
     * Teleport the localizer heading to a new orientation. This may be useful,
     * for instance, for updating heading based on vision targeting.
     * @param heading_rad heading in radians
     */
    void setLocalizerHeading(float heading_rad);

    /**
     * Get the current status of the localizer algorithm
     * @return current status of the localizer algorithm
     */
    LocalizerStatus getLocalizerStatus();

    /**
     * Reset the localizer pose to (0,0,0) and recalibrate the IMU.
     * Poll {@link #getLocalizerStatus()} for {@link LocalizerStatus#RUNNING_FUSION}
     * to determine when the reset is complete.
     */
    void resetLocalizer();
}
