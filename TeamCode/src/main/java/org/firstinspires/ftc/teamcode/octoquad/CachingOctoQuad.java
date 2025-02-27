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


public interface CachingOctoQuad extends OctoQuadBase
{
    /**
     * Controls how data is cached to reduce the number
     * of Lynx transactions needed to read the encoder data
     */
    enum CachingMode
    {
        /*
         * No caching at all
         */
        NONE,

        /**
         * The cache is only updated when you call {@link #refreshCache()}
         */
        MANUAL,

        /**
         * The cache is updated the 2nd time you call {@link #readSinglePosition_Caching(int)} (int)}
         * or {@link #readSingleVelocity_Caching(int)} (int)} for the same encoder index.
         */
        AUTO
    }

    /**
     * Set the data caching mode for the OctoQuad
     * @param mode mode to use
     */
    void setCachingMode(CachingMode mode);

    /**
     * Manually refresh the position and velocity data cache
     */
    void refreshCache();

    /**
     * Read a single position from the OctoQuad
     * Note this call may return cached data based on
     * the {@link CachingMode} you selected!
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     * @param idx the index of the encoder to read
     * @return the position for the specified encoder
     */
    int readSinglePosition_Caching(int idx);

    /**
     * Read a single velocity from the OctoQuad
     * Note this call may return cached data based on
     * the {@link CachingMode} you selected!
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     * @param idx the index of the encoder to read
     * @return the velocity for the specified encoder
     */
    short readSingleVelocity_Caching(int idx);
}
