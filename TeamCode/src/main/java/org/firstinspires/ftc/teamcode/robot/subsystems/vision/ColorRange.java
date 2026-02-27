/*
 * Copyright (c) 2024 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

/**
 * An {@link ColorRange represents a 3-channel minimum/maximum
 * range for a given color space}
 */
// note: for simplicity, removed colorspace parameter. assume HSV
@Config
public class ColorRange
{
    public final Scalar min;
    public final Scalar max;

    public static Scalar purpleLow1  = new Scalar(154, 84, 0);
    public static Scalar purpleHigh1 = new Scalar(166, 255, 255);
    public static Scalar purpleLow2  = new Scalar(255, 0, 0);
    public static Scalar purpleHigh2 = new Scalar(0, 255, 255);

    public static Scalar greenLow  = new Scalar(54, 0, 0);
    // Upper bound for green
    public static Scalar greenHigh = new Scalar(108, 255, 255);

    // todo: temporary values
    public static final ColorRange GREEN = new ColorRange(
            greenLow,
            greenHigh
    );

    public static final ColorRange PURPLE_1 = new ColorRange(
            purpleLow1,
            purpleHigh1
    );

    public static final ColorRange PURPLE_2 = new ColorRange(
            purpleLow2,
            purpleHigh2
    );

    public ColorRange(Scalar min, Scalar max)
    {
        this.min = min;
        this.max = max;
    }
}
