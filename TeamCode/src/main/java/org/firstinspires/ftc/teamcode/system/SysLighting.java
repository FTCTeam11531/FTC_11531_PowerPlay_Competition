/* Copyright (c) 2022 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Arrays;
import java.util.List;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysLighting.java

/**
 * <h2>System Class for the Robot Lighting and Effects</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <b>Light Controller:</b> {@value RobotConstants.Lighting#COMMENT_LIGHTING_CONTROLLER}<br>
 * <hr>
 * <p>
 * This system defines <b>ALL</b> configures all attributes, configurations, and methods for the robot
 * LED lighting system.
 * </p>
 * <hr>
 *
 */
public class SysLighting {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private RevBlinkinLedDriver ledLightController;
    public RevBlinkinLedDriver.BlinkinPattern ledLightPattern;

    /**
     * <h2>Lighting System Constructor</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Define a constructor that allows the OpMode to pass a reference to itself.
     * </p>
     * <hr>
     * @param inOpMode Pass in Calling OpMode
     */
    public SysLighting(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Lighting System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the lighting system.
     * This method must be called <b>ONCE</b> when the OpMode is initialized.
     * </p>
     * <br>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     * <hr>
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        ledLightController = sysOpMode.hardwareMap.get(RevBlinkinLedDriver.class, RobotConstants.Configuration.LABEL_CONTROLLER_LIGHTING);

        // Set the initial lighting pattern
        ledLightPattern = RobotConstants.Lighting.LIGHT_PATTERN_DEFAULT;

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: Lighting Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Lighting Method: getLightPatternNext</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the next light pattern
     * </p>
     * @return RevBlinkinLedDriver.BlinkinPattern - LED Light Pattern (next)
     */
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternNext() {

        // Get Next LED Light Pattern
        RevBlinkinLedDriver.BlinkinPattern outLightPattern = ledLightPattern.next();

        // Avoid Patterns that currently appear to cause the pattern to stop cycling
        while((sysOpMode.opModeIsActive() || sysOpMode.opModeInInit()) && checkValidLightPattern(outLightPattern)) {
            outLightPattern = ledLightPattern.next();
        }

        // Return the next light pattern
        return outLightPattern;
    }

    /**
     * <h2>Lighting Method: getLightPatternPrevious</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the previous light pattern
     * </p>
     * @return RevBlinkinLedDriver.BlinkinPattern - LED Light Pattern (previous)
     */
    public RevBlinkinLedDriver.BlinkinPattern getLightPatternPrevious() {

        // Get Next LED Light Pattern
        RevBlinkinLedDriver.BlinkinPattern outLightPattern = ledLightPattern.previous();

        // Avoid Patterns that currently appear to cause the pattern to stop cycling
        while((sysOpMode.opModeIsActive() || sysOpMode.opModeInInit()) && checkValidLightPattern(outLightPattern)) {
            outLightPattern = ledLightPattern.previous();
        }

        // Return the previous light pattern
        return outLightPattern;
    }

    /**
     * <h2>Lighting Method: checkValidLightPattern</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Check that the light pattern is a valid pattern
     * </p>
     * @param inLightPattern RevBlinkinLedDriver.BlinkinPattern - Light Pattern Setting
     *
     * @return boolean - True of valid / False if invalid
     */
    public boolean checkValidLightPattern(RevBlinkinLedDriver.BlinkinPattern inLightPattern) {

        // Variable to check if pattern is valid
        // Pattern will always be true to start and false if the checks find a match
        boolean isValid = true;

        // Check pattern against avoided pattern(s) when pattern is not one of the avoided keywords

        // Avoid TWINKLES
        if(inLightPattern.toString().contains(RobotConstants.Lighting.LIGHT_PATTERN_AVOID_KEYWORD_TWINKLES)) {
            isValid = false;
        }

        return isValid;
    }

    /**
     * <h2>Lighting Method: setLightPattern</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set/Display the light pattern to
     * </p>
     * @param inLightPattern RevBlinkinLedDriver.BlinkinPattern - Light Pattern Setting
     *
     * <br>
     */
    public void setLightPattern(RevBlinkinLedDriver.BlinkinPattern inLightPattern) {

        // Check to verify the pattern is valid
        if(checkValidLightPattern(inLightPattern)) {

            // Set the Light Pattern on the Lighting Controller
            ledLightPattern = inLightPattern;
            ledLightController.setPattern(ledLightPattern);
        }

    }

}
