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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysSound.java

/**
 * <h2>System Class for the Robot Sound Effects</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 */
public class SysTargeting {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private DistanceSensor junctionRangeLeft;
    private DistanceSensor junctionRangeRight;
    private DistanceSensor junctionRangeMiddle;
    private RevColorSensorV3 test;

    /**
     * <h2>Targeting System Constructor</h2>
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
    public SysTargeting(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Targeting System Initialize</h2>
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
        junctionRangeLeft = sysOpMode.hardwareMap.get(DistanceSensor.class, RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_LEFT);
        junctionRangeRight = sysOpMode.hardwareMap.get(DistanceSensor.class, RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_RIGHT);
        junctionRangeMiddle = sysOpMode.hardwareMap.get(DistanceSensor.class, RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_MIDDLE);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: Targeting Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Lighting Method: checkTargetJunctionRangeTolerance</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Check the target junction range to see if it is in tolerance
     * </p>
     * @param inRangeSensorDistance
     * @return
     */
    public boolean checkTargetJunctionRangeTolerance(double inRangeSensorDistance) {
        boolean isInTargetJunctionRange;

        if(inRangeSensorDistance <= RobotConstants.Targeting.TARGET_JUNCTION_RANGE_TOLERANCE_MAX
//                && inRangeSensorDistance >= RobotConstants.Targeting.TARGET_JUNCTION_RANGE_TOLERANCE_MIN
        ) {
            isInTargetJunctionRange = true;
        }
        else {
            isInTargetJunctionRange = false;
        }

        return isInTargetJunctionRange;
    }

    /**
     * <h2>Lighting Method: getTargetJunctionRangeSensorDistance</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the distance from a target junction range sensor
     * </p>
     * @return double - Range / Distance
     */
    public double getTargetJunctionRangeSensorDistance(String inRangeSensorLabel) {
        // Variable for output distance / range
        double outDistance;

        // Get value for sensor
        switch (inRangeSensorLabel) {
            // Target Junction Range Sensor Left
            case(RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_LEFT):
                outDistance = junctionRangeLeft.getDistance(RobotConstants.Targeting.TARGET_JUNCTION_DISTANCE_UNIT);
                break;

            // Target Junction Range Sensor Right
            case(RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_RIGHT):
                outDistance = junctionRangeRight.getDistance(RobotConstants.Targeting.TARGET_JUNCTION_DISTANCE_UNIT);
                break;

            // Target Junction Range Sensor Middle
            case(RobotConstants.Configuration.LABEL_SENSOR_RANGE_JUNCTION_MIDDLE):
                outDistance = junctionRangeMiddle.getDistance(RobotConstants.Targeting.TARGET_JUNCTION_DISTANCE_UNIT);
                break;

            // default
            default:
                outDistance = 0;
        }

        // return value
        return outDistance;
    }

}
