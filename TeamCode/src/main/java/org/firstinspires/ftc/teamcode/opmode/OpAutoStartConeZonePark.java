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

package org.firstinspires.ftc.teamcode.opmode;

import android.app.Activity;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.system.SysClaw;
import org.firstinspires.ftc.teamcode.system.SysDrivetrain;
import org.firstinspires.ftc.teamcode.system.SysLighting;
import org.firstinspires.ftc.teamcode.system.SysLinearSlide;
import org.firstinspires.ftc.teamcode.system.SysVision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.utility.RobotInitialization;

// Program Copied from FTC example: ConceptExternalHardwareCLass.java
// Renamed in TeamCode as: OpAutoStartConeZonePark.java

/**
 * <h2>FTC Driver Station Autonomous OpMode/Command: Start Cone | Zone Park</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * This Autonomous Command/Mode will perform the following autonomous actions:<br>
 * - Read Custom Zone Marker<br>
 * - Move to High Goal<br>
 * - Place Start Cone on Goal<br>
 * - Move to correct Zone indicated by marker<br>
 * - Park completely within the zone<br>
 * </p>
 * <hr>
 */
@Autonomous(name="Start Cone | Zone Park", group="_auto")
//@Disabled
public class OpAutoStartConeZonePark extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion
    RobotInitialization utilRobotInit = new RobotInitialization(this);

    // -- Lighting System
    SysLighting sysLighting = new SysLighting(this);

    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- Vision System
    //SysVision sysVision = new SysVision(this);

    // -- LinearSlide System
    SysLinearSlide sysLinearSlide = new SysLinearSlide(this);

    // -- Claw System
    SysClaw sysClaw = new SysClaw(this);

    // Settings for captured image
    Recognition recognitionTargetZone;

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------
        utilRobotInit.init();

        sysLighting.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

        sysDrivetrain.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        //sysVision.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysLinearSlide.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LINEARSLIDE);

        sysClaw.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_CLAW);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // -- Set to run without encoders for timed drive mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ------------------------------------------------------------
        // Inputs for OpMode
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw, inputTimeSeconds, inputOutputPower;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // ------------------------------------------------------------
        // Allow for Start Option Selection
        // ------------------------------------------------------------
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_PREGAME_OPTION_CONFIG);

        // Reset runtime clock
        runtime.reset();

        // Robot Initialization Settings - Autonomous
        utilRobotInit.displayInitializationSettingsAutonomous(RobotConstants.CommonSettings.INIT_SETTING_DISPLAY_MODE_AUTONOMOUS);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) return;

        if (opModeIsActive()) {

            // ------------------------------------------------------------
            // Commands to Run
            // ------------------------------------------------------------
            // Close Clamp at start
            sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_CLOSE);

            // Move Linear Slide to High Goal
            sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_HIGH_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH);

            // Test drive 24 inches forward
            sysDrivetrain.driveDistanceAxial(48, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive turn to heading 180
            sysDrivetrain.driveTurnToHeading(180, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 24 inches forward (went backward)
            sysDrivetrain.driveDistanceAxial(12, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 12 inches left
//            sysDrivetrain.driveDistanceLateral(-12, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Move Linear Slide to High Goal
            while (opModeIsActive() && sysLinearSlide.getLinearSlideCurrentPosition(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_PRIMARY) != RobotConstants.LinearSlide.ENCODER_SET_POINT_HIGH_GOAL) {
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_HIGH_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH);
            }

            // Open the Claw
            sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_OPEN);
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_OPEN);

            // Test drive 12 inches right //??
            sysDrivetrain.driveDistanceLateral(12, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive turn to heading 90
            //sysDrivetrain.driveTurnToHeading(90, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 12 inches left
            //sysDrivetrain.driveDistanceLateral(-12, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 24 inches right
            //sysDrivetrain.driveDistanceLateral(24, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 24 inches backward
            //sysDrivetrain.driveDistanceAxial(-24, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);

            // Test drive 24 inches left
            //sysDrivetrain.driveDistanceLateral(-24, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);


            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // ------------------------------------------------------------
            // - Telemetry Data
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Autonomous               --");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "<No Data Available>");

        }

        // Update the Transition Adjustment Value for the IMU
        RobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

        // ------------------------------------------------------------
        // - send telemetry to driver hub
        // ------------------------------------------------------------
        telemetry.update();

    }
}
