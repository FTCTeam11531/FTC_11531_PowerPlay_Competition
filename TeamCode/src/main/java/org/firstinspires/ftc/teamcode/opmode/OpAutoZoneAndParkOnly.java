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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.system.SysClaw;
import org.firstinspires.ftc.teamcode.system.SysLighting;
import org.firstinspires.ftc.teamcode.system.SysLinearSlide;
import org.firstinspires.ftc.teamcode.system.SysVision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.system.SysDrivetrain;
import org.firstinspires.ftc.teamcode.utility.RobotInitialization;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

// Program Copied from FTC example: ConceptExternalHardwareCLass.java
// Renamed in TeamCode as: OpAutoZoneAndParkOnly.java

/**
 * <h2>FTC Driver Station Autonomous OpMode/Command: Zone and Park</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * This Autonomous Command/Mode will perform the following autonomous actions:<br>
 * - Read Custom Zone Marker<br>
 * - Move to correct Zone indicated by marker<br>
 * - Park completely within the zone<br>
 * </p>
 * <hr>
 */
@Autonomous(name="Zone and Park", group="_auto")
//@Disabled
public class OpAutoZoneAndParkOnly extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion
    RobotInitialization utilRobotInit = new RobotInitialization(this);

    // -- Lighting System
    SysLighting sysLighting = new SysLighting(this);

    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- Claw System
    SysClaw sysClaw = new SysClaw(this);

    // -- Vision System
    SysVision sysVision = new SysVision(this);

    // Settings for captured image
    Recognition recognitionTargetZone;

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();
    private int targetZone = 0;

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

        sysVision.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysClaw.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_CLAW);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // -- Set to run using encoder for encoder drive mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // Reset runtime/lighting to Default
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_DEFAULT);
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) return;

        if (opModeIsActive()) {

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Hardware Profile: ", RobotConstants.CommonSettings.getRobotConfigurationFileManagerNameActive());
            telemetry.update();

            // ------------------------------------------------------------
            // Get Cone Sleeve Image
            // ------------------------------------------------------------
            boolean isImageFound = false;
            while (opModeIsActive() && !isImageFound) {

                recognitionTargetZone = sysVision.getRecognition(sysVision.getRecognitionList());

                if(recognitionTargetZone != null) {
                    isImageFound = true;
                }

            }

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("TensorFlow Model File: ", sysVision.getCurrentModelFileName());
            telemetry.addData("TensorFlow Model Path: ", sysVision.getCurrentModelFilePath());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Image: ", "%s (%.0f %% Conf.)", recognitionTargetZone.getLabel(), recognitionTargetZone.getConfidence() * 100 );
            telemetry.addData("- Position (Row/Col): ","%.0f / %.0f", sysVision.getRecognitionRow(recognitionTargetZone), sysVision.getRecognitionColumn(recognitionTargetZone));
            telemetry.addData("- Size (Width/Height): ","%.0f / %.0f", sysVision.getRecognitionWidth(recognitionTargetZone), sysVision.getRecognitionHeight(recognitionTargetZone));
            telemetry.update();

            // ------------------------------------------------------------
            // Claw - Clamp starting cone
            // ------------------------------------------------------------
            sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_CLOSE);
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_CLOSED);

            // ------------------------------------------------------------
            // Drive and Park in Correct Zone!
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Drivetrain");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Drivetrain Mode", sysDrivetrain.getLabelDrivetrainMode());
            telemetry.addData("Drivetrain Power", sysDrivetrain.getLabelDrivetrainOutputPower());
            telemetry.addData("-", "------------------------------");
            telemetry.update();

            // Get the target zone from recongnition
            targetZone = sysVision.getTargetZone(recognitionTargetZone.getLabel());

            switch (targetZone) {

                // ---------------------------
                // Drive to Zone 1
                // ---------------------------
                case 1:
                    telemetry.addData("Zone 1", recognitionTargetZone.getLabel());
                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- Before (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Drive to Zone 1
                    // ---------------------------
                    // [Y] - Axial - Driving forward and backward
                    // [X] - Lateral - Strafing right and left
                    // [R] - Yaw - Rotating Clockwise and counter clockwise

                    // Drive Forward - Two Seconds
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(1, 0, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 2);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Drive Left - One Second
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(0, -1, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 1);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Zone Parking Complete
                    // ---------------------------
                    sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE);
                    break;

                // ---------------------------
                // Drive to Zone 2
                // ---------------------------
                case 2:
                    telemetry.addData("Zone 2", recognitionTargetZone.getLabel());
                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- Before (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Drive to Zone 2
                    // ---------------------------
                    // [Y] - Axial - Driving forward and backward
                    // [X] - Lateral - Strafing right and left
                    // [R] - Yaw - Rotating Clockwise and counter clockwise

                    // Drive Forward - Two Seconds
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(1, 0, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 2);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Zone Parking Complete
                    // ---------------------------
                    sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO);
                    break;

                // ---------------------------
                // Drive to Zone 3
                // ---------------------------
                case 3:
                    telemetry.addData("Zone 3", recognitionTargetZone.getLabel());
                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- Before (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Drive to Zone 3
                    // ---------------------------
                    // [Y] - Axial - Driving forward and backward
                    // [X] - Lateral - Strafing right and left
                    // [R] - Yaw - Rotating Clockwise and counter clockwise

                    // Drive Forward - Two Seconds
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(1, 0, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 2);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Drive Right - One Second
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(0, 1, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 1);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Zone Parking Complete
                    // ---------------------------
                    sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE);
                    break;

                // ------------------------------------------------
                // Action to perform when Zone was not determined!
                // ------------------------------------------------
                default:
                    telemetry.addData("None", recognitionTargetZone.getLabel());
                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- Before (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Default
                    // ---------------------------
                    // [Y] - Axial - Driving forward and backward
                    // [X] - Lateral - Strafing right and left
                    // [R] - Yaw - Rotating Clockwise and counter clockwise

                    // Drive Forward - Two Seconds
                    // ---------------------------
                    sysDrivetrain.driveInputTimed(1, 0, 0, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED, 2);

                    telemetry.addData("-", "------------------------------");
                    telemetry.addData("- After (Heading)", sysDrivetrain.getRobotHeadingAdj());
                    telemetry.addData("Encoder Front left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
                    telemetry.addData("Encoder Back  left/Right", "%5f, %5f"
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                            , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
                    telemetry.update();

                    // Zone Parking Invalid :(
                    // ---------------------------
                    sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_INVALID);
            }

            // ------------------------------------------------------------
            // - Lighting telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Lighting");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());
            telemetry.update();

        }

        // Update the Transition Adjustment Value for the IMU
        RobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

        // ------------------------------------------------------------
        // - send telemetry to driver hub
        // ------------------------------------------------------------
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "-- Autonomous Routine Complete");
        telemetry.addData("-", "------------------------------");
        telemetry.update();

        // Set telemetry mode back to auto-clear
        telemetry.setAutoClear(true);

        // Pace this loop so hands move at a reasonable speed.
        //sleep(50);
    }
}
