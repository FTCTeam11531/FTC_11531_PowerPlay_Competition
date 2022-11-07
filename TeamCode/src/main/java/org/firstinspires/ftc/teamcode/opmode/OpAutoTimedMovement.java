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
import java.util.List;

// Program Copied from FTC example: ConceptExternalHardwareCLass.java
// Renamed in TeamCode as: OpAutoTimedMovement.java

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
@Autonomous(name="Auto: Timed Movement", group="_auto")
//@Disabled
public class OpAutoTimedMovement extends LinearOpMode {
    // ------------------------------------------------------------
    // Robot Configuration
    // ------------------------------------------------------------
    RobotConfigFileManager robotConfigFileManager;
    String robotConfigName;

    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- Claw System
    SysClaw sysClaw = new SysClaw(this);

    // -- LinearSlide System
    SysLinearSlide sysLinearSlide = new SysLinearSlide(this);

    // -- Lighting System
    SysLighting sysLighting = new SysLighting(this);

    // -- Vision System
    SysVision sysVision = new SysVision(this);

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
        // Get Hardware Configuration Profile Name
        // ------------------------------------------------------------
        robotConfigFileManager = new RobotConfigFileManager((Activity) hardwareMap.appContext);
        robotConfigName = robotConfigFileManager.getActiveConfig().getName();

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        sysLighting.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

        sysDrivetrain.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysClaw.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_CLAW);

        sysLinearSlide.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LINEARSLIDE);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // -- Set to run without encoders for timed drive mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------------------------------------------
        // Inputs for: Drivetrain
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw, inputTimeSeconds, inputOutputPower;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // ------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        // ------------------------------------------------------------
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_DEFAULT);
        waitForStart();

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

            // ---------------------- //
            // SETUP FOR TESTING ONLY //
            // ---------------------- //
            inputOutputPower = .5;

            // ------------------------------------------------------------
            // Claw - Clamp starting cone
            // ------------------------------------------------------------
            sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_CLOSE);
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_CLOSED);

            // ------------------------------------------------------------
            // Commands to Run
            // ------------------------------------------------------------

            // Position 1
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE);

            // Current Config
            // forward =  -1 | backward = 1
            // left = -1 | right = 1

            // Drive forward for 2 seconds
            inputAxial = 0;
            inputLateral = -1;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive left for 2 seconds
            inputAxial = -1;
            inputLateral = 0;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive right for 2 seconds
            inputAxial = 1;
            inputLateral = 0;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive backwards for 2 seconds
            inputAxial = 0;
            inputLateral = 1;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Position 2
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO);
/*
        // Drive left (field centric) for 2 seconds
        inputAxial = 0;
        inputLateral = 1;
        inputYaw = 0;
        inputTimeSeconds = 2;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Drive right (field centric) for 4 seconds
        inputAxial = 0;
        inputLateral = -1;
        inputYaw = 0;
        inputTimeSeconds = 4;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Drive left (field centric) for 2 seconds
        inputAxial = 0;
        inputLateral = 1;
        inputYaw = 0;
        inputTimeSeconds = 2;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Drive backwards (field centric) for 2 seconds
        inputAxial = -1;
        inputLateral = 0;
        inputYaw = 0;
        inputTimeSeconds = 2;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Position 3
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE);

        // Drive left (field centric) for 2 seconds
        inputAxial = 0;
        inputLateral = 1;
        inputYaw = 0;
        inputTimeSeconds = 2;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Drive right (field centric) for 4 seconds
        inputAxial = 0;
        inputLateral = -1;
        inputYaw = 0;
        inputTimeSeconds = 4;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

        // Drive left (field centric) for 2 seconds
        inputAxial = 0;
        inputLateral = 1;
        inputYaw = 0;
        inputTimeSeconds = 2;
        sysDrivetrain.driveMecanumFieldCentricTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);
*/
            // Complete!!
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH);

/*
        // ------------------------------------------------------------
        // Driver Hub Feedback
        // ------------------------------------------------------------
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Hardware Profile: ", robotConfigName);

        // ------------------------------------------------------------
        // - Telemetry Data
        // ------------------------------------------------------------
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "-- Autonomous               --");
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "<No Data Available>");

        // ------------------------------------------------------------
        // - Drivetrain telemetry
        // ------------------------------------------------------------
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "-- Drivetrain");
        telemetry.addData("-", "------------------------------");
        telemetry.addData("Drivetrain Mode: ", sysDrivetrain.getLabelDrivetrainMode());
        telemetry.addData("Drivetrain Power: ", sysDrivetrain.getLabelDrivetrainOutputPower());
        telemetry.addData("-", "------------------------------");
        telemetry.addData("Front left/Right: ", "%4.2f, %4.2f"
                , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
        telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f"
                , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
        telemetry.addData("-", "------------------------------");
        telemetry.addData("Robot Heading: ", sysDrivetrain.getIMUHeading());

        // ------------------------------------------------------------
        // - Lighting telemetry
        // ------------------------------------------------------------
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "-- Lighting");
        telemetry.addData("-", "------------------------------");
        telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());

        // ------------------------------------------------------------
        // - Vision telemetry
        // ------------------------------------------------------------
        telemetry.addData("-", "------------------------------");
        telemetry.addData("-", "-- Vision");
        telemetry.addData("-", "------------------------------");
        telemetry.addData("TensorFlow Model File: ", sysVision.getCurrentModelFileName());
        telemetry.addData("TensorFlow Model Path: ", sysVision.getCurrentModelFilePath());
        telemetry.addData("-", "------------------------------");

        // Process results for each Recognition
        if (listVisionRecognitions != null) {
            telemetry.addData("Objects Detected: ", listVisionRecognitions.size());

            for (Recognition objRecognition : listVisionRecognitions) {
                double objColumn = (objRecognition.getLeft() + objRecognition.getRight()) / 2;
                double objRow = (objRecognition.getTop()  + objRecognition.getBottom()) / 2;
                double objWidth = Math.abs(objRecognition.getRight() - objRecognition.getLeft());
                double objHeight = Math.abs(objRecognition.getTop()  - objRecognition.getBottom());

                telemetry.addData("-", "------------------------------");
                telemetry.addData("Image: ", "%s (%.0f %% Conf.)", objRecognition.getLabel(), objRecognition.getConfidence() * 100 );
                telemetry.addData("- Position (Row/Col): ","%.0f / %.0f", objRow, objColumn);
                telemetry.addData("- Size (Width/Height): ","%.0f / %.0f", objWidth, objHeight);
            }

        }
        else {
            telemetry.addData("-", "<No Data Available>");
        }
*/
        }

        // Update the Transition Adjustment Value for the IMU
        RobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getIMUHeading());

        // ------------------------------------------------------------
        // - send telemetry to driver hub
        // ------------------------------------------------------------
        telemetry.update();

        // Pace this loop so hands move at a reasonable speed.
        //sleep(50);
    }
}
