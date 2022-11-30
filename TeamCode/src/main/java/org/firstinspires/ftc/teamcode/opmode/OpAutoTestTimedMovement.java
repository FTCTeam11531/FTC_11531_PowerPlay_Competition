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
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.utility.RobotInitialization;

// Program Copied from FTC example: ConceptExternalHardwareCLass.java
// Renamed in TeamCode as: OpAutoTestTimedMovement.java

/**
 * <h2>FTC Driver Station Autonomous OpMode/Command: Test Timed Movement</h2>
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
@Autonomous(name="Test: Timed Movement", group="z_test")
@Disabled
public class OpAutoTestTimedMovement extends LinearOpMode {
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

    // -- LinearSlide System
    SysLinearSlide sysLinearSlide = new SysLinearSlide(this);

    // -- Vision System
    //SysVision sysVision = new SysVision(this);

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
        //sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

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
        // Allow for Start Option Selection
        // ------------------------------------------------------------
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_PREGAME_OPTION_CONFIG);

        // Reset runtime clock
        runtime.reset();

        // Robot Initialization Settings - Autonomous
        utilRobotInit.displayRobotInitializationSettings(RobotConstants.CommonSettings.INIT_SETTING_DISPLAY_MODE_AUTONOMOUS);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // Reset runtime/lighting to Default
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_AUTONOMOUS);
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
            // [y] Axial forward =  1 | backward = -1
            // [x] lateral left = -1 | right = 1

            // Drive forward for 2 seconds
            inputAxial = 1;
            inputLateral = 0; // forward
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive left for 2 seconds
            inputAxial = 0;  // left
            inputLateral = -1;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive right for 2 seconds
            inputAxial = 0; // right
            inputLateral = 1;
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive backwards for 2 seconds
            inputAxial = -1;
            inputLateral = 0; // backwards
            inputYaw = 0;
            inputTimeSeconds = 2;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);

            // Drive rotate for .5 seconds
            inputAxial = 0;
            inputLateral = 0; // backwards
            inputYaw = 1;
            inputTimeSeconds = .5;
            sysDrivetrain.driveInputTimed(inputAxial, inputLateral, inputYaw, inputOutputPower, inputTimeSeconds);


            // Position 2
            sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO);

        }

        // Update the Transition Adjustment Value for the IMU
        RobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

        // ------------------------------------------------------------
        // - send telemetry to driver hub
        // ------------------------------------------------------------
        telemetry.update();

    }
}
