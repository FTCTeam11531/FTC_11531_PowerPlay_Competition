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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.system.SysClaw;
import org.firstinspires.ftc.teamcode.system.SysDrivetrain;
import org.firstinspires.ftc.teamcode.system.SysLighting;
import org.firstinspires.ftc.teamcode.system.SysLinearSlide;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.utility.RobotInitialization;
import org.firstinspires.ftc.teamcode.utility.StateDriveMotorMaxOutputPower;
import org.firstinspires.ftc.teamcode.utility.StateDrivetrainMode;

// Program Copied from FTC example: ConceptExternalHardwareCLass.java
// Renamed in TeamCode as: OpTeleopMain.java

/**
 * <h2>FTC Driver Station Teleop OpMode/Command: Robot Main</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * This is the main Teleop OpMode/Command and will include all references to systems used
 * and will provide the mapping from input(s) to system methods.
 * </p>
 * <hr>
 */
@TeleOp(name="Robot Main", group="_main")
//@Disabled
public class OpTeleopMain extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion
    RobotInitialization utilRobotInit = new RobotInitialization(this);

    // -- Lighting System
    SysLighting sysLighting = new SysLighting(this);

    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- LinearSlide System
    SysLinearSlide sysLinearSlide = new SysLinearSlide(this);

    // -- Claw System
    SysClaw sysClaw = new SysClaw(this);

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

        sysLinearSlide.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LINEARSLIDE);

        sysClaw.init();
        sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_CLAW);

        // ------------------------------------------------------------
        // Configure drivetrain for Teleop Mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw;
        boolean isManualSlideMode = false;

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

//        waitForStart();

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        // Reset runtime clock
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) return;

        // ------------------------------------------------------------
        // Command Loop: run until the end of the match (driver presses STOP)
        // ------------------------------------------------------------
        while (opModeIsActive()) {

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_x, left_stick_y): Drive
            // -- -- Axis (right_stick_x): Rotate
            // -- -- X: 180 spin
            // -- -- Y: Set Output Speed to Med
            // -- -- A: Set Output Speed to Low
            // ------------------------------------------------------------
            // Gamepad2 = Co-Driver
            // ------------------------------------------------------------
            // -- Linear Slide
            // -- -- Y: High Limit
            // -- -- X: Middle Limit
            // -- -- B: Low Limit
            // -- -- A: Ground Limit
            // -- -- Axis (right_stick_y): Move up/down (restrict movement beyond High/Ground)
            // -- Claw
            // -- -- Left Bumper: Open Claw
            // -- -- Right Bumper: Close Claw
            // -- -- D-Pad Up: Reset up/down and left/right to center (.5, .5)
            // -- -- D-Pad left: Left Max Setpoint
            // -- -- D-Pad right: Right Max Setpoint
            // -- -- D-Pad down: Down Max Setpoint
            // -- -- Axis (left_stick_y): Up/Down (0-1) Movement
            // -- -- Axis (left_stick_x): Left/Right (0-1) Movement
            // -- -- Left Trigger: Left Movement
            // -- -- Right Trigger: Right Movement

            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------

            // Button Action - Cycle Drive mode
            if(gamepad1.back) {
                if(sysDrivetrain.getLabelDrivetrainMode().equals(RobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC)) {
                    sysDrivetrain.stateDrivetrainMode = StateDrivetrainMode.Robot_Centric;
                }
                else {
                    sysDrivetrain.stateDrivetrainMode = StateDrivetrainMode.Field_Centric;
                }
            }

            // Button Action - Cycle Output Power Setting
            //if(gamepad1.start)
            //    sysDrivetrain.setDrivetrainOutputPowerNext();

            // Button Action - Set Output Power Mode to Medium
            if(gamepad1.y) {
                sysDrivetrain.stateMaxDriveOutputPower = StateDriveMotorMaxOutputPower.Medium;
            }

            // Button Action - Set Output Power Mode to Low
            if(gamepad1.a) {
                sysDrivetrain.stateMaxDriveOutputPower = StateDriveMotorMaxOutputPower.Low;
            }

            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors
            inputYaw =  -(gamepad1.right_stick_x);
            inputAxial = -(gamepad1.left_stick_y);
            inputLateral = gamepad1.left_stick_x;

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
            if(sysDrivetrain.getLabelDrivetrainMode().equals(RobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC)) {
                // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
            }
            else {
                // Send gamepad input for drivetrain to driveMecanumFieldCentric method in the drivetrain system class
                sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
            }

            // Button Action - Turn robot 180 degrees (180 deg spin)
            if(gamepad1.x) {
                sysDrivetrain.driveTurnToHeading(180, 1, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED);
            }

            // ------------------------------------------------------------
            // LinearSlide
            // ------------------------------------------------------------

            // Manually control Linear Slide
            if(Math.abs(gamepad2.right_stick_y) == 1) {
                isManualSlideMode = true;

                // Check Lower Limit Switch at command/input level
                if(sysLinearSlide.getLinearSlideLimitSwitchSetting()) {
                    if(gamepad2.right_stick_y < 0) {
                        sysLinearSlide.moveLinearSlideManually(-(gamepad2.right_stick_y), RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_MED);
                    }
                }
                else {
                    sysLinearSlide.moveLinearSlideManually(-(gamepad2.right_stick_y), RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_MED);
                }

            }

            // Button Action - Set Linear Slide to High Goal
            if(gamepad2.y) {
                isManualSlideMode = false;

                // Move Linear Slide to High Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_HIGH_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH);
            }

            // Button Action - Set Linear Slide to Medium Goal
            if(gamepad2.x) {
                isManualSlideMode = false;

                // Move Linear Slide to Medium Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_MED_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED);
            }

            // Button Action - Set Linear Slide to Low Goal
            if(gamepad2.b) {
                isManualSlideMode = false;

                // Move Linear Slide to Low Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_LOW_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW);
            }

            // Button Action - Set Linear Slide to Ground Goal
            if(gamepad2.a) {
                isManualSlideMode = false;

                // Move the Claw to Center (Ground could damage claw if left out)
                sysClaw.moveClawSideToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_SIDE_START);
                sysClaw.moveClawUpDownToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_UPDOWN_START);

                // Move Linear Slide to Ground Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_GROUND_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND);
            }

            // Apply 'Break' to Linear Slide (when returning from manual override)
            if(Math.abs(gamepad2.right_stick_y) != 1 && isManualSlideMode) {
                sysLinearSlide.moveLinearSlideToTarget(sysLinearSlide.getLinearSlideCurrentPosition(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_PRIMARY), RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_HIGH);
            }

            // ------------------------------------------------------------
            // Claw
            // ------------------------------------------------------------
            // Button Action - Close the claw
            if(gamepad2.right_bumper) {

                // Close the Claw
                sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_CLOSE);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_CLOSED);
            }

            // Button Action - Open the claw
            if(gamepad2.left_bumper) {

                // Open the Claw
                sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_OPEN);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_OPEN);
            }

            // Manually control side-to-side motion
            if(Math.abs(gamepad2.left_stick_x) == 1) {
                sysClaw.moveClawSideManually(gamepad2.left_stick_x);
            }

            if(Math.abs(gamepad2.left_trigger) == 1) {
                sysClaw.moveClawSideManually(gamepad2.left_trigger);
            }

            if(Math.abs(gamepad2.right_trigger) == 1) {
                sysClaw.moveClawSideManually(-(gamepad2.right_trigger));
            }

            // Manually control up-down motion
            if(Math.abs(gamepad2.left_stick_y) == 1) {
                sysClaw.moveClawUpDownManually(gamepad2.left_stick_y);
            }

            // Button Action - move to claw up setpoint
            if(gamepad1.dpad_up) {

                // Claw Reset up-down and lef-right to center
                // reset to start position
                sysClaw.moveClawSideToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_SIDE_START);
                sysClaw.moveClawUpDownToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_UPDOWN_START);
            }


            // Button Action - move to claw left side setpoint
            if(gamepad1.dpad_left) {

                // Claw side-to-side movement (Left)
                sysClaw.moveClawSideToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_SIDE_LEFT);

            }

            // Button Action - move to claw right side setpoint
            if(gamepad1.dpad_right) {

                // Claw side-to-side movement (Right)
                sysClaw.moveClawSideToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_SIDE_RIGHT);

            }

            // Button Action - move to claw down setpoint
            if(gamepad1.dpad_down) {

                // Claw up-down movement (Down)
                sysClaw.moveClawUpDownToTarget(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_UPDOWN_DOWN);

            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Hardware Profile", RobotConstants.CommonSettings.getRobotConfigurationFileManagerNameActive());

            // ------------------------------------------------------------
            // - Gamepad telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Gamepad");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Gamepad 1 - [Y] Axial", "%4.2f", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 - [X] Lateral", "%4.2f", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 - [R] Rotation", "%4.2f", gamepad1.right_stick_x);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Gamepad 2 - Linear Slide", "%4.2f", gamepad2.right_stick_y);
            telemetry.addData("Gamepad 2 - Claw Up-Down", "%4.2f", gamepad2.left_stick_y);
            telemetry.addData("Gamepad 2 - Claw Side-to-Side", "%4.2f", gamepad2.left_stick_x);
            telemetry.addData("-", "------------------------------");

            // ------------------------------------------------------------
            // - Drivetrain telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Drivetrain");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Drivetrain Mode", sysDrivetrain.getLabelDrivetrainMode());
            telemetry.addData("Drivetrain Power", sysDrivetrain.getLabelDrivetrainOutputPower());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Power Front left/Right", "%4.2f, %4.2f"
                    , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                    , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
            telemetry.addData("Power Back  left/Right", "%4.2f, %4.2f"
                    , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                    , sysDrivetrain.getDrivetrainMotorPower(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Encoder Front left/Right", "%7d, %7d"
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
            telemetry.addData("Encoder Back  left/Right", "%7d, %7d"
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Robot Heading Raw", sysDrivetrain.getRobotHeadingRaw());
            telemetry.addData("Heading Adjustment", RobotConstants.CommonSettings.getImuTransitionAdjustment());
            telemetry.addData("Robot Heading (Adjusted)", sysDrivetrain.getRobotHeadingAdj());

            // ------------------------------------------------------------
            // - LinearSlide telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- LinearSlide");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Linear Encoder Point", sysLinearSlide.getLinearSlideCurrentPosition(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_PRIMARY));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Slide left/Right", "%4.2f, %4.2f"
                    , sysLinearSlide.getLinearSlideMotorPower(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT)
                    , sysLinearSlide.getLinearSlideMotorPower(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT));

            // ------------------------------------------------------------
            // - Claw telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- The Claw");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Claw Clamp/Side/UpDown", "%4.2f, %4.2f, %4.2f"
                    , sysClaw.getClawClampPosition()
                    , sysClaw.getClawSidePosition()
                    , sysClaw.getClawUpDownPosition());

            // ------------------------------------------------------------
            // - Lighting telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Lighting");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------

            // Temporary!! input assignment to 'pause' telemetry update(s)
            if (!gamepad1.dpad_right) {
                telemetry.update();
            }

            // Pace this loop so commands move at a reasonable speed.
            //sleep(RobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }

        // ------------------------------------------------------------
        // Closing Teleop
        // ------------------------------------------------------------
        // Update the Transition Adjustment Value for the IMU
        RobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

    }
}
