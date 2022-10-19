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
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.system.SysClaw;
import org.firstinspires.ftc.teamcode.system.SysDrivetrain;
import org.firstinspires.ftc.teamcode.system.SysLighting;
import org.firstinspires.ftc.teamcode.system.SysLinearSlide;
import org.firstinspires.ftc.teamcode.system.SysVision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

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
    // Robot Configuration
    // ------------------------------------------------------------
    RobotConfigFileManager robotConfigFileManager;
    String robotConfigName;

    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- LinearSlide System
    SysLinearSlide sysLinearSlide = new SysLinearSlide(this);

    // -- Claw System
    SysClaw sysClaw = new SysClaw(this);

    // -- Lighting System
    SysLighting sysLighting = new SysLighting(this);

    // -- Vision System
    SysVision sysVision = new SysVision(this);
    List<Recognition> listVisionRecognitions = null;
    Recognition recognitionTargetZone = null;

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------
        // Get Hardware Configuration Profile Name
        // ------------------------------------------------------------
        robotConfigFileManager = new RobotConfigFileManager((Activity) hardwareMap.appContext);
        robotConfigName = robotConfigFileManager.getActiveConfig().getName();

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        sysDrivetrain.init();
        sysLinearSlide.init();
        sysClaw.init();
        sysLighting.init();
        sysVision.init(robotConfigName);

        // ------------------------------------------------------------
        // Inputs for: Drivetrain
        // ------------------------------------------------------------
        double inputAxial = 0;
        double inputLateral = 0;
        double inputYaw = 0;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // ------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        // ------------------------------------------------------------
        waitForStart();
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) return;

        // ------------------------------------------------------------
        // Command Loop: run until the end of the match (driver presses STOP)
        // ------------------------------------------------------------
        while (opModeIsActive()) {
            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Select / Change Drive mode
            if(gamepad1.back)
                sysDrivetrain.setDrivetrainModeNext();

            if(gamepad1.start)
                sysDrivetrain.setDrivetrainOutputPowerNext();

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            inputYaw =  gamepad1.right_stick_x;

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
            if(sysDrivetrain.getLabelDrivetrainMode().equals(RobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC)) {
                // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
                inputAxial = -(gamepad1.left_stick_y);
                inputLateral = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
            }
            else {
                // Send gamepad input for drivetrain to driveMecanumFieldCentric method in the drivetrain system class
                inputAxial = -(gamepad1.left_stick_x);
                inputLateral = -(gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
                sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
            }

            // ------------------------------------------------------------
            // LinearSlide
            // ------------------------------------------------------------
            if(gamepad1.y) {

                // Move Linear Slide to High Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_HIGH_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_LOW);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH);
            }

            if(gamepad1.x) {

                // Move Linear Slide to Medium Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_MED_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_LOW);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED);
            }

            if(gamepad1.b) {

                // Move Linear Slide to Low Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_LOW_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_LOW);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW);
            }

            if(gamepad1.a) {

                // Move Linear Slide to Ground Goal
                sysLinearSlide.moveLinearSlideToTarget(RobotConstants.LinearSlide.ENCODER_SET_POINT_GROUND_GOAL, RobotConstants.LinearSlide.MOTOR_OUTPUT_POWER_LOW);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND);
            }

            // ------------------------------------------------------------
            // Claw
            // ------------------------------------------------------------
            if(gamepad1.right_bumper) {

                // Close the Claw
                sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_CLOSE);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_CLOSED);
            }

            if(gamepad1.left_bumper) {

                // Open the Claw
                sysClaw.setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_OPEN);
                sysLighting.setLightPattern(RobotConstants.Lighting.LIGHT_PATTERN_CLAW_CLAMP_OPEN);
            }

            if(gamepad1.dpad_left) {

                // Claw side-to-side movement (Left)


            }

            //if(gamepad1.dpad_right) {

                // Claw side-to-side movement (Right)

            //}

            if(gamepad1.dpad_up) {

                // Claw up-down movement (Up)
                sysLighting.setLightPattern(sysLighting.getLightPatternNext());

            }

            if(gamepad1.dpad_down) {

                // Claw up-down movement (Down)
                sysLighting.setLightPattern(sysLighting.getLightPatternPrevious());

            }

            // ------------------------------------------------------------
            // Vision
            // ------------------------------------------------------------
            // Get Vision Recognitions
            listVisionRecognitions = sysVision.getVisonRecognitions();

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Hardware Profile: ", robotConfigName);

            // ------------------------------------------------------------
            // - Drivetrain telemetry
            // ------------------------------------------------------------
            //String dataDrivetrainMode = sysDrivetrain.getDrivetrainModeCurrent();
            //double dataDrivetrainOutputPower = sysDrivetrain.getDrivetrainOutputPowerCurrent();

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
            // - LinearSlide telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- LinearSlide");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Linear Encoder Point: ", sysDrivetrain.getLabelDrivetrainOutputPower());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Slide left/Right: ", "%4.2f, %4.2f"
                    , sysLinearSlide.getLinearSlideMotorPower(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT)
                    , sysLinearSlide.getLinearSlideMotorPower(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT));

            // ------------------------------------------------------------
            // - Claw telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- The Claw");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Claw Clamp/Side/UpDown: ", "%4.2f, %4.2f, %4.2f"
                    , sysClaw.getClawClampPosition()
                    , sysClaw.getClawSidePosition()
                    , sysClaw.getClawUpDownPosition());

            // ------------------------------------------------------------
            // - Lighting telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Lighting");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "<No Data Available>");

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("TensorFlow Model: ", RobotConstants.Vision.TENSORFLOW_MODEL_SELECTION);
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
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", objRecognition.getLabel(), objRecognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", objRow, objColumn);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", objWidth, objHeight);
                }

            }
            else {
                telemetry.addData("-", "<No Data Available>");
            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------

            // Temporary!! input assignment to 'pause' telemetry update(s)
            if (!gamepad1.dpad_right) {
                telemetry.update();
            }

            // Pace this loop so hands move at a reasonable speed.
            //sleep(RobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }
    }
}
