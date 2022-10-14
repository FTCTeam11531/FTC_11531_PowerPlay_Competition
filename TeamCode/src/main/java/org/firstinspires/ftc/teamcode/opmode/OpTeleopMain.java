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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.SysDrivetrain;
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
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Drivetrain System
    SysDrivetrain sysDrivetrain = new SysDrivetrain(this);

    // -- Claw System

    // -- LinearSlide System

    // -- Lighting System

    // -- Vision System


    // ------------------------------------------------------------
    // Command Object(s)
    // ------------------------------------------------------------


    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        sysDrivetrain.init();

        // ------------------------------------------------------------
        // Inputs for: Drivetrain
        // ------------------------------------------------------------
        double inputAxial = 0;
        double inputLateral = 0;
        double inputYaw = 0;

        int driveModeIndex = 0;

        // Set Drivetrain Mode Value
        sysDrivetrain.setDrivetrainModeNext();
        sysDrivetrain.setDrivetrainOutputPowerNext();

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
            inputYaw     =  gamepad1.right_stick_x;

            // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
            if(sysDrivetrain.getDrivetrainModeCurrent().equals(RobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC)) {
                inputAxial = -(gamepad1.left_stick_x);
                inputLateral = -(gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
                sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, 1);
            }
            else {
                inputAxial = -(gamepad1.left_stick_y);
                inputLateral = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, 1);
            }

            // ------------------------------------------------------------
            // Claw
            // ------------------------------------------------------------



            // ------------------------------------------------------------
            // LinearSlide
            // ------------------------------------------------------------


            // ------------------------------------------------------------
            // Lighting
            // ------------------------------------------------------------


            // ------------------------------------------------------------
            // Vision
            // ------------------------------------------------------------


            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // ------------------------------------------------------------
            // - Drivetrain telemetry
            // ------------------------------------------------------------
            //String dataDrivetrainMode = sysDrivetrain.getDrivetrainModeCurrent();
            //double dataDrivetrainOutputPower = sysDrivetrain.getDrivetrainOutputPowerCurrent();

            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Drivetrain");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Drivetrain Mode: ", sysDrivetrain.getDrivetrainModeCurrent());
            telemetry.addData("Drivetrain Power: ", "%4.2f", sysDrivetrain.getDrivetrainOutputPowerCurrent());
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
            // - Claw telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- The Claw");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "<No Data Available>");

            // ------------------------------------------------------------
            // - LinearSlide telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- LinearSlide");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "<No Data Available>");

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
            telemetry.addData("-", "<No Data Available>");

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            //sleep(50);
        }
    }
}
