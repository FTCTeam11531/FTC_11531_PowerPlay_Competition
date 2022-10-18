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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.utility.StateDriveMotorMaxOutputPower;
import org.firstinspires.ftc.teamcode.utility.StateDrivetrainMode;

import java.util.Arrays;
import java.util.List;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysLinearSlide.java

/**
 * <h2>System Class for the Linear Slide</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <b>Linear Slide Motor(s):</b> {@value RobotConstants.LinearSlide#COMMENT_LINEARSLIDE_MOTOR}<br>
 * <hr>
 * <p>
 * This system defines <b>ALL</b> configures all attributes, configurations, and methods for the Robot
 * drivetrain. The hardware configuration is for a mecanum drivetrain.
 * </p>
 * <br>
 * <p>
 * <b>This drivetrain system includes methods for:</b> standard mecanum drive,  field centric mecanum
 * drive, encoder distance drive, and encoder timed drive.
 * </p>
 * <hr>
 *
 */
public class SysLinearSlide {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotorEx leftLinearSlideMotor, rightLinearSlideMotor;
    private List<DcMotorEx> listMotorsLinearSlide;

    /**
     * <h2>Linear Slide System Constructor</h2>
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
    public SysLinearSlide(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Linear Slide System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the linear slide system.
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
        leftLinearSlideMotor = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT);
        rightLinearSlideMotor = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT);

        // Add Motors to an Array/List of Motors
        listMotorsLinearSlide = Arrays.asList(leftLinearSlideMotor, rightLinearSlideMotor);

        // Set Zero Setting to Brake Mode
        setLinearSlideMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Linear Slide Motor Direction
        leftLinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: LinearSlide Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Drivetrain Method: configInitDriveMotorEncoders</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <p>
     * This is a standard mecanum drivetrain or a 'Robot Centric' drivetrain
     * </p>
     * <br>
     *
     * @return void
     * <br>
     */
    public void configInitLinearSlideMotorEncoders() {

        // Reset Drive Motor Encoders
        configResetLinearSlideMotorEncoders();

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        setLinearSlideMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void configResetLinearSlideMotorEncoders() {

        // Reset the Drive Motor Encoders
        setLinearSlideMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * <h2>Linear Slide Method: moveSlideToEncoderSetPoint</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the Linear Slide to the Encoder Set Point
     * </p>
     * @param inEncoderSetPoint int - Encoder Set Point to move to
     *
     * @return void
     * <br>
     */
    public void moveSlideToEncoderSetPoint(int inEncoderSetPoint, double inOutputPowerPercent) {

        int inputEncoderSetPoint = inEncoderSetPoint;

        leftLinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlideMotor.setTargetPosition(inEncoderSetPoint);
        leftLinearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLinearSlideMotor.setPower(inOutputPowerPercent);

        //rightLinearSlideMotor.setTargetPosition(inputEncoderSetPoint);
    }

    /**
     * <h2>Drivetrain Method: getDrivetrainMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the output power value for a drivetrain motor.
     * </p>
     * @param inMotorLabel  The Label Name of the motor to get the power value from
     *
     * @return double - Output power value of the motor
     * <br>
     */
    public double getLinearSlideMotorPower(String inMotorLabel) {
        // Variable for output Power value for drivetrain motor(s)
        double outPowerValue = 0;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Drivetrain Motor - Left Front
            case RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT:
                outPowerValue = leftLinearSlideMotor.getPower();
                break;
            // Drivetrain Motor - Left Back
            case RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT:
                outPowerValue = rightLinearSlideMotor.getPower();
                break;
            // Default - No match
            default:
                outPowerValue = 0;
        }

        // Return value
        return outPowerValue;
    }

    /**
     * <h2>Linear Slide Method: setLinearSlideMotorRunMode</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the run mode for each motor.
     * </p>
     *
     * @param inRunMode
     *
     * @return void (Nothing)
     */
    public void setLinearSlideMotorRunMode(DcMotor.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorsLinearSlide) {
            itemMotor.setMode(inRunMode);
        }
    }

    /**
     * <h2>Linear Slide Method: setLinearSlideMotorZeroPowerBehavior</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the 'Zero Behavior' for each motor. Brake/Coast/Etc
     * </p>
     *
     * @param inZeroPowerBehavior
     *
     * @return void (Nothing)
     */
    public void setLinearSlideMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsLinearSlide) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

}
