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
 * linear slide.
 * </p>
 * <hr>
 *
 */
public class SysLinearSlide {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private DcMotorEx leftLinearSlideMotor, rightLinearSlideMotor, singleLinearSlideMotor;
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
        if(RobotConstants.LinearSlide.IS_USING_SINGLE_MOTOR) {
            singleLinearSlideMotor = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_PRIMARY);

            // Add Motors to an Array/List of Motors
            listMotorsLinearSlide = Arrays.asList(singleLinearSlideMotor);
        }
        else {
            leftLinearSlideMotor = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT);
            rightLinearSlideMotor = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT);

            // Add Motors to an Array/List of Motors
            listMotorsLinearSlide = Arrays.asList(leftLinearSlideMotor, rightLinearSlideMotor);
        }

        // Clone Configuration and apply to all Motors in the list (set max RPM to 100%)
        for (DcMotorEx itemMotor : listMotorsLinearSlide) {
            MotorConfigurationType motorConfigurationType = itemMotor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            itemMotor.setMotorType(motorConfigurationType);
        }

        if(RobotConstants.LinearSlide.IS_USING_SINGLE_MOTOR) {
            // Linear Slide Motor Direction
            singleLinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            // Linear Slide Motor Direction
            leftLinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            rightLinearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Set Zero Setting to Brake Mode
        setLinearSlideMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Linear Slide Motor Encoder(s)
        setLinearSlideMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: LinearSlide Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Linear Slide Method: moveLinearSlideToTarget</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the Linear Slide to the Target Set Point
     * </p>
     * @param inTargetSetPoint int - Target set point to move to
     * @param inOutputPowerPercent double - Output power percentage for motor
     *
     * @return void
     * <br>
     */
    public void moveLinearSlideToTarget(int inTargetSetPoint, double inOutputPowerPercent) {

        // Configure Motor Target Set Point and Set motor as Run to Position
        setLinearSlideTargetPosition(inTargetSetPoint);
        setLinearSlideMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Move linear slide within min/max limits
        moveLinearSlideWithinLimits(inOutputPowerPercent);
    }

    /**
     * <h2>Linear Slide Method: moveLinearSlideWithinLimits</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Move linear slide within min/max limits
     * </p>
     * @param inOutputPowerPercent double - Output power percentage for motor
     */
    public void moveLinearSlideWithinLimits(double inOutputPowerPercent) {

        double calcLinearSlideMovement = getLinearSlideCurrentPosition(RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_PRIMARY) + inOutputPowerPercent;

        // Allow motor output within min/max limits
        if((calcLinearSlideMovement < RobotConstants.LinearSlide.ENCODER_SET_POINT_LIMIT_MAX)
            && (calcLinearSlideMovement > RobotConstants.LinearSlide.ENCODER_SET_POINT_LIMIT_MIN)) {
            setLinearSlideMotorPower(inOutputPowerPercent);
        }
        else {
            setLinearSlideMotorPower(0);
        }

    }

    /**
     * <h2>Drivetrain Method: getLinearSlideCurrentPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the encoder position value for a linear slide motor.
     * </p>
     * @param inMotorLabel  The Label Name of the motor to get the power value from
     *
     * @return int - Encoder position value of the motor
     * <br>
     */
    public int getLinearSlideCurrentPosition(String inMotorLabel) {
        // Variable for output Power value for drivetrain motor(s)
        int outEncoderPosition = 0;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Linear Slide Motor - Left Front
            case RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT:
                outEncoderPosition = leftLinearSlideMotor.getCurrentPosition();
                break;
            // Linear Slide Motor - Left Back
            case RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_RIGHT:
                outEncoderPosition = rightLinearSlideMotor.getCurrentPosition();
                break;
            // Default - No match
            default:
                outEncoderPosition = 0;
        }

        // Return value
        return outEncoderPosition;
    }

    /**
     * <h2>Drivetrain Method: getLinearSlideMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the output power value for a linear slide motor.
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
            // Linear Slide Motor - Left Front
            case RobotConstants.Configuration.LABEL_MOTOR_LINEAR_SLIDE_LEFT:
                outPowerValue = leftLinearSlideMotor.getPower();
                break;
            // Linear Slide Motor - Left Back
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
     * <h2>Linear Slide Method: setLinearSlideTargetPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the target position for each motor.
     * </p>
     *
     * @param inTargetPosition
     */
    public void setLinearSlideTargetPosition(int inTargetPosition) {
        for (DcMotorEx itemMotor: listMotorsLinearSlide) {
            itemMotor.setTargetPosition(inTargetPosition);
        }
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

    /**
     * <h2>Linear Slide Method: setLinearSlideMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the output power for each motor.
     * </p>
     *
     * @param inOutputPower
     *
     * @return void (Nothing)
     */
    public void setLinearSlideMotorPower(double inOutputPower) {
        for (DcMotorEx itemMotor : listMotorsLinearSlide) {
            itemMotor.setPower(inOutputPower);
        }
    }

}
