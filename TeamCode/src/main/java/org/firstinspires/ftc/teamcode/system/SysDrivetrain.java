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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.teamcode.utility.StateDrivetrainMode;
import org.firstinspires.ftc.teamcode.utility.StateDriveMotorMaxOutputPower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysDrivetrain.java

/**
 * <h2>System Class for the Robot Drivetrain</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <b>Drivetrain Wheel Type:</b> {@value RobotConstants.Drivetrain#COMMENT_DRIVE_WHEEL_TYPE}<br>
 * <b>Drivetrain Motor(s):</b> {@value RobotConstants.Drivetrain#COMMENT_DRIVE_MOTOR}<br>
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
public class SysDrivetrain {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Drivetrain Mode Enumerator
    public StateDrivetrainMode stateDrivetrainMode;

    // Define Drivetrain Output Power Enumerator
    public StateDriveMotorMaxOutputPower stateMaxDriveOutputPower;

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private List<DcMotorEx> listMotorsDrivetrain;

    // Robot Heading
    private BNO055IMU imuUnit = null;
    private double trackHeadingRobot, trackHeadingOffset, trackHeadingError;

    /**
     * <h2>Drivetrain System Constructor</h2>
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
    public SysDrivetrain(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Drivetrain System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the drivetrain system.
     * This method must be called <b>ONCE</b> when the OpMode is initialized.
     * </p>
     * <br>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     * <hr>
     */
    public void init() {
        // Set Drivetrain Enumerator default value(s)
        stateDrivetrainMode = StateDrivetrainMode.Field_Centric;
        stateMaxDriveOutputPower = StateDriveMotorMaxOutputPower.Low;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT);
        rightFrontDrive = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT);
        leftBackDrive  = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK);
        rightBackDrive = sysOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK);

        // Add Motors to an Array/List of Motors
        listMotorsDrivetrain = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

        // Clone Configuration and apply to all Motors in the list (set max RPM to 100%)
        for (DcMotorEx itemMotor : listMotorsDrivetrain) {
            MotorConfigurationType motorConfigurationType = itemMotor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            itemMotor.setMotorType(motorConfigurationType);
        }

        // Set Zero Setting to Brake Mode
        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis (roll)
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis (pitch)
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location

        // Initialize the IMU board/unit on the Rev Control Hub
        imuUnit = sysOpMode.hardwareMap.get(BNO055IMU.class, RobotConstants.Configuration.LABEL_CONTROLHUB_IMU);
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        // Set the Angle Unit to Radians
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        // Initialize the IMU unit
        imuUnit.initialize(imuParameters);

        // Reset Drive Motor Encoder(s)
        setDriveMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRobotHeadingReset();

        // Display telemetry
        sysOpMode.telemetry.addData(">", "------------------------------------");
        sysOpMode.telemetry.addData(">", " System: Drivetrain Initialized");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Drivetrain Method: driveMecanum</h2>
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
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inAxial   [Y] Driving forward and backward
     * @param inLateral [X] Strafing right and left
     * @param inYaw     [R] Rotating Clockwise and counter clockwise
     * @param inMaxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanum(double inAxial, double inLateral, double inYaw, double inMaxOutputPowerPercent) {

        double modMaintainMotorRatio;

        double inputAxial   = (inAxial * inMaxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double inputLateral = (inLateral * inMaxOutputPowerPercent) * RobotConstants.Drivetrain.MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION; // Mod to even out strafing
        double inputYaw     = (inYaw * inMaxOutputPowerPercent);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(inputAxial) + Math.abs(inputLateral) + Math.abs(inputYaw), RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MAX);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (inputAxial + inputLateral + inputYaw) / modMaintainMotorRatio;
        double rightFrontPower = (inputAxial - inputLateral - inputYaw) / modMaintainMotorRatio;
        double leftBackPower   = (inputAxial - inputLateral + inputYaw) / modMaintainMotorRatio;
        double rightBackPower  = (inputAxial + inputLateral - inputYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * <h2>Drivetrain Method: driveMecanumFieldCentric</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <br>
     * <p>
     * This is a 'Field Centric' variation of the mecanum drivetrain
     * </p>
     * <br>
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inAxial   [Y] Driving forward and backward
     * @param inLateral [X] Strafing right and left
     * @param inYaw     [R] Rotating Clockwise and counter clockwise
     * @param inMaxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanumFieldCentric(double inAxial, double inLateral, double inYaw, double inMaxOutputPowerPercent) {

        double modMaintainMotorRatio;

        double inputAxial   = (inAxial * inMaxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double inputLateral = (inLateral * inMaxOutputPowerPercent) * RobotConstants.Drivetrain.MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION; // Mod to even out strafing
        double inputYaw     = (inYaw * inMaxOutputPowerPercent);

        // Get heading value from the IMU
        double botHeading = getIMUHeading() + RobotConstants.CommonSettings.getImuTransitionAdjustment();

        // Adjust the lateral and axial movements based on heading
        double adjLateral = inputLateral * Math.cos(botHeading) - inputAxial * Math.sin(botHeading);
        double adjAxial = inputLateral * Math.sin(botHeading) + inputAxial * Math.cos(botHeading);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(inputAxial) + Math.abs(inputLateral) + Math.abs(inputYaw), RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MAX);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (adjAxial + adjLateral + inputYaw) / modMaintainMotorRatio;
        double rightFrontPower = (adjAxial - adjLateral - inputYaw) / modMaintainMotorRatio;
        double leftBackPower   = (adjAxial - adjLateral + inputYaw) / modMaintainMotorRatio;
        double rightBackPower  = (adjAxial + adjLateral - inputYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * <h2>Drivetrain Method: driveInputTimed</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * This method will run the desired inputs for the specified time.
     * </p>
     * <br>
     * @param inAxial - forward and backward movement
     * @param inLateral - Left and Right movement
     * @param inYaw - Rotational movement
     * @param inMaxOutputPowerPercent - Max output power to apply to motors
     * @param inTimeSeconds - Time in seconds to run for
     */
    public void driveInputTimed(double inAxial, double inLateral, double inYaw, double inMaxOutputPowerPercent, double inTimeSeconds) {

        // Drive time clock define and reset
        ElapsedTime driveTime = new ElapsedTime();
        driveTime.reset();

        while (sysOpMode.opModeIsActive() && (driveTime.seconds() < inTimeSeconds)) {

            // Perform timed drive until the timer completes
            driveMecanumFieldCentric(inAxial, inLateral, inYaw, inMaxOutputPowerPercent);
        }

        // Stop driving once time has elapsed
        driveMecanumFieldCentric(0, 0, 0, 0);
    }

    /**
     * <h2>Drivetrain Method: driveTurnToHeading</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Use the IMU to turn the robot to a new target heading.
     * </p>
     * <br>
     *
     * @param inTargetHeading
     * @param inMaxOutputPowerPercent
     */
    public void driveTurnToHeading(double inTargetHeading, double inMaxOutputPowerPercent) {
        // local variable for turn output power
        double robotTurnSpeed;

        // Pre-Calculate the current error
        getSteeringCorrection(inTargetHeading, RobotConstants.Drivetrain.HEADING_P_DRIVE_GAIN);

        // Loop while not at heading
        while (sysOpMode.opModeIsActive() && (Math.abs(trackHeadingError) > RobotConstants.Drivetrain.AUTO_HEADING_THRESHOLD)) {

            // Calculate the current error
            robotTurnSpeed = getSteeringCorrection(inTargetHeading, RobotConstants.Drivetrain.HEADING_P_DRIVE_GAIN);

            // Determine steering to keep heading
            robotTurnSpeed = Range.clip(robotTurnSpeed, -inMaxOutputPowerPercent, inMaxOutputPowerPercent);

            // Move Robot
            driveMecanumFieldCentric(0, 0, robotTurnSpeed, RobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_LOW);

        }

        // Stop Robot
        driveMecanumFieldCentric(0, 0, 0, 0);
    }

    /**
     *
     * @param inAxialInches
     * @param inLateralInches
     * @param inMaxOutputPowerPercent
     */
    public void driveDistance(double inAxialInches, double inLateralInches, double inMaxOutputPowerPercent) {

        // ---------------------
        // WORK IN PROGRESS!!
        // ---------------------

        // Set target position
        leftFrontDrive.setTargetPosition(100);
        leftBackDrive.setTargetPosition(-100);
        rightFrontDrive.setTargetPosition(100);
        rightBackDrive.setTargetPosition(-100);

        // Turn on RUN_TO_POSITION (moved encoders to that position)
        setDriveMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Make sure all motors are stopped after completed path
        setDriveMotorPower(0, 0, 0, 0);

        // Turn off RUN_TO_POSITION
        setDriveMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * <h2>Drivetrain Method: getIMUHeading</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the output heading value from the IMU.
     * </p>
     * @return double - Output heading value from the IMU
     * <br>
     */
    public double getIMUHeading() {
        // Variable for output heading value
        double outIMUHeadingValue = 0;

        // Get heading value from the IMU
        // Read inverse IMU heading, as the IMU heading is CW positive
        outIMUHeadingValue = -(imuUnit.getAngularOrientation().firstAngle);

        // Should the IMU heading be inversed? Does it matter?
        // Will need to view the heading readout on the driver hub

        //imuUnit.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle

        return outIMUHeadingValue;
    }

    /**
     *
     * @param inTargetHeader
     * @param inProportionalGain
     * @return
     */
    public double getSteeringCorrection(double inTargetHeader, double inProportionalGain) {

        // Determine the robot heading current PID error
        trackHeadingRobot = getIMUHeading() - trackHeadingOffset;

        // Determine the heading current error
        trackHeadingError = inTargetHeader - trackHeadingRobot;

        // Normalize the error to be within +/- 180 degrees
        while (trackHeadingError > 180) trackHeadingError -= 360;
        while (trackHeadingError <= -180) trackHeadingError += 360;

        return Range.clip(trackHeadingError * inProportionalGain, -1, 1);
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
    public double getDrivetrainMotorPower(String inMotorLabel) {
        // Variable for output Power value for drivetrain motor(s)
        double outPowerValue = 0;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Drivetrain Motor - Left Front
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT:
                outPowerValue = leftFrontDrive.getPower();
                break;
            // Drivetrain Motor - Left Back
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK:
                outPowerValue = leftBackDrive.getPower();
                break;
            // Drivetrain Motor - Right Front
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT:
                outPowerValue = rightFrontDrive.getPower();
                break;
            // Drivetrain Motor - Right Back
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK:
                outPowerValue = rightBackDrive.getPower();
                break;
            // Default - No match
            default:
                outPowerValue = 0;
        }

        // Return value
        return outPowerValue;
    }

    /**
     *
     * @param inMotorLabel
     * @return
     */
    public int getDrivetrainMotorEncoderPosition(String inMotorLabel) {
        // Variable for output Encoder value for drivetrain motor(s)
        int outEncoderValue = 0;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Drivetrain Motor - Left Front
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT:
                outEncoderValue = leftFrontDrive.getCurrentPosition();
                break;
            // Drivetrain Motor - Left Back
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK:
                outEncoderValue = leftBackDrive.getCurrentPosition();
                break;
            // Drivetrain Motor - Right Front
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT:
                outEncoderValue = rightFrontDrive.getCurrentPosition();
                break;
            // Drivetrain Motor - Right Back
            case RobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK:
                outEncoderValue = rightBackDrive.getCurrentPosition();
                break;
            // Default - No match
            default:
                outEncoderValue = 0;
        }

        // Return value
        return outEncoderValue;
    }

    /**
     * <h2>Drivetrain Method: getLabelDrivetrainMode</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the value of the current Drivetrain mode from List Iterator.
     * </p>
     * @return String - Output the current mode value
     * <br>
     */
    public String getLabelDrivetrainMode() {
        return stateDrivetrainMode.getLabel();
    }

    /**
     * <h2>Drivetrain Method: getLabelDrivetrainOutputPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the label of the current Drivetrain output power setting from List Iterator.
     * </p>
     * @return String - Output the current output power setting label
     * <br>
     */
    public String getLabelDrivetrainOutputPower() {
        return stateMaxDriveOutputPower.getLabel();
    }

    /**
     * <h2>Drivetrain Method: getDrivetrainOutputPowerCurrent</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the value of the current Drivetrain output power setting from List Iterator.
     * </p>
     * @return double - Output the current output power setting value
     * <br>
     */
    public double getValueDrivetrainOutputPower() { return stateMaxDriveOutputPower.getValue(); }

    /**
     * <h2>Drivetrain Method: setDrivetrainModeNext</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the next value of the Drivetrain mode from List Iterator.
     * </p>
     * <br>
     */
    public void setDrivetrainModeNext() {

        // Cycle drivetrain mode
        stateDrivetrainMode =  stateDrivetrainMode.nextState();
        sysOpMode.sleep(RobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
    }

    /**
     * <h2>Drivetrain Method: setDrivetrainOutputPowerNext</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the next value of the Drivetrain output power setting from List Iterator.
     * </p>
     * <br>
     */
    public void setDrivetrainOutputPowerNext() {

        // Cycle drivetrain output power
        stateMaxDriveOutputPower = stateMaxDriveOutputPower.nextState();
        sysOpMode.sleep(RobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
    }

    /**
     * <h2>Drivetrain Method: setDriveMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     * </p>
     * @param inleftFrontPower  Power to Left Front Wheel
     * @param inrightFrontPower Power to Right Front Wheel
     * @param inleftBackPower   Power to Left Back Wheel
     * @param inrightBackPower  Power to Right Back Wheel
     *
     * <br>
     */
    private void setDriveMotorPower(double inleftFrontPower, double inrightFrontPower, double inleftBackPower, double inrightBackPower) {

        // Send calculated power to wheels
        leftFrontDrive.setPower(inleftFrontPower);
        rightFrontDrive.setPower(inrightFrontPower);
        leftBackDrive.setPower(inleftBackPower);
        rightBackDrive.setPower(inrightBackPower);
    }

    /**
     * <h2>Drivetrain Method: setDriveMotorRunMode</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the run mode for each drive motor.
     * </p>
     *
     * @param inRunMode DcMotor.RunMode - Set the run mode for each drive motor
     *
     */
    public void setDriveMotorRunMode(DcMotor.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorsDrivetrain) {
            itemMotor.setMode(inRunMode);
        }
    }

    /**
     * <h2>Drivetrain Method: setDriveMotorZeroPowerBehavior</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the 'Zero Behavior' for each drive motor. Brake/Coast
     * </p>
     *
     * @param inZeroPowerBehavior DcMotor.ZeroPowerBehavior - Set the Zero Power behavior for each drive motor(s)
     *
     */
    public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsDrivetrain) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    /**
     *
     */
    public void setRobotHeadingReset() {

        // Set the Heading Offset to the IMU raw heading
        trackHeadingOffset = getIMUHeading();

        // Reset the Robot Heading to Zero
        trackHeadingRobot = 0;
    }

}
