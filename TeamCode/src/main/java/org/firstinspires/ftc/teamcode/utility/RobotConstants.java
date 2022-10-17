package org.firstinspires.ftc.teamcode.utility;

// Program Class created from new Java Class
// Named in TeamCode as: RobotConstants.java

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 *  <h2>Robot Constant Values</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Defines <b>ALL</b> robot constant settings and properties to be used throughout
 * the robot system(s) and command(s).
 * </p>
 * <br>
 */
public class RobotConstants {

    // General 'Global' Constants
    // Conversions
    // - - Divide Millimeters by Conversion to get Inches
    // - - Multiply Inches by Conversion to get mm
    public static final double CONVERSION_MM_TO_INCH = 25.4;

    public static final class About {
        // Document Comments and any other 'About the Program' reference
        public static final String COMMENT_AUTHOR_NAME = "FTC Team 11531 - Green Team";
        public static final String COMMENT_SEASON_PERIOD = "2022 / 2023";
    }

    public static final class Configuration {
        // Hardware Configuration from Control Hub and/or Expansion Hub
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT = "left_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_BACK = "left_back_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT = "right_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK = "right_back_drive";

        public static final String LABEL_CONTROLHUB_IMU = "imu_ch";
        public static final String LABEL_EXPANSIONHUB_IMU = "imu_eh";

        public static final String LABEL_MOTOR_LINEAR_SLIDE_LEFT = "linear_slide_left";
        public static final String LABEL_MOTOR_LINEAR_SLIDE_RIGHT = "linear_slide_right";

        public static final String LABEL_SERVO_CLAW_ROTATE = "claw_rotate";
        public static final String LABEL_SERVO_CLAW_CLAMP = "claw_clamp";

        public static final String LABEL_CONTROLLER_LIGHTING = "cntl_lighting";

    }

    public static final class Drivetrain {
        // Encoder, Motor, and Drivetrain Settings based on the following [comments]:
        // Drivetrain specific Constants
        public static final String COMMENT_DRIVE_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVE_WHEEL_TYPE = "Mecanum";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "FIELD_CENTRIC";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "ROBOT_CENTRIC";

        // List Item - Drivetrain Output Power Levels
        //public static final String LIST_OUTPUT_POWER_HIGH = "High";
        //public static final String LIST_OUTPUT_POWER_MED = "Med";
        //public static final String LIST_OUTPUT_POWER_LOW = "Low";
        //public static final String LIST_OUTPUT_POWER_SNAIL = "Snail";

        // Physical Robot Settings
        public static final double DRIVETRAIN_TRACK_WIDTH_INCHES = 1; // <<--- Need to measure!
        public static final double DRIVE_WHEEL_DIAMETER_MILLIMETER = 96; // <<--- Need to confirm!

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = DRIVE_WHEEL_DIAMETER_MILLIMETER / CONVERSION_MM_TO_INCH;
        public static final double DRIVE_WHEEL_RADIUS_MILLIMETER = DRIVE_WHEEL_DIAMETER_MILLIMETER / 2;
        public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_RADIUS_MILLIMETER / CONVERSION_MM_TO_INCH;

        public static final double DRIVE_EXTERNAL_GEAR_RATIO = 1.0; // 1 = No External Gearing

        // Motor/Encoder Configuration Settings
        public static final double DRIVE_MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double DRIVE_ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        public static final double DRIVE_ENCODER_TICKS_PER_INCH = (DRIVE_ENCODER_TICKS_PER_REV * DRIVE_EXTERNAL_GEAR_RATIO) /
                                                                    (DRIVE_WHEEL_DIAMETER_INCHES * Math.PI);

        // Motor Encoder PIDF Settings
        public static final double DRIVE_MOTOR_VELOCITY_P = 0;
        public static final double DRIVE_MOTOR_VELOCITY_I = 0;
        public static final double DRIVE_MOTOR_VELOCITY_D = 0;
        public static final PIDFCoefficients DRIVE_MOTOR_VELOCITY_PIDF = new PIDFCoefficients(
                                                    DRIVE_MOTOR_VELOCITY_P,
                                                    DRIVE_MOTOR_VELOCITY_I,
                                                    DRIVE_MOTOR_VELOCITY_D,
                                                    getMotorVelocityF(DRIVE_MOTOR_MAX_RPM / 60 * DRIVE_ENCODER_TICKS_PER_REV));

        // Motor Encoder Feed Forward Configuration(s)
        public static final double FEED_FORWARD_KV = 1.0 / rpmToVelocity(DRIVE_MOTOR_MAX_RPM); // volts * seconds / distance
        public static final double FEED_FORWARD_KA = 0; // volts * seconds^2 / distance
        public static final double FEED_FORWARD_KS = 0; // volts

        // Motor Limit 'Max' Settings
        public static final double LIMIT_MAX_VELOCITY = 30;
        public static final double LIMIT_MAX_ACCELERATION = 30;
        public static final double LIMIT_MAX_ANGLE_VELOCITY = Math.toRadians(60);
        public static final double LIMIT_MAX_ANGLE_ACCELERATION = Math.toRadians(60);


        // Motor Output Setting(s)
        public static final double MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOTOR_OUTPUT_POWER_MAX = 1;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75;
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.50;
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.20;

        /**
         * <h2>Convert: Encoder Ticks to Inches</h2>
         * <hr>
         * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert Encoder Ticks/Pulses/Cycles to Inches
         * </p>
         * <br>
         *
         * @param inTicks
         *
         * @return double (Inches)
         */
        public static double encoderTicksToInches(double inTicks) {
            double outInches = 0;

            outInches = DRIVE_WHEEL_RADIUS_INCHES * 2 * Math.PI * DRIVE_EXTERNAL_GEAR_RATIO * inTicks / DRIVE_ENCODER_TICKS_PER_REV;

            return outInches;
        }

        /**
         * <h2>Convert: RPM to Velocity</h2>
         * <hr>
         * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert RPM to Velocity
         * </p>
         * <br>
         *
         * @param inRpm
         *
         * @return double (Velocity)
         */
        public static double rpmToVelocity(double inRpm) {
            double outVelocity = 0;

            outVelocity = inRpm * DRIVE_EXTERNAL_GEAR_RATIO * Math.PI * DRIVE_WHEEL_RADIUS_INCHES / 60.0;

            return outVelocity;
        }

        /**
         * <h2>Get Motor Velocity F Value for PIDF</h2>
         * <hr>
         * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Calculating PIDF Values from Maximum Velocity
         * Once you have your maximum velocity (maxV), you can calculate the velocity PIDF values. Your F value is calculated like this: F = 32767maxV. So if your max velocity is 2600 ticks per second (reasonable for a mechanism that uses the HD Hex Motor), then your F value should be about 12.6.
         * </p>
         * <br>
         * Source: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
         *
         * @param inTicksPerSecond
         *
         * @return double (Velocity F - for PIDF)
         */
        public static double getMotorVelocityF(double inTicksPerSecond) {
            double outVelocityF = 0;

            outVelocityF = 32767 / inTicksPerSecond;

            return outVelocityF;
        }

    }
}
