package org.firstinspires.ftc.teamcode.utility;

// Program Class created from new Java Class
// Named in TeamCode as: RobotConstants.java

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

    }

    public static final class Drivetrain {
        // Encoder, Motor, and Drivetrain Settings based on the following [comments]:
        // Drivetrain specific Constants
        public static final String COMMENT_DRIVETRAIN_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVETRAIN_WHEEL_TYPE = "Mecanum";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "FIELD_CENTRIC";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "ROBOT_CENTRIC";

        // List Item - Drivetrain Output Power Levels
        public static final String LIST_OUTPUT_POWER_HIGH = "High";
        public static final String LIST_OUTPUT_POWER_MED = "Med";
        public static final String LIST_OUTPUT_POWER_LOW = "Low";
        public static final String LIST_OUTPUT_POWER_SNAIL = "Snail";

        // Motor Output Setting(s)
        public static final double MOD_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOD_OUTPUT_POWER_MAX = 1;

        public static final double MOD_OUTPUT_POWER_HIGH = 1;
        public static final double MOD_OUTPUT_POWER_MED = 0.75;
        public static final double MOD_OUTPUT_POWER_LOW = 0.50;
        public static final double MOD_OUTPUT_POWER_SNAIL = 0.20;

        // Encoder Configuration Settings
        public static final double DRIVE_WHEEL_DIAMETER_MILLIMETER = 96;
        public static final double DRIVE_WHEEL_DIAMETER_INCHES = DRIVE_WHEEL_DIAMETER_MILLIMETER / CONVERSION_MM_TO_INCH;
        public static final double DRIVE_GEAR_REDUCTION = 1.0; // 1 = No External Gearing
        public static final double DRIVE_ENCODER_PULSES_PER_REV = 537.7;
        public static final double DRIVE_ENCODER_PULSES_PER_INCH = (DRIVE_ENCODER_PULSES_PER_REV * DRIVE_GEAR_REDUCTION) /
                                                                    (DRIVE_WHEEL_DIAMETER_INCHES * Math.PI);

    }
}
