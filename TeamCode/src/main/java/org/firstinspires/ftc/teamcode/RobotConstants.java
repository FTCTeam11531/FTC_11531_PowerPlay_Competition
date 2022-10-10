package org.firstinspires.ftc.teamcode;

// Program Class created from new Java Class
// Named in TeamCode as: RobotConstants.java

/**
 *  <h2>Robot Constant Values</h2>
 * <hr>
 * <b>Author:</b> {@value org.firstinspires.ftc.teamcode.RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value org.firstinspires.ftc.teamcode.RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Defines <b>ALL</b> robot constant settings and properties to be used throughout
 * the robot system(s) and command(s).
 * </p>
 * <br>
 */

public class RobotConstants {

    public static final class About {
        public static final String COMMENT_AUTHOR_NAME = "FTC Team 11531 - Green Team";
        public static final String COMMENT_SEASON_PERIOD = "2022 / 2023";
    }

    public static final class Configuration {
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT = "left_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_BACK = "left_back_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT = "right_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK = "right_back_drive";

        public static final String LABEL_CONTROLHUB_IMU = "imu_ch";
        public static final String LABEL_EXPANSIONHUB_IMU = "imu_eh";

    }

    public static final class Drivetrain {
        public enum DRIVE_MODE_TELEOP {FIELD_CENTRIC, ROBOT_CENTRIC};

        public static final double MOD_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOD_OUTPUT_POWER_MAX = 1;
        public static final double MOD_OUTPUT_POWER_HIGH = 1;
        public static final double MOD_OUTPUT_POWER_MED = 0.75;
        public static final double MOD_OUTPUT_POWER_LOW = 0.50;
        public static final double MOD_OUTPUT_POWER_SNAIL = 0.20;

    }
}
