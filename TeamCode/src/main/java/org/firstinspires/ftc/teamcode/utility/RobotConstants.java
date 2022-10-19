package org.firstinspires.ftc.teamcode.utility;

// Program Class created from new Java Class
// Named in TeamCode as: RobotConstants.java

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

        public static final String LABEL_SERVO_CLAW_ROTATE_SIDE = "claw_rotate_side";
        public static final String LABEL_SERVO_CLAW_ROTATE_UPDOWN = "claw_rotate_updown";
        public static final String LABEL_SERVO_CLAW_CLAMP = "claw_clamp";

        public static final String LABEL_CONTROLLER_LIGHTING = "lighting_control";

        public static final String LABEL_CAMERA_VISION_TRACKING = "camera_vision_track";

    }

    public static final class CommonSettings {
        public static final int SLEEP_TIMER_MILLISECONDS_DEFAULT = 50;
    }

    public static final class Drivetrain {
        // Hardware Settings based on the following [comments]:
        // Drivetrain specific Constants
        public static final String COMMENT_DRIVE_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVE_WHEEL_TYPE = "Mecanum";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "Field Centric";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "Robot Centric";

        // List Item - Drivetrain Output Power Levels
        //public static final String LIST_OUTPUT_POWER_HIGH = "High";
        //public static final String LIST_OUTPUT_POWER_MED = "Med";
        //public static final String LIST_OUTPUT_POWER_LOW = "Low";
        //public static final String LIST_OUTPUT_POWER_SNAIL = "Snail";

        // Physical Robot Settings
        public static final double DRIVETRAIN_TRACK_WIDTH_INCHES = 10; // <<--- Need to measure!
        public static final double WHEEL_DIAMETER_MILLIMETER = 96; // <<--- Need to confirm!

        public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MILLIMETER / CONVERSION_MM_TO_INCH;
        public static final double WHEEL_RADIUS_MILLIMETER = WHEEL_DIAMETER_MILLIMETER / 2;
        public static final double WHEEL_RADIUS_INCHES = WHEEL_RADIUS_MILLIMETER / CONVERSION_MM_TO_INCH;

        public static final double EXTERNAL_GEAR_RATIO = 1.0; // 1 = No External Gearing

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        public static final double ENCODER_TICKS_PER_INCH = (ENCODER_TICKS_PER_REV * EXTERNAL_GEAR_RATIO) /
                                                                    (WHEEL_DIAMETER_INCHES * Math.PI);

        // Motor Encoder PIDF Settings
        public static final double MOTOR_VELOCITY_P = 0;
        public static final double MOTOR_VELOCITY_I = 0;
        public static final double MOTOR_VELOCITY_D = 0;
        public static final PIDFCoefficients DRIVE_MOTOR_VELOCITY_PIDF = new PIDFCoefficients(
                                                    MOTOR_VELOCITY_P,
                                                    MOTOR_VELOCITY_I,
                                                    MOTOR_VELOCITY_D,
                                                    getMotorVelocityF(MOTOR_MAX_RPM / 60 * ENCODER_TICKS_PER_REV));

        // Motor Encoder Feed Forward Configuration(s)
        public static final double FEED_FORWARD_KV = 1.0 / rpmToVelocity(MOTOR_MAX_RPM); // volts * seconds / distance
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

            outInches = WHEEL_RADIUS_INCHES * 2 * Math.PI * EXTERNAL_GEAR_RATIO * inTicks / ENCODER_TICKS_PER_REV;

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

            outVelocity = inRpm * EXTERNAL_GEAR_RATIO * Math.PI * WHEEL_RADIUS_INCHES / 60.0;

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

    public static final class LinearSlide {
        // Hardware Settings based on the following [comments]:
        // Linear Slide specific Constants
        public static final String COMMENT_LINEARSLIDE_MOTOR = "<enter motor maker/model>";

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        // Encoder Set Points for each Goal (High/Med/Low/Ground)
        public static final int ENCODER_SET_POINT_HIGH_GOAL = 2000;
        public static final int ENCODER_SET_POINT_MED_GOAL = 1500;
        public static final int ENCODER_SET_POINT_LOW_GOAL = 1000;
        public static final int ENCODER_SET_POINT_GROUND_GOAL = 500;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75;
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.50;
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.20;

    }

    public static final class Claw {
        // Hardware Settings based on the following [comments]:
        // 'The Claw' specific Constants
        public static final String COMMENT_CLAW_SERVO = "<enter servo maker/model>";

        // Servo Position(s)
        // Claw Clamp
        public static final double SERVO_POSITION_CLAW_CLAMP_START = .50;
        public static final double SERVO_POSITION_CLAW_CLAMP_OPEN = .10;
        public static final double SERVO_POSITION_CLAW_CLAMP_CLOSE = .80;

        // Claw Rotate Side-to-side
        public static final double SERVO_POSITION_CLAW_ROTATE_SIDE_START = .50;
        public static final double SERVO_POSITION_CLAW_ROTATE_SIDE_LEFT = .10;
        public static final double SERVO_POSITION_CLAW_ROTATE_SIDE_RIGHT = .90;

        // Claw Rotate Up-Down
        public static final double SERVO_POSITION_CLAW_ROTATE_UPDOWN_START = .50;
        public static final double SERVO_POSITION_CLAW_ROTATE_UPDOWN_UP = .10;
        public static final double SERVO_POSITION_CLAW_ROTATE_UPDOWN_DOWN = .90;


    }

    public static final class Lighting {
        // Hardware Settings based on the following [comments]:
        // Lighting specific Constants
        public static final String COMMENT_LIGHTING_CONTROLLER = "<enter controller maker/model>";

        // Default Light Pattern
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT = RevBlinkinLedDriver.BlinkinPattern.GREEN;

        // Light Patterns for Robot State(s) - Linear Slide
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;

        // Light Patterns for Robot State(s) - Claw
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_OPEN = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_CLOSED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;

    }

    public static final class Vision {
        // Hardware Settings based on the following [comments]:
        // 'The Claw' specific Constants
        public static final String COMMENT_VISION_CAMERA = "<enter camera maker/model>";

    }

}
