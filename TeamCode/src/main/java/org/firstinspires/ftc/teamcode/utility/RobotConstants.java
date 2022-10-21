package org.firstinspires.ftc.teamcode.utility;

// Program Class created from new Java Class
// Named in TeamCode as: RobotConstants.java

import android.os.Environment;

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

    // Device Storage (External Storage)
    public static final String DEVICE_EXTERNAL_STORAGE_PATH = Environment.getExternalStorageDirectory().getPath();
    public static final String TENSORFLOW_MODEL_EXTERNAL_STORAGE_DIRECTORY = "FIRST/tflitemodels/";
    public static final String TENSORFLOW_MODEL_EXTERNAL_STORAGE_PATH = DEVICE_EXTERNAL_STORAGE_PATH + TENSORFLOW_MODEL_EXTERNAL_STORAGE_DIRECTORY;

    /**
     *  <h2>Robot Constant Values - About The Robot</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines 'About the Team' robot constant settings and properties.
     * </p>
     * <li>Program Team</li>
     * <li>Development Season</li>
     * <br>
     */
    public static final class About {
        // Document Comments and any other 'About the Program' reference
        public static final String COMMENT_AUTHOR_NAME = "FTC Team 11531 - Green Team";
        public static final String COMMENT_SEASON_PERIOD = "2022 / 2023";
    }

    /**
     *  <h2>Robot Constant Values - Configuration</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines robot configuration robot constant settings and properties.
     * </p>
     * <li>Hardware Configuration (Label Names)</li>
     * <br>
     */
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

    /**
     *  <h2>Robot Constant Values - Common</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines common constants used throughout.
     * Can include timeout values, math functions, etc.
     * </p>
     * <li>Command Sleep Timeout(s)</li>
     * <li>Math Constants</li>
     * <br>
     */
    public static final class CommonSettings {
        public static final int SLEEP_TIMER_MILLISECONDS_DEFAULT = 50;

    }

    /**
     *  <h2>Robot Constant Values - Drivetrain</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Drivetrain constants
     * </p>
     * <li>Drivetrain Constants</li>
     * <br>
     */
    public static final class Drivetrain {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_DRIVE_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVE_WHEEL_TYPE = "Mecanum";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "Field Centric";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "Robot Centric";

        // Physical Robot Settings
        public static final double DRIVETRAIN_TRACK_WIDTH_INCHES = 10; // !! need to tune !!
        public static final double WHEEL_DIAMETER_MILLIMETER = 96; // !! need to tune !!
        public static final double WHEEL_DIAMETER_INCHES = convertMillimetersToInches(WHEEL_DIAMETER_MILLIMETER);

        public static final double WHEEL_RADIUS_MILLIMETER = WHEEL_DIAMETER_MILLIMETER / 2;
        public static final double WHEEL_RADIUS_INCHES = convertMillimetersToInches(WHEEL_RADIUS_MILLIMETER);

        public static final double EXTERNAL_GEAR_RATIO = 1.0; // 1 = No External Gearing // !! need to tune !!

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        public static final double ENCODER_TICKS_PER_INCH = (ENCODER_TICKS_PER_REV * EXTERNAL_GEAR_RATIO) /
                                                                    (WHEEL_DIAMETER_INCHES * Math.PI);

        // Motor Encoder PIDF Settings
        public static final double MOTOR_VELOCITY_P = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_I = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_D = 0; // !! need to tune !!
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
        public static final double LIMIT_MAX_VELOCITY = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ACCELERATION = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_VELOCITY = Math.toRadians(60); // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_ACCELERATION = Math.toRadians(60); // !! need to tune !!


        // Motor Output Setting(s)
        public static final double MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOTOR_OUTPUT_POWER_MAX = 1;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75;
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.50;
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.15;

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

    /**
     *  <h2>Robot Constant Values - Linear Slide</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Linear Slide constants
     * </p>
     * <li>Linear Slide Constants</li>
     * <br>
     */
    public static final class LinearSlide {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_LINEARSLIDE_MOTOR = "<enter motor maker/model>";

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        // Encoder Set Points for each Goal (High/Med/Low/Ground)
        public static final int ENCODER_SET_POINT_HIGH_GOAL = 2000;
        public static final int ENCODER_SET_POINT_MED_GOAL = 1500;
        public static final int ENCODER_SET_POINT_LOW_GOAL = 1000;
        public static final int ENCODER_SET_POINT_GROUND_GOAL = 500;

        // Motor Output Setting(s)
        public static final double MOTOR_OUTPUT_POWER_MAX = 1;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75;
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.50;
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.20;

    }

    /**
     *  <h2>Robot Constant Values - Claw</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Claw constants
     * </p>
     * <li>Claw Constants</li>
     * <br>
     */
    public static final class Claw {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_CLAW_SERVO = "<enter servo maker/model>";

        // Servo Position(s) [0.00 - 1.00]
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

    /**
     *  <h2>Robot Constant Values - Lighting</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Lighting constants
     * </p>
     * <li>Lighting Constants</li>
     * <br>
     */
    public static final class Lighting {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_LIGHTING_CONTROLLER = "REV-11-1105 Blinkin LED Driver";

        // Default Light Pattern
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT = RevBlinkinLedDriver.BlinkinPattern.GREEN;

        // System Initialize Light Patter/state
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_LIGHTING = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_LINEARSLIDE = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_CLAW = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_VISION = RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST;

        // Light Patterns for Robot State(s) - Linear Slide
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;

        // Light Patterns for Robot State(s) - Claw
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_OPEN = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_CLOSED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;

    }

    /**
     *  <h2>Robot Constant Values - Vision</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Vision constants
     * </p>
     * <li>Vision Constants</li>
     * <br>
     */
    public static final class Vision {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_VISION_CAMERA = "<enter camera maker/model>";

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        // Vuforia Initialization
        public static final String VUFORIA_LICENSE_KEY = "ATRnn8b/////AAABmeFNnAK1Okq+liJrV5mh0AgBKJo6aYC+NHCSsDAJnM8e8aw6DFF//uTcG+lY97v3ldt8xMDSpQrkf4IM67uu89gHXA6CH+txR6GEEMWahM0/7/uTY5NoOmVqBOP6bQZ8SmQQJtveI+ab2DOZmJj2zG1QzPJG1rBbNo6a5SPXMXGrRSaIoWxG+8ZmSQ0IcB28A+NwS9O5lMRQvvEtHb2v5zJnZuUo+8Fwi4tJNM16xZkTeMTIqlP5p8K+Zn9wqsFAUysWFlLU0l1HAM/2G0C8PVFFSjYdBFBlpSWq07UiN6HJpx7/0wlApfIlg06+aepqiZpNOe8Sp2EeJcfYkbe6uvb6AN5c1hkdR16RspDFMSY8";
        public static final String VUFORIA_LICENSE_KEY_NAME = "ftc11531_green";

        // Tensor Flow Configuration
        public static final String TENSORFLOW_CONFIG_MONITOR_VIEW_ID = "tensorFlowMonitorViewID";
        public static final String TENSORFLOW_CONFIG_MONITOR_VIEW_DEF_TYPE = "id";

        // Tensor Flow Image Processing
        public static final double TENSORFLOW_IMAGE_PROCESSING_IMAGE_WIDTH = 16.0;
        public static final double TENSORFLOW_IMAGE_PROCESSING_IMAGE_HEIGHT = 9.0;
        public static final double TENSORFLOW_IMAGE_PROCESSING_MAGNIFICATION = 1.0;
        public static final double TENSORFLOW_IMAGE_PROCESSING_ASPECT_RATIO = TENSORFLOW_IMAGE_PROCESSING_IMAGE_WIDTH
                                                                                / TENSORFLOW_IMAGE_PROCESSING_IMAGE_HEIGHT;

        // Tensor Flow Parameter(s)
        public static final float TENSORFLOW_PARAMETER_MIN_RESULT_CONFIDENCE = 0.75f;
        public static final boolean TENSORFLOW_PARAMETER_IS_MODEL_TENSORFLOW2 = true;
        public static final int TENSORFLOW_PARAMETER_INPUT_SIZE = 300;

        // TensorFlow Model
        public static final String TENSORFLOW_MODEL_ID_POWERPLAY_FIRST = "config_first_comp";
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST = "PowerPlay.tflite";

        public static final String TENSORFLOW_MODEL_ID_POWERPLAY_GREEN = "config_green_comp";
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILE = "tf_ftc11531_powerplay.tflite";
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_PATH = TENSORFLOW_MODEL_EXTERNAL_STORAGE_PATH;
        //public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_PATH = "/sdcard/" + TENSORFLOW_MODEL_EXTERNAL_STORAGE_DIRECTORY;
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILEPATH = TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_PATH + TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILE;

        public static final String TENSORFLOW_MODEL_ID_POWERPLAY_BLUE = "config_blue_comp";
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILE = "tf_ftc18092_powerplay.tflite";
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_PATH = TENSORFLOW_MODEL_EXTERNAL_STORAGE_PATH;
        //public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_PATH = "/sdcard/" + TENSORFLOW_MODEL_EXTERNAL_STORAGE_DIRECTORY;
        public static final String TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILEPATH = TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_PATH + TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILE;

        public static final String[] TENSORFLOW_MODEL_LABELS_POWERPLAY_FIRST = {
                "1 Bolt",
                "2 Bulb",
                "3 Panel"
        };

        public static final String[] TENSORFLOW_MODEL_LABELS_POWERPLAY_GREEN = {
                "1 Peaches",
                "2 Trojan",
                "3 Cards"
        };

        public static final String[] TENSORFLOW_MODEL_LABELS_POWERPLAY_BLUE = {
                "1 Peaches",
                "2 Trojan",
                "3 Cards"
        };

    }

    /**
     * <h2>RobotConstants Method: convertMillimetersToInches</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from millimeters to inches
     * </p>
     * @param inValueInMillimeters - value to be converted (in millimeters)
     *
     * @return double - value converted to inches
     */
    public static double convertMillimetersToInches(double inValueInMillimeters) {
        return inValueInMillimeters / CONVERSION_MM_TO_INCH;
    }

    /**
     * <h2>RobotConstants Method: convertInchesToMillimeters</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from inches to millimeters
     * </p>
     * @param inValueInInches - value to be converted (in inches)
     *
     * @return double - value converted to millimeters
     */
    public static double convertInchesToMillimeters(double inValueInInches) {
        return inValueInInches * CONVERSION_MM_TO_INCH;
    }
}
