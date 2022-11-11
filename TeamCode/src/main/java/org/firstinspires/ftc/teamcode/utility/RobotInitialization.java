package org.firstinspires.ftc.teamcode.utility;

import android.app.Activity;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotInitialization {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode;   // gain access to methods in the calling OpMode.

    // Robot hardware configuration
    private RobotConfigFileManager robotConfigFileManager;

    /**
     * <h2>Robot Initialization Constructor</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Define a constructor that allows the OpMode to pass a reference to itself.
     * </p>
     * <hr>
     * @param inOpMode
     */
    public RobotInitialization(LinearOpMode inOpMode) { sysOpMode = inOpMode; }

    public void init() {

        // The robot config name is passed into the initialization of this system
        robotConfigFileManager = new RobotConfigFileManager((Activity) sysOpMode.hardwareMap.appContext);
        RobotConstants.CommonSettings.setRobotConfigurationFileManagerNameActive(robotConfigFileManager.getActiveConfig().getName());

    }

    public void displayInitializationSettingsAutonomous(String inDisplayMode) {

        // ------------------------------------------------------------
        // Allow for Start Option Selection
        // ------------------------------------------------------------
        // Display Mode Key:
        // - 'autonomous' = Autonomous Op
        // - 'teleop' = Teleop

        // Init Loop to allow for Start Option Selection(s)
        while (sysOpMode.opModeInInit()) {
            sysOpMode.telemetry.setAutoClear(false);
            sysOpMode.telemetry.clearAll();

            sysOpMode.telemetry.addData(">", "------------------------------------");
            sysOpMode.telemetry.addData(">", "Setting Selection Mode Available");
            sysOpMode.telemetry.addData(">", "------------------------------------");
            sysOpMode.telemetry.addData(">", " Use Main Driver Gamepad");
            sysOpMode.telemetry.addData(">", "------------------------------------");

            switch (inDisplayMode) {
                case RobotConstants.CommonSettings.INIT_SETTING_DISPLAY_MODE_AUTONOMOUS:
                    switch (RobotConstants.CommonSettings.getInitializationSettingAutonomousMode()) {
                        case(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_MODE_RIGHT):
                            sysOpMode.telemetry.addData(">>>", "Press (X) - Left Autonomous Mode");
                            sysOpMode.telemetry.addData(">>>", "Press (B) - Right Autonomous Mode (ACTIVE)");
                            break;
                        case(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_MODE_LEFT):
                            sysOpMode.telemetry.addData(">>>", "Press (X) - Left Autonomous Mode (ACTIVE)");
                            sysOpMode.telemetry.addData(">>>", "Press (B) - Right Autonomous Mode");
                            break;
                        default:
                            sysOpMode.telemetry.addData(">>>", "Press (X) - Left Autonomous Mode");
                            sysOpMode.telemetry.addData(">>>", "Press (B) - Right Autonomous Mode");
                    }

                    switch (RobotConstants.CommonSettings.getInitializationSettingAutonomousImageSource()) {
                        case(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_GREEN):
                            sysOpMode.telemetry.addData(">>>", "Press (Y) - Custom Images (ACTIVE)");
                            sysOpMode.telemetry.addData(">>>", "Press (A) - Stock Images");
                            break;
                        case(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_STOCK):
                            sysOpMode.telemetry.addData(">>>", "Press (Y) - Custom Images");
                            sysOpMode.telemetry.addData(">>>", "Press (A) - Stock Images (ACTIVE)");
                            break;
                        default:
                            sysOpMode.telemetry.addData(">>>", "Press (Y) - Custom Images");
                            sysOpMode.telemetry.addData(">>>", "Press (A) - Stock Images");
                    }
                    break;
                case RobotConstants.CommonSettings.INIT_SETTING_DISPLAY_MODE_TELEOP:
                    sysOpMode.telemetry.addData(">>>", "No Options Available");
                    break;
                default:
                    sysOpMode.telemetry.addData(">>>", "Unavailable");
            }

            sysOpMode.telemetry.addData(">", "------------------------------------");
            sysOpMode.telemetry.update();

            // Set Autonomous Mode to Left Side
            if(sysOpMode.gamepad1.x) {
                RobotConstants.CommonSettings.setInitializationSettingAutonomousMode(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_MODE_LEFT);
            }

            // Set Autonomous Mode to Right Side
            if(sysOpMode.gamepad1.b) {
                RobotConstants.CommonSettings.setInitializationSettingAutonomousMode(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_MODE_RIGHT);
            }

            // Set image model to use custom model
            if(sysOpMode.gamepad1.y) {
                RobotConstants.CommonSettings.setInitializationSettingAutonomousImageSource(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_GREEN);
            }

            // Set image model to use custom model
            if(sysOpMode.gamepad1.a) {
                RobotConstants.CommonSettings.setInitializationSettingAutonomousImageSource(RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_STOCK);
            }

            // Pace this loop so commands move at a reasonable speed.
            sysOpMode.sleep(RobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }

    }

}
