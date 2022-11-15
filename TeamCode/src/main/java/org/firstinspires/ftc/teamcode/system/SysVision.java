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

import android.app.Activity;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysVision.java

/**
 * <h2>System Class for the Robot Vision</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <b>Camera:</b> {@value RobotConstants.Vision#COMMENT_VISION_CAMERA}<br>
 * <hr>
 * <p>
 * This system defines <b>ALL</b> configures all attributes, configurations, and methods for the robot
 * vision system.
 * </p>
 * <hr>
 *
 */
public class SysVision {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tensorFlowObjectDetector;

    // Tensor Flow Model used
    private String modelSelectionFileName;
    private String modelSelectionFilePath;

    /**
     * <h2>Vision System Constructor</h2>
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
    public SysVision(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Vision System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the vision system.
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

        // Initialize Vuforia and Tensor Flow
        initializeVuforia();
        initializeTensorFlow();

        // Activate Tensor Flow during Init (before start)
        if (tensorFlowObjectDetector != null) {
            tensorFlowObjectDetector.activate();

            tensorFlowObjectDetector.setZoom(RobotConstants.Vision.TENSORFLOW_IMAGE_PROCESSING_MAGNIFICATION
                                            , RobotConstants.Vision.TENSORFLOW_IMAGE_PROCESSING_ASPECT_RATIO);
        }

        // Display telemetry
        sysOpMode.telemetry.addData(">", "------------------------------------");
        sysOpMode.telemetry.addData(">", " System: Vision Initialized");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Vision Method: initializeVuforia</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the Vuforia localization engine.
     * </p>
     * <br>
     */
    private void initializeVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

        // Configure License Key and Camera for Vuforia Engine
        vuforiaParameters.vuforiaLicenseKey = RobotConstants.Vision.VUFORIA_LICENSE_KEY;
        vuforiaParameters.cameraName = sysOpMode.hardwareMap.get(WebcamName.class, RobotConstants.Configuration.LABEL_CAMERA_VISION_TRACKING);

        //  Instantiate the Vuforia engine
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(vuforiaParameters);
    }

    /**
     * <h2>Vision Method: initializeTensorFlow</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the TensorFlow Object Detection engine.
     * </p>
     * <br>
     */
    private void initializeTensorFlow() {
        // Tensor Flow Hardware Configuration/Id
        int tensorFlowMonitorViewID = sysOpMode.hardwareMap.appContext.getResources().getIdentifier(
                RobotConstants.Vision.TENSORFLOW_CONFIG_MONITOR_VIEW_ID
                , RobotConstants.Vision.TENSORFLOW_CONFIG_MONITOR_VIEW_DEF_TYPE
                , sysOpMode.hardwareMap.appContext.getPackageName()
        );

        // Setup Tensor Flow Parameters
        TFObjectDetector.Parameters tensorFlowParameters = new TFObjectDetector.Parameters(tensorFlowMonitorViewID);
        tensorFlowParameters.minResultConfidence = RobotConstants.Vision.TENSORFLOW_PARAMETER_MIN_RESULT_CONFIDENCE;
        tensorFlowParameters.isModelTensorFlow2 = RobotConstants.Vision.TENSORFLOW_PARAMETER_IS_MODEL_TENSORFLOW2;
        tensorFlowParameters.inputSize = RobotConstants.Vision.TENSORFLOW_PARAMETER_INPUT_SIZE;

        // Configure Tensor Flow Object Detector
        tensorFlowObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tensorFlowParameters, vuforiaLocalizer);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        switch(RobotConstants.CommonSettings.getInitializationSettingAutonomousImageSource()) {
            case (RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_GREEN):
                tensorFlowObjectDetector.loadModelFromAsset(RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST
                        , RobotConstants.Vision.TENSORFLOW_MODEL_LABELS_POWERPLAY_FIRST);

                // Store the value(s) for the model to be referenced
                modelSelectionFileName = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST;
                modelSelectionFilePath = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST;

//                // Custom TensorFlow Model
//                tensorFlowObjectDetector.loadModelFromFile(RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILEPATH
//                        , RobotConstants.Vision.TENSORFLOW_MODEL_LABELS_POWERPLAY_GREEN);
//
//                // Store the value(s) for the model to be referenced
//                modelSelectionFileName = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILE;
//                modelSelectionFilePath = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILEPATH;

                break;

            case (RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_BLUE):
                // TensorFlow Model for Blue Team
                tensorFlowObjectDetector.loadModelFromFile(RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILEPATH
                        , RobotConstants.Vision.TENSORFLOW_MODEL_LABELS_POWERPLAY_BLUE);

                // Store the value(s) for the model to be referenced
                modelSelectionFileName = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILE;
                modelSelectionFilePath = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_BLUE_FILEPATH;

                break;

            default:
                // TensorFlow Model for Default First Images
//                tensorFlowObjectDetector.loadModelFromAsset(RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST
//                        , RobotConstants.Vision.TENSORFLOW_MODEL_LABELS_POWERPLAY_FIRST);
//
//                // Store the value(s) for the model to be referenced
//                modelSelectionFileName = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST;
//                modelSelectionFilePath = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_FIRST;
                // Custom TensorFlow Model
                tensorFlowObjectDetector.loadModelFromFile(RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILEPATH
                        , RobotConstants.Vision.TENSORFLOW_MODEL_LABELS_POWERPLAY_GREEN);

                // Store the value(s) for the model to be referenced
                modelSelectionFileName = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILE;
                modelSelectionFilePath = RobotConstants.Vision.TENSORFLOW_MODEL_ASSET_POWERPLAY_GREEN_FILEPATH;

        }

    }

    /**
     * <h2>Vision Method: getCurrentModelFileName</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * TensorFlow Model information
     * </p>
     *
     * @return String - Return the name of the TensorFlow model used
     */
    public String getCurrentModelFileName() {
        return modelSelectionFileName;
    }

    /**
     * <h2>Vision Method: getCurrentModelFilePath</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * TensorFlow Model information
     * </p>
     *
     * @return String - Return the file path to the TensorFlow model used
     */
    public String getCurrentModelFilePath() {
        return modelSelectionFilePath;
    }

    /**
     * <h2>Vision Method: getVisonRecognitions</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get list of recognition(s) from TensorFlow object detection
     * </p>
     * @return List<Recognition> - List of Recognition(s)
     *
     * <br>
     */
    public List<Recognition> getRecognitionList() {
        List<Recognition> outputRecognitionList = null;

        // Process the Current Image from the camera and return recognition data for the image
        if (tensorFlowObjectDetector != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            outputRecognitionList = tensorFlowObjectDetector.getUpdatedRecognitions();
        }
        return outputRecognitionList;
    }

    /**
     * <h2>Vision Method: getRecognition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get recognition from list
     * </p>
     * @param inListRecognition - Get viable recognition from input list
     * @return Recognition - Return the recognition from the list
     */
    public Recognition getRecognition(List<Recognition> inListRecognition) {
        Recognition outRecognition = null;

        if(inListRecognition != null) {
            for (Recognition objRecognition : inListRecognition) {

                if (objRecognition.getConfidence() >= RobotConstants.Vision.RECOGNITION_IMAGE_CONFIDENCE_PERCENT_MIN) {
                    // Found an image with high confidence
                    outRecognition = objRecognition;
                }
            }
        }

        return outRecognition;
    }

    /**
     * <h2>Vision Method: getRecognitionColumn</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get recognition information - column info
     * </p>
     * @param inRecognition - recognition to get information from
     * @return double - column information
     */
    public double getRecognitionColumn(Recognition inRecognition) {
        double outRecognitionColumn = 0;

        if(inRecognition != null) {
            outRecognitionColumn = (inRecognition.getLeft() + inRecognition.getRight()) / 2;
        }

        return outRecognitionColumn;
    }

    /**
     * <h2>Vision Method: getRecognitionRow</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get recognition information - row info
     * </p>
     * @param inRecognition - recognition to get information from
     * @return double - row information
     */
    public double getRecognitionRow(Recognition inRecognition) {
        double outRecognitionRow = 0;

        if(inRecognition != null) {
            outRecognitionRow = (inRecognition.getTop()  + inRecognition.getBottom()) / 2;
        }
        return outRecognitionRow;
    }

    /**
     * <h2>Vision Method: getRecognitionWidth</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get recognition information - width info
     * </p>
     * @param inRecognition - recognition to get information from
     * @return double - width information
     */
    public double getRecognitionWidth(Recognition inRecognition) {
        double outRecognitionWidth = 0;

        if(inRecognition != null) {
            outRecognitionWidth = Math.abs(inRecognition.getRight() - inRecognition.getLeft());
        }

        return outRecognitionWidth;
    }

    /**
     * <h2>Vision Method: getRecognitionHeight</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get recognition information - height info
     * </p>
     * @param inRecognition - recognition to get information from
     * @return double - height information
     */
    public double getRecognitionHeight(Recognition inRecognition) {
        double outRecognitionHeight = 0;

        if(inRecognition != null) {
            outRecognitionHeight = Math.abs(inRecognition.getTop()  - inRecognition.getBottom());
        }

        return outRecognitionHeight;
    }

    /**
     * <h2>Vision Method: getTargetZone</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the target zone from the identified vision target label
     * </p>
     * @param inVisionTargetLabel - identified vision target label
     * @return int - return the target zone identifier (1,2,3)
     */
    public int getTargetZone(String inVisionTargetLabel) {
        int outTargetZone;

        // Switch on TensorFlow Model - Default is FIRST Robotics
        // Then switch on vision label
        // NOTE: Should be able to clean this logic up a bit. Not completely sold on the double switch.
        // - Cannot do a simple single switch on zone because green and blue values are the same
        // - Maybe do a single switch/case based on model_id combined with zone label (will need to think about it)
        switch(RobotConstants.CommonSettings.getInitializationSettingAutonomousImageSource()) {
            case (RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_GREEN):
                switch(inVisionTargetLabel) {
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_GREEN_ZONE1):
                        outTargetZone = 1;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_GREEN_ZONE2):
                        outTargetZone = 2;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_GREEN_ZONE3):
                        outTargetZone = 3;
                        break;
                    default:
                        outTargetZone = 0;
                }
                break;

            case (RobotConstants.CommonSettings.INIT_SETTING_AUTONOMOUS_IMAGE_SOURCE_CUSTOM_BLUE):
                switch(inVisionTargetLabel) {
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_BLUE_ZONE1):
                        outTargetZone = 1;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_BLUE_ZONE2):
                        outTargetZone = 2;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_BLUE_ZONE3):
                        outTargetZone = 3;
                        break;
                    default:
                        outTargetZone = 0;
                }
                break;

            default:
                switch(inVisionTargetLabel) {
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_FIRST_ZONE1):
                        outTargetZone = 1;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_FIRST_ZONE2):
                        outTargetZone = 2;
                        break;
                    case (RobotConstants.Vision.TENSORFLOW_MODEL_LABEL_POWERPLAY_FIRST_ZONE3):
                        outTargetZone = 3;
                        break;
                    default:
                        outTargetZone = 0;
                }
        }

        return outTargetZone;
    }

}
