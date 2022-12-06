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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysSound.java

/**
 * <h2>System Class for the Robot Sound Effects</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 */
public class SysSound {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    SoundPlayer defaultSoundPlayer;
    SoundPlayer.PlaySoundParams defaultSoundPlayerParameters;

    /**
     * <h2>Lighting System Constructor</h2>
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
    public SysSound(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Sound System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the lighting system.
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

        // Set Default Sound Player
        defaultSoundPlayer = SoundPlayer.getInstance();

        // Sound player parameters
        defaultSoundPlayerParameters = new SoundPlayer.PlaySoundParams();
        defaultSoundPlayerParameters.loopControl = 0;
        defaultSoundPlayerParameters.waitForNonLoopingSoundsToFinish = true;
        defaultSoundPlayerParameters.rate = 1.0f;
        defaultSoundPlayerParameters.volume = RobotConstants.Sound.SOUND_PLAYER_MASTER_VOLUME_PLAYSOUND_SETPOINT;

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: Sound Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    public void playSoundFileByName(String inSoundFileName) {

        if (checkSoundFileStatus(getSoundFileIDByName(inSoundFileName))) {
            defaultSoundPlayer.setMasterVolume(RobotConstants.Sound.SOUND_PLAYER_MASTER_VOLUME_PLAYSOUND_SETPOINT);

            defaultSoundPlayer.startPlaying(sysOpMode.hardwareMap.appContext
                    , getSoundFileIDByName(inSoundFileName)
                    , defaultSoundPlayerParameters
                    , null
                    , new Runnable() {
                        @Override
                        public void run() {
                            defaultSoundPlayer.setMasterVolume(RobotConstants.Sound.SOUND_PLAYER_MASTER_VOLUME_DEFAULT);
                        }
                    });
        }
    }

    public void stopAllSoundPlayback() {

        // Stop playing all active sounds!
        defaultSoundPlayer.stopPlayingAll();
        defaultSoundPlayer.stopPlayingLoops();
    }

    private boolean checkSoundFileStatus(int inSoundFileID) {
        boolean isSoundFileFound = false;

        if (inSoundFileID != 0) {
            isSoundFileFound = defaultSoundPlayer.preload(sysOpMode.hardwareMap.appContext, inSoundFileID);
        }

        return isSoundFileFound;
    }

    private int getSoundFileIDByName(String inSoundFileName) {

        // Sound ID will return Zero (0) if Sounds file not found
        return sysOpMode.hardwareMap.appContext.getResources().getIdentifier(inSoundFileName, "raw", sysOpMode.hardwareMap.appContext.getPackageName());
    }

}
