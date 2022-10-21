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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

// Program Copied from FTC example: RobotHardware.java
// Renamed in TeamCode as: SysClaw.java

/**
 * <h2>System Class for The Claw</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <b>Claw Servo(s):</b> {@value RobotConstants.Claw#COMMENT_CLAW_SERVO}<br>
 * <hr>
 * <p>
 * This system defines <b>ALL</b> configures all attributes, configurations, and methods for the
 * claw.
 * </p>
 * <hr>
 *
 */
public class SysClaw {

    /* Declare OpMode members. */
    private LinearOpMode sysOpMode = null;   // gain access to methods in the calling OpMode.

    // Define hardware objects  (Make them private so they can't be accessed externally)
    private Servo rotateSideClawServo, rotateUpDownClawServo, clampClawServo;

    /**
     * <h2>Claw System Constructor</h2>
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
    public SysClaw(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    /**
     * <h2>Claw System Initialize</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Initialize the robot hardware for the claw system.
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
        rotateSideClawServo = sysOpMode.hardwareMap.get(Servo.class, RobotConstants.Configuration.LABEL_SERVO_CLAW_ROTATE_SIDE);
        rotateUpDownClawServo = sysOpMode.hardwareMap.get(Servo.class, RobotConstants.Configuration.LABEL_SERVO_CLAW_ROTATE_UPDOWN);
        clampClawServo = sysOpMode.hardwareMap.get(Servo.class, RobotConstants.Configuration.LABEL_SERVO_CLAW_CLAMP);

        // Set Initial Position for Servo(s)
        setClawClampPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_CLAMP_START);
        setClawSidePosition(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_SIDE_START);
        setClawUpDownPosition(RobotConstants.Claw.SERVO_POSITION_CLAW_ROTATE_UPDOWN_START);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.addData(">", " System: The Claw Initialized");
        sysOpMode.telemetry.addData(">", "--------------------------------");
        sysOpMode.telemetry.update();
    }

    /**
     * <h2>Claw Method: getClawClampPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the current position for the claw clamp.
     * </p>
     * @return double - servo position
     * <br>
     */
    public double getClawClampPosition() {
        return clampClawServo.getPosition();
    }

    /**
     * <h2>Claw Method: getClawSidePosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the current position for the claw clamp.
     * </p>
     * @return double - servo position
     * <br>
     */
    public double getClawSidePosition() {
        return rotateSideClawServo.getPosition();
    }

    /**
     * <h2>Claw Method: getClawUpDownPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the current position for the claw clamp.
     * </p>
     * @return double - servo position
     * <br>
     */
    public double getClawUpDownPosition() {
        return rotateUpDownClawServo.getPosition();
    }

    /**
     * <h2>Claw Method: setClawClampPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the claw clamp to the target position
     * </p>
     * @param inTargetPosition double - Target position to move servo to
     *
     * <br>
     */
    public void setClawClampPosition(double inTargetPosition) {

        // Move Clamp to position
        clampClawServo.setPosition(inTargetPosition);
    }

    /**
     * <h2>Claw Method: setClawSidePosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the claw clamp to the target position
     * </p>
     * @param inTargetPosition double - Target position to move servo to
     *
     * <br>
     */
    public void setClawSidePosition(double inTargetPosition) {

        // Move Clamp to position
        rotateSideClawServo.setPosition(inTargetPosition);
    }

    /**
     * <h2>Claw Method: setClawUpDownPosition</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Set the claw clamp to the target position
     * </p>
     * @param inTargetPosition double - Target position to move servo to
     *
     * <br>
     */
    public void setClawUpDownPosition(double inTargetPosition) {

        // Move Clamp to position
        rotateUpDownClawServo.setPosition(inTargetPosition);
    }

}
