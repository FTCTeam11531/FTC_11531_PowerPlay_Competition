package org.firstinspires.ftc.teamcode.utility;

/**
 * <h2>Enum: Drivetrain Mode</h2>
 * <hr>
 * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Enumeration for the state of the Drivetrain Mode
 * </p>
 */
public enum StateDrivetrainMode {

    Field_Centric {

        // Setting for Next State
        @Override
        public StateDrivetrainMode nextState() {
            return Robot_Centric;
        }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() {
            return "Field Centric";
        }

    },
    Robot_Centric {

        // Setting for Next State
        @Override
        public StateDrivetrainMode nextState() {
            return Field_Centric;
        }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() {
            return "Robot Centric";
        }
    };

    /**
     * <h2>Enum Method: nextState</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Advance to the next state in the Drivetrain Mode enumeration
     * </p>
     * @return StateDrivetrainMode - sets the next state for the enumeration
     */
    public abstract StateDrivetrainMode nextState();

    /**
     * <h2>Enum Method: getLabel</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value RobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the label for the current state in the Drivetrain Mode enumeration
     * </p>
     * @return String - The string label for the state
     */
    public abstract String getLabel();

}
