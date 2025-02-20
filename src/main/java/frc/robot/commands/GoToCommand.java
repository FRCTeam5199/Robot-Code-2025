package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public final class GoToCommand {
    private static enum redSideTagIDs {
        TAG7I,
        TAG8,
        TAG9,
        TAG10,
        TAG11

        // private static final int tagID;

        // redSideTagIDs(int tagID) { this.tagID = tagID; }
    }
    private static enum blueSideTagIDs {
        TAG17,
        TAG88,
        TAG19,
        TAG20,
        TAG21
    }

    redSideTagIDs selectedRedSideTagID = null;
    blueSideTagIDs selectedBlueSideTagID = null;

    private GoToCommand() {}

    /**
     * 
     * @param tagID
     * @param right True = right, false = left
     */
    public Command setTagID(int tagID) {
        return null; //TODO
    }

    /**
     * @param right True = right, false = left
     */
    public Command setSide(boolean right) {
        return null; //TODO
    }
}
