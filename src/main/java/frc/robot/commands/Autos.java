/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.UserInterface;

public final class Autos {
    private SendableChooser<Command> autoChooser;
    private static Autos autos;
    PathConstraints pathConstraints = new PathConstraints(4, 2, 2, 2);

    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public SendableChooser<Command> getAutoChooser() {
        if (autoChooser == null) {
            autoChooser = AutoBuilder.buildAutoChooser();
            UserInterface.getTab("Auton").add("AutoChooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(1, 1).withPosition(0, 0);
        }

        return autoChooser;
    }

    public static class OnePiece {

        public static class Blue {
            public static Command onePieceBlueHPBL4() {
                return new PathPlannerAuto("1 Piece Blue HPB L4");
            }

            public static Command onePieceBlueHPTL4() {
                return new PathPlannerAuto("1 Piece Blue HPT L4");
            }
        }

        public static class Red {
            public static Command onePieceRedHPBL4() {
                return new PathPlannerAuto("1 Piece Red HPB L4");
            }

            public static Command onePieceRedHPTL4() {
                return new PathPlannerAuto("1 Piece Red HPT L4");
            }

        }
    }


    public static class TwoPiece {
        public static class Blue {
            public static Command twoPieceBlueGBottomL4() {
                return new PathPlannerAuto("2 Piece Blue G Bottom L4");
            }

            public static Command twoPieceBlueHTopL4() {
                return new PathPlannerAuto("2 Piece Blue H Top L4");
            }

            public static Command twoPieceBlueDBottomL4() {
                return new PathPlannerAuto("2 Piece Blue D Bottom L4");
            }

            public static Command twoPieceBlueKTopL4() {
                return new PathPlannerAuto("2 Piece Blue K Top L4");
            }
        }

        public static class Red {
            public static Command twoPieceRedHBottomL4() {
                return new PathPlannerAuto("2 Piece Red H Bottom L4");
            }

            public static Command twoPieceRedGTopL4() {
                return new PathPlannerAuto("2 Piece Red G Top L4");
            }

            public static Command twoPieceRedKBottomL4() {
                return new PathPlannerAuto("2 Piece K Red Bottom L4");
            }

            public static Command twoPieceRedDTopL4() {
                return new PathPlannerAuto("2 Piece D Red Top L4");
            }
        }
    }


    public static class ThreePiece {
        public static class Blue {
            public static Command threePieceBlueBL1() {
                return new PathPlannerAuto("3 Piece Blue Bottom L1");
            }

            public static Command threePieceBlueTL1() {
                return new PathPlannerAuto("3 Piece Blue Top L1");
            }

            public static Command threePieceBlueBL4() {
                return new PathPlannerAuto("3 Piece Blue Bottom 2 L4");
            }

            public static Command robertdBlueThreePcTest() {
                return new PathPlannerAuto("3 Piece Blue Front C L4");
            }

            public static Command threePieceBlueTL4() {
                return new PathPlannerAuto("3 Piece Blue Top L4");
            }

        }

        public static class Red {
            public static Command threePieceRedBL1() {
                return new PathPlannerAuto("3 Piece Red Bottom L1");
            }

            public static Command threePieceRedTL1() {
                return new PathPlannerAuto("3 Piece Red Top L1");
            }

            public static Command threePieceRedBL4() {
                return new PathPlannerAuto("3 Piece Red Bottom L4");
            }

            public static Command threePieceRedTL4() {
                return new PathPlannerAuto("3 Piece Red Top L4");
            }
        }

    }

    public static class FourPiece {
        public static class Blue {
            public static Command fourPieceBlueBL1() {
                return new PathPlannerAuto("4 Piece Blue Bottom L1");
            }

            public static Command fourPieceBlueTL1() {
                return new PathPlannerAuto("4 Piece Blue Top L1");
            }

            public static Command fourPieceBlueBL4() {
                return new PathPlannerAuto("4 Piece Blue Bottom L4");
            }

            public static Command fourPieceBlueTL4() {
                return new PathPlannerAuto("4 Piece Blue Top L4");
            }

        }

        public static class Red {
            public static Command fourPieceRedBL1() {
                return new PathPlannerAuto("4 Piece Red Bottom L1");
            }

            public static Command fourPieceRedTL1() {
                return new PathPlannerAuto("4 Piece Red Top L1");
            }

            public static Command fourPieceRedBL4() {
                return new PathPlannerAuto("4 Piece Red Bottom L4");
            }

            public static Command fourPieceRedTL4() {
                return new PathPlannerAuto("4 Piece Red Top L4");
            }
        }

    }


    public static class GO_TO {

    }

}
