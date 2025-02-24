/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UserInterface;

public final class Autos {
    private static SendableChooser<Command> autoChooser;
    PathConstraints pathConstraints = new PathConstraints(4, 2, 2, 2);

    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public static  SendableChooser<Command> setupAutoChooser() {
        if (autoChooser == null) {
            autoChooser = AutoBuilder.buildAutoChooser();
            UserInterface.addSendableChooserComponent("AutoChooser", autoChooser, BuiltInWidgets.kComboBoxChooser, 1, 1, 0, 0, null);
        }

        return autoChooser;
    }

    public static class OnePiece {

        public static class Blue {
            public static Command onePieceBlueHPTL1() {
                return new PathPlannerAuto("1 Piece Blue HPT L1");
            }

            public static Command onePieceBlueHPBL1() {
                return new PathPlannerAuto("1 Piece Blue HPB L1");
            }

            public static Command onePieceBlueHPTL4() {
                return new PathPlannerAuto("1 Piece Blue HPT L4");
            }

            public static Command onePieceBlueHPBL4() {
                return new PathPlannerAuto("1 Piece Blue HPB L4");
            }
        }

        public static class Red {
            public static Command onePieceRedHPTL1() {
                return new PathPlannerAuto("1 Piece Red HPT L1");
            }

            public static Command onePieceRedHPBL1() {
                return new PathPlannerAuto("1 Piece Red HPB L1");
            }

            public static Command onePieceRedHPTL4() {
                return new PathPlannerAuto("1 Piece Red HPT L4");
            }

            public static Command onePieceRedHPBL4() {
                return new PathPlannerAuto("1 Piece Red HPB L4");
            }
        }
    }


    public static class TwoPiece {
        public static class Blue {
            public static Command twoPieceBlueBL1() {
                return new PathPlannerAuto("2 Piece Blue Bottom L1");
            }

            public static Command twoPieceBlueTL1() {
                return new PathPlannerAuto("2 Piece Blue Top L1");
            }

            public static Command twoPieceBlueBL4() {
                return new PathPlannerAuto("2 Piece Blue Bottom L4");
            }

            public static Command twoPieceBlueTL4() {
                return new PathPlannerAuto("2 Piece Blue Top L4");
            }

            public static Command twoPieceBlueFrontCL4() {
                return new PathPlannerAuto("2 Piece Blue Front C L4");
            }


        }

        public static class Red {
            public static Command twoPieceRedBL1() {
                return new PathPlannerAuto("2 Piece Red Bottom L1");
            }

            public static Command twoPieceRedTL1() {
                return new PathPlannerAuto("2 Piece Red Top L1");
            }

            public static Command twoPieceRedBL4() {
                return new PathPlannerAuto("2 Piece Red Bottom L4");
            }

            public static Command twoPieceRedTL4() {
                return new PathPlannerAuto("2 Piece Red Top L4");
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
}
