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

    private Autos() {
        initalizeNamedCommands();
    }

    public static Autos getInstance() {
        if (autos == null) autos = new Autos();
        return autos;
    }

    public void initalizeNamedCommands() {
        NamedCommands.registerCommand("INTAKE", ScoreCommands.intake());
        NamedCommands.registerCommand("OUTTAKE", ScoreCommands.outtake());
        NamedCommands.registerCommand("HP", ScoreCommands.intakeHP());
        NamedCommands.registerCommand("L1", ScoreCommands.scoreL1());
        NamedCommands.registerCommand("L2", ScoreCommands.scoreL2());
        NamedCommands.registerCommand("L3", ScoreCommands.scoreL3());
        NamedCommands.registerCommand("L4", ScoreCommands.scoreL4());
        NamedCommands.registerCommand("armL2", ScoreCommands.armL2());
        NamedCommands.registerCommand("armL3", ScoreCommands.armL3());
        NamedCommands.registerCommand("armL4", ScoreCommands.armL4());
    }

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

    public class OnePiece {

        public class Blue {
            public Command onePieceBlueHPTL1() {
                return new PathPlannerAuto("1 Piece Blue HPT L1");
            }

            public Command onePieceBlueHPBL1() {
                return new PathPlannerAuto("1 Piece Blue HPB L1");
            }

            public Command onePieceBlueHPTL4() {
                return new PathPlannerAuto("1 Piece Blue HPT L4");
            }

            public Command onePieceBlueHPBL4() {
                return new PathPlannerAuto("1 Piece Blue HPB L4");
            }
        }

        public class Red {
            public Command onePieceRedHPTL1() {
                return new PathPlannerAuto("1 Piece Red HPT L1");
            }

            public Command onePieceRedHPBL1() {
                return new PathPlannerAuto("1 Piece Red HPB L1");
            }

            public Command onePieceRedHPTL4() {
                return new PathPlannerAuto("1 Piece Red HPT L4");
            }

            public Command onePieceRedHPBL4() {
                return new PathPlannerAuto("1 Piece Red HPB L4");
            }
        }
    }


    public class TwoPiece {
        public class Blue {
            public Command twoPieceBlueBL1() {
                return new PathPlannerAuto("2 Piece Blue Bottom L1");
            }

            public Command twoPieceBlueTL1() {
                return new PathPlannerAuto("2 Piece Blue Top L1");
            }

            public Command twoPieceBlueBL4() {
                return new PathPlannerAuto("2 Piece Blue Bottom L4");
            }

            public Command twoPieceBlueTL4() {
                return new PathPlannerAuto("2 Piece Blue Top L4");
            }

        }

        public class Red {
            public Command twoPieceRedBL1() {
                return new PathPlannerAuto("2 Piece Red Bottom L1");
            }

            public Command twoPieceRedTL1() {
                return new PathPlannerAuto("2 Piece Red Top L1");
            }

            public Command twoPieceRedBL4() {
                return new PathPlannerAuto("2 Piece Red Bottom L4");
            }

            public Command twoPieceRedTL4() {
                return new PathPlannerAuto("2 Piece Red Top L4");
            }
        }
    }


    public class ThreePiece {
        public class Blue {
            public Command threePieceBlueBL1() {
                return new PathPlannerAuto("3 Piece Blue Bottom L1");
            }

            public Command threePieceBlueTL1() {
                return new PathPlannerAuto("3 Piece Blue Top L1");
            }

            public Command threePieceBlueBL4() {
                return new PathPlannerAuto("3 Piece Blue Bottom L4");
            }

            public Command threePieceBlueTL4() {
                return new PathPlannerAuto("3 Piece Blue Top L4");
            }

        }

        public class Red {
            public Command threePieceRedBL1() {
                return new PathPlannerAuto("3 Piece Red Bottom L1");
            }

            public Command threePieceRedTL1() {
                return new PathPlannerAuto("3 Piece Red Top L1");
            }

            public Command threePieceRedBL4() {
                return new PathPlannerAuto("3 Piece Red Bottom L4");
            }

            public Command threePieceRedTL4() {
                return new PathPlannerAuto("3 Piece Red Top L4");
            }
        }

    }

    public class FourPiece {
        public class Blue {
            public Command fourPieceBlueBL1() {
                return new PathPlannerAuto("4 Piece Blue Bottom L1");
            }

            public Command fourPieceBlueTL1() {
                return new PathPlannerAuto("4 Piece Blue Top L1");
            }

            public Command fourPieceBlueBL4() {
                return new PathPlannerAuto("4 Piece Blue Bottom L4");
            }

            public Command fourPieceBlueTL4() {
                return new PathPlannerAuto("4 Piece Blue Top L4");
            }

        }

        public class Red {
            public Command fourPieceRedBL1() {
                return new PathPlannerAuto("4 Piece Red Bottom L1");
            }

            public Command fourPieceRedTL1() {
                return new PathPlannerAuto("4 Piece Red Top L1");
            }

            public Command fourPieceRedBL4() {
                return new PathPlannerAuto("4 Piece Red Bottom L4");
            }

            public Command fourPieceRedTL4() {
                return new PathPlannerAuto("4 Piece Red Top L4");

            }
        }

    }


    public class GO_TO {

    }


}
