/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UserInterface;

public final class Autos {
    private SendableChooser<Command> autoChooser;
    private static Autos autos;


    public static SendableChooser<Command> autonChooserRed = new SendableChooser<>();
    public static SendableChooser<Command> autonChooserBlue = new SendableChooser<>();

    private static PathPlannerAuto onePieceBlueHPBL4;
    private static PathPlannerAuto onePieceBlueHPTL4;
    private static PathPlannerAuto onePieceRedHPBL4;
    private static PathPlannerAuto onePieceRedHPTL4;
    private static PathPlannerAuto twoPieceBlueGHPBL4;
    private static PathPlannerAuto twoPieceBlueHHPTL4;
    private static PathPlannerAuto twoPieceBlueDHPBL4;
    private static PathPlannerAuto twoPieceBlueKHPTL4;
    private static PathPlannerAuto twoPieceRedHHPBL4;
    private static PathPlannerAuto twoPieceRedGHPTL4;
    private static PathPlannerAuto twoPieceRedKHPBL4;
    private static PathPlannerAuto twoPieceRedDHPTL4;

    private static PathPlannerAuto threePieceBlueBottomL4;


    private static PathPlannerAuto driveByTopBlue;
    private static PathPlannerAuto twoPieceDriveByTopBlue;

    private static PathPlannerAuto driveByBottomBlue;
    private static PathPlannerAuto twoPieceDriveByBottomBlue;

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

    public static void initializeAutos() {
        onePieceBlueHPBL4 = new PathPlannerAuto("1 Piece Blue HPB L4");
        onePieceBlueHPTL4 = new PathPlannerAuto("1 Piece Blue HPT L4");
        onePieceRedHPBL4 = new PathPlannerAuto("1 Piece Red HPB L4");
        onePieceRedHPTL4 = new PathPlannerAuto("1 Piece Red HPT L4");

        twoPieceBlueGHPBL4 = new PathPlannerAuto("2 Piece Blue G Bottom L4");
        twoPieceBlueHHPTL4 = new PathPlannerAuto("2 Piece Blue H Top L4");
        twoPieceBlueDHPBL4 = new PathPlannerAuto("2 Piece Blue D Bottom L4");
        twoPieceBlueKHPTL4 = new PathPlannerAuto("2 Piece Blue K Top L4");
        twoPieceRedHHPBL4 = new PathPlannerAuto("2 Piece Red H Bottom L4");
        twoPieceRedGHPTL4 = new PathPlannerAuto("2 Piece Red G Top L4");
        twoPieceRedKHPBL4 = new PathPlannerAuto("2 Piece K Red Bottom L4");
        twoPieceRedDHPTL4 = new PathPlannerAuto("2 Piece D Red Top L4");

        threePieceBlueBottomL4 = new PathPlannerAuto("3 Piece Blue Bottom 2 L4");

        driveByTopBlue = new PathPlannerAuto("Drive By");
        twoPieceDriveByTopBlue = new PathPlannerAuto("Drive By L4");

        driveByBottomBlue = new PathPlannerAuto("Drive By Blue Bottom");
        twoPieceDriveByBottomBlue = new PathPlannerAuto("Drive By Blue Bottom 2 Piece L4");


        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);

        autonChooserRed.addOption("1 Piece Red Right", onePieceRedHPTL4);
        autonChooserRed.addOption("1 Piece Red Left", onePieceRedHPBL4);
        autonChooserRed.addOption("2 Piece Red Right", twoPieceRedHHPBL4);
        autonChooserRed.addOption("2 Piece Red Left", twoPieceBlueHHPTL4);

        autonChooserBlue.addOption("1 Piece blue Left", onePieceBlueHPTL4);
        autonChooserBlue.addOption("1 Piece Blue Right", onePieceBlueHPBL4);
        autonChooserBlue.addOption("2 Piece Blue Left", twoPieceBlueHHPTL4);
        autonChooserBlue.addOption("2 Piece Blue Right", twoPieceBlueGHPBL4);
        autonChooserBlue.addOption("Drive By Left", driveByTopBlue);
        autonChooserBlue.addOption("2 Piece Drive By Top", twoPieceDriveByTopBlue);
        autonChooserBlue.addOption("Drive By Right", driveByBottomBlue);
        autonChooserBlue.addOption("2 Piece Drive By Right", twoPieceDriveByBottomBlue);
        autonChooserBlue.addOption("3 Piece Right", threePieceBlueBottomL4);


    }

    public static class OnePiece {

        public static class Blue {
            public static Command onePieceBlueHPBL4() {
                return onePieceBlueHPBL4;
            }

            public static Command onePieceBlueHPTL4() {
                return onePieceBlueHPTL4;
            }
        }

        public static class Red {
            public static Command onePieceRedHPBL4() {
                return onePieceRedHPBL4;
            }

            public static Command onePieceRedHPTL4() {
                return onePieceRedHPTL4;
            }

        }
    }


    public static class TwoPiece {
        public static class Blue {
            public static Command twoPieceBlueGBottomL4() {
                return twoPieceBlueGHPBL4;
            }

            public static Command twoPieceBlueHTopL4() {
                return twoPieceBlueHHPTL4;
            }

            public static Command twoPieceBlueDBottomL4() {
                return twoPieceBlueDHPBL4;
            }

            public static Command twoPieceBlueKTopL4() {
                return twoPieceBlueKHPTL4;
            }
        }

        public static class Red {
            public static Command twoPieceRedHBottomL4() {
                return twoPieceRedHHPBL4;
            }

            public static Command twoPieceRedGTopL4() {
                return twoPieceRedGHPTL4;
            }

            public static Command twoPieceRedKBottomL4() {
                return twoPieceRedKHPBL4;
            }

            public static Command twoPieceRedDTopL4() {
                return twoPieceRedDHPTL4;
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
