/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

    private static PathPlannerAuto onePieceBlueDropL1;
    private static PathPlannerAuto onePieceRedDropL1;

    private static PathPlannerAuto twoPieceBlueBottomL4;
    private static PathPlannerAuto twoPieceBlueTopL4;

    private static PathPlannerAuto twoPieceRedBottomL4;
    private static PathPlannerAuto twoPieceRedTopL4;

    private static PathPlannerAuto testBlue;

    private static PathPlannerAuto threePieceBlueBottomL4;
    private static PathPlannerAuto threePieceBlueTopL4;

    private static PathPlannerAuto threePieceRedBottomL4;
    private static PathPlannerAuto threePieceRedTopL4;

    private static PathPlannerAuto helper2PieceTopBlue;
    private static PathPlannerAuto helper2PieceTopRed;

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

    //Blue - Top is Left, Bottom is Right
    //Red - Top is Right, Bottom is Left
    public static void initializeAutos() {
        onePieceBlueDropL1 = new PathPlannerAuto("Blue Climber Drop");
        onePieceRedDropL1 = new PathPlannerAuto("Red Climber Drop");

        helper2PieceTopBlue = new PathPlannerAuto("2 Piece Blue Top L4 Helper");
        helper2PieceTopRed = new PathPlannerAuto("2 Piece Red Top L4 Helper");

        twoPieceBlueBottomL4 = new PathPlannerAuto("2 Piece Blue Bottom L4");
        twoPieceBlueTopL4 = new PathPlannerAuto("2 Piece Blue Top L4");
        twoPieceRedBottomL4 = new PathPlannerAuto("2 Piece Red Bottom L4");
        twoPieceRedTopL4 = new PathPlannerAuto("2 Piece Red Top L4");

        threePieceBlueBottomL4 = new PathPlannerAuto("3 Piece Blue Bottom L4");
        threePieceBlueTopL4 = new PathPlannerAuto("3 Piece Blue Top L4");
        threePieceRedBottomL4 = new PathPlannerAuto("3 Piece Red Bottom L4");
        threePieceRedTopL4 = new PathPlannerAuto("3 Piece Red Top L4");

        testBlue = new PathPlannerAuto("Test Blue");

        Shuffleboard.getTab("Autons").add("Red Autons", autonChooserRed)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);
        Shuffleboard.getTab("Autons").add("Blue Autons", autonChooserBlue)
                .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0)
                .withSize(2, 1);

        autonChooserRed.addOption("1 Piece Red Climber Drop", onePieceRedDropL1);
        autonChooserRed.addOption("2 Piece Red Left", twoPieceRedBottomL4);
        autonChooserRed.addOption("2 Piece Red Right", twoPieceRedTopL4);
        autonChooserRed.addOption("3 Piece Red Left", threePieceRedBottomL4);
        autonChooserRed.addOption("3 Piece Red Right", threePieceRedTopL4);
        autonChooserRed.addOption("2 Piece Red Left Helper", helper2PieceTopRed);

        autonChooserBlue.addOption("1 Piece Blue Climber Drop", onePieceBlueDropL1);
        autonChooserBlue.addOption("2 Piece Blue Left", twoPieceBlueTopL4);
        autonChooserBlue.addOption("2 Piece Blue Right", twoPieceBlueBottomL4);
        autonChooserBlue.addOption("3 Piece Blue Left", threePieceBlueTopL4);
        autonChooserBlue.addOption("3 Piece Blue Right", threePieceBlueBottomL4);
        autonChooserBlue.addOption("2 Piece Blue Left Helper", helper2PieceTopBlue);
    }
}
