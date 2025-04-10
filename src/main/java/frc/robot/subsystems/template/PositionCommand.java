package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj2.command.Command;

public class PositionCommand extends Command {
    private double vel;
    private double acc;
    private double goal;
    private TemplateSubsystem templateSubsystem;
    private boolean updateGoalPosition;
    private boolean changeConstraint;

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        changeConstraint = false;

        addRequirements(templateSubsystem);
    }

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal, double vel, double acc) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;

        this.vel = vel;
        this.acc = acc;

        changeConstraint = true;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        if (changeConstraint) {
            templateSubsystem.setPosition(goal, false, vel, acc);
        } else {
            templateSubsystem.setPosition(goal, false);
        }
    }

    @Override
    public void execute() {
        templateSubsystem.followLastMechProfile();

        if (updateGoalPosition) {
            templateSubsystem.setPosition(goal, false);
            updateGoalPosition = false;
        }
    }


    @Override
    public boolean isFinished() {
        return templateSubsystem.isMechAtGoal(false);
    }

    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setFollowLastMechProfile(true);
    }

    public void setGoal(double goal) {
        this.goal = goal;
        updateGoalPosition = true;
    }
}
