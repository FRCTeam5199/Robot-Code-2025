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

    //Used for if the velocity/acceleration constraint needs to be changed
    public PositionCommand(TemplateSubsystem templateSubsystem, double goal, boolean isGoingUp) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;
        if (isGoingUp) { //100, 150
            //Up
            this.vel = 75;
            this.acc = 150;
        } else { //80,100
            //Down
            this.vel = 50;
            this.acc = 100;
        }
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
