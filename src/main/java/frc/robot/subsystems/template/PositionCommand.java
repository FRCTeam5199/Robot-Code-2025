package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj2.command.Command;

public class PositionCommand extends Command {
    private double goal;
    private TemplateSubsystem templateSubsystem;
    private boolean updateGoalPosition;

    public PositionCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateGoalPosition = false;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        templateSubsystem.setPosition(goal, false);
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
