package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj2.command.Command;

public class VelocityCommand extends Command {
    private double goal;
    private TemplateSubsystem templateSubsystem;
    private boolean updateVelocity;

    public VelocityCommand(TemplateSubsystem templateSubsystem, double goal) {
        this.templateSubsystem = templateSubsystem;
        this.goal = goal;
        updateVelocity = false;

        addRequirements(templateSubsystem);
    }

    @Override
    public void initialize() {
        templateSubsystem.setVelocity(goal);
    }

    @Override
    public void execute() {
        if (updateVelocity) {
            templateSubsystem.setVelocity(goal);
            updateVelocity = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        templateSubsystem.setPercent(0);
    }

    public void setGoal(double goal) {
        this.goal = goal;
        updateVelocity = true;
    }
}
