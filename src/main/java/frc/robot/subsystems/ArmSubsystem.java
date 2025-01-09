package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.PivotParser;
import frc.robot.tagalong.TagalongPivot;
import frc.robot.tagalong.TagalongSubsystemBase;

public class ArmSubsystem extends TagalongSubsystemBase implements PivotAugment{
    private final TagalongPivot arm;


    public ArmSubsystem(String filePath) {
        this(filePath == null ? null : new ArmParser(Filesystem.getDeployDirectory(), filePath));
    }


    public ArmSubsystem(){
        super(ArmParser arm);
    }


    @Override
    public TagalongPivot getPivot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }


    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }

}