package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.tagalong.ArmParser;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.PivotParser;
import frc.robot.tagalong.TagalongPivot;
import frc.robot.tagalong.TagalongSubsystemBase;

public class ArmSubsystem extends TagalongSubsystemBase implements PivotAugment{
    private final TagalongPivot arm;
        private static ArmSubsystem armSubsystem;
        public final ArmParser armParser;




    public ArmSubsystem(String filePath) {
        this(filePath == null ? null : new ArmParser(Filesystem.getDeployDirectory(), filePath));
    }


    public ArmSubsystem(ArmParser parser){
        super(parser);
        armParser = parser;
        arm = new TagalongPivot(armParser.pivotParser);

        
    }


    public Command setGround(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.GROUND));
    }

    public Command setGroundBack(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.GROUND_2));
    }
    public Command setL1(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.L1));
    }
    public Command setL2(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.L2));
    }
    public Command setL3(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.L3));
    }
    public Command setL4(){
        return new InstantCommand(()->arm.setPivotProfile(ArmConstants.L4));
    }


    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem("configs/subsystems/armConf");
        }
        return armSubsystem;
    }



    @Override
    public TagalongPivot getPivot() {
        return arm;
    }


    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }




    
}