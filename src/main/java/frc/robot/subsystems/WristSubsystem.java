package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.tagalong.ArmParser;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.PivotParser;
import frc.robot.tagalong.TagalongPivot;
import frc.robot.tagalong.TagalongSubsystemBase;
import frc.robot.tagalong.WristParser;
import frc.robot.tagalong.WristParser;

public class WristSubsystem extends TagalongSubsystemBase implements PivotAugment{
    private final TagalongPivot wrist;
        private static WristSubsystem wristSubsystem;
        public final WristParser wristParser;




    public WristSubsystem(String filePath) {
        this(filePath == null ? null : new WristParser(Filesystem.getDeployDirectory(), filePath));
    }


    public WristSubsystem(WristParser parser){
        super(parser);
        wristParser = parser;
        wrist = new TagalongPivot(wristParser.pivotParser);

        
    }


    public Command setGround(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.GROUND));
    }

    public Command setGroundBack(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.GROUND_2));
    }
    public Command setL1(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L1));
    }
    public Command setL2(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L2));
    }
    public Command setL3(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L3));
    }
    public Command setL4(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L4));
    }


    public static WristSubsystem getInstance() {
        if (wristSubsystem == null) {
            wristSubsystem = new WristSubsystem("configs/subsystems/wristConf");
        }
        return wristSubsystem;
    }



    @Override
    public TagalongPivot getPivot() {
        return wrist;
    }


    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }




    
}