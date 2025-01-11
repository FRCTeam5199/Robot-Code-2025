package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.tagalong.ArmParser;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.PivotParser;
import frc.robot.tagalong.TagalongPivot;
import frc.robot.tagalong.TagalongSubsystemBase;
import edu.wpi.first.networktables.GenericEntry;


public class ArmSubsystem extends TagalongSubsystemBase implements PivotAugment{
    private final TagalongPivot arm;
        private static ArmSubsystem armSubsystem;
        public final ArmParser armParser;
        private VoltageOut armTest;
        double voltageKg;
        GenericEntry voltage;



    public ArmSubsystem(String filePath) {
        this(filePath == null ? null : new ArmParser(Filesystem.getDeployDirectory(), filePath));
    }


    public ArmSubsystem(ArmParser parser){
        super(parser);
        armParser = parser;
        arm = new TagalongPivot(armParser.pivotParser);
        voltageKg = getArmVolt();





        
    
    }

    @Override
        voltageKg = getArmVolt();

        armTest.withOutput(voltageKg);

        System.out.println(arm.getVoltage());

    }

    public float getArmVolt() {
        if (voltage != null) {
            return voltage.getFloat(0);
        }
        return 0;
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


    //     public final SysIdRoutine m_sysIdRoutineArm = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(1), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSArm_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(volts)),
    //         null,
    //         this
    //     )
    // );






    
}