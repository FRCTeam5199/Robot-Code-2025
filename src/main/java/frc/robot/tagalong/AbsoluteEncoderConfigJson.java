package frc.robot.tagalong;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AbsoluteEncoderConfigJson {
  public boolean zeroToOne;
  public UnitJson magnetOffset;
  public boolean clockwisePositive;
  public CANcoderConfiguration cancoderConfiguration;
  public MagnetSensorConfigs magnetSensorConfigs;



  public double getAbsoluteDiscontinuityPoint() {
    return zeroToOne ? 1
                     : 0;
  }

  public SensorDirectionValue getSensorDirection() {
    return clockwisePositive ? SensorDirectionValue.Clockwise_Positive
                             : SensorDirectionValue.CounterClockwise_Positive;
  }

  public CANcoderConfiguration getCancoderConfig() {
    if (cancoderConfiguration == null) {
      MagnetSensorConfigs magnetSensorConfigs =
          new MagnetSensorConfigs()
              .withAbsoluteSensorDiscontinuityPoint(getAbsoluteDiscontinuityPoint())
              .withMagnetOffset(magnetOffset.getDistRotation().getRotations())
              .withSensorDirection(getSensorDirection());
      cancoderConfiguration = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    }
    return cancoderConfiguration;
  }
}
