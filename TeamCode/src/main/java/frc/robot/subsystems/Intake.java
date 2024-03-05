package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public boolean intakeSwitchStatus;
  CANSparkMax intake;
  DigitalInput intakeLimitSwitch;
  public Intake() {

    // Set up device IDs
    intake = new CANSparkMax(3, MotorType.kBrushless); // CAN ID 3
    intake.setSoftLimit(SoftLimitDirection.kForward, 1);
    intake.setSoftLimit(SoftLimitDirection.kReverse, -1);

    intakeLimitSwitch = new DigitalInput(0); // ID 0 on DIO rail
  }

  @Override
  public void periodic() {}

  public void run (double speed) {
    intake.set(speed);
  }

  public void stop () {
    intake.stopMotor();
  }

  public void getSwitchStatus () {
    intakeSwitchStatus = intakeLimitSwitch.get();
  }
}
