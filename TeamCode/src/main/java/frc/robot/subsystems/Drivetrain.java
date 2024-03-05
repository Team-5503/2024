package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  CANSparkMax leftFront;
  CANSparkMax leftRear;
  CANSparkMax rightFront;
  CANSparkMax rightRear;

  public Drivetrain() {
    // Initialize device IDs
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftRear = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(3, MotorType.kBrushless);
    rightRear = new CANSparkMax(4, MotorType.kBrushless);

    // Set idle mode
    leftFront.setIdleMode(IdleMode.kBrake);
    leftRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);

    // Set master and slave controllers
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
  }

  @Override
  public void periodic() {
  }

  public void drive (double X, double Y) {
    leftFront.set(X);
    rightFront.set(Y);
  }

  public void stop () {
    leftFront.stopMotor();
    rightFront.stopMotor();
  }
}
