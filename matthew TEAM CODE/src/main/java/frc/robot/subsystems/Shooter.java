package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private CANSparkMax mLeftShooterMotor;
  private CANSparkMax mRightShooterMotor;

  private SparkPIDController mLeftShooterPID;
  private SparkPIDController mRightShooterPID;

  private RelativeEncoder mLeftShooterEncoder;
  private RelativeEncoder mRightShooterEncoder;

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1);

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);
    mLeftShooterMotor.restoreFactoryDefaults();
    mRightShooterMotor.restoreFactoryDefaults();
    mLeftShooterMotor.setSmartCurrentLimit(75);
    mRightShooterMotor.setSmartCurrentLimit(75);

    mLeftShooterPID = mLeftShooterMotor.getPIDController();
    mLeftShooterPID.setP(Constants.kShooterP);
    mLeftShooterPID.setI(Constants.kShooterI);
    mLeftShooterPID.setD(Constants.kShooterD);
    mLeftShooterPID.setFF(Constants.kShooterFF);
    mLeftShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    mRightShooterPID = mRightShooterMotor.getPIDController();
    mRightShooterPID.setP(Constants.kShooterP);
    mRightShooterPID.setI(Constants.kShooterI);
    mRightShooterPID.setD(Constants.kShooterD);
    mRightShooterPID.setFF(Constants.kShooterFF);
    mRightShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    mLeftShooterEncoder = mLeftShooterMotor.getEncoder();
    mRightShooterEncoder = mRightShooterMotor.getEncoder();

    mLeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(true);
    mRightShooterMotor.setInverted(false);

  }

  private static class PeriodicIO {
    ShooterTarget aim = ShooterTarget.SPEAKER;
    double shooter_rpm = 0;
  }

  public enum ShooterTarget {
    SPEAKER,
    AMP, 
    NONE
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.shooter_rpm = ShooterStateToSpeed(mPeriodicIO.aim);
  }

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mSpeedLimiter.calculate(mPeriodicIO.shooter_rpm);
    mLeftShooterMotor.set(limitedSpeed);
    mRightShooterMotor.set(limitedSpeed - ShooterStateToSpin(mPeriodicIO.aim));
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", mLeftShooterEncoder.getVelocity());
    putNumber("Right speed:", mRightShooterEncoder.getVelocity());
  }

  @Override
  public void reset() {
  }

  public double ShooterStateToSpeed(ShooterTarget state) {
    switch (state) {
      case SPEAKER:
        return .65;
      
      case AMP:
        return .17;
      case NONE:
        return 0;
      default:
        // "Safe" default
        return 0.0;
    }
  }
  public double ShooterStateToSpin(ShooterTarget state) {
    switch (state) {
      case SPEAKER:
        return .05;
      
      case AMP:
        return .03;
      case NONE:
        return 0;
      default:
        // "Safe" default
        return 0.0;
    }
  }
  /*---------------------------------- Custom Public Functions ----------------------------------*/


  public void setAmp(){
    mPeriodicIO.aim = ShooterTarget.AMP;
  }
  public void setSpeaker(){
    mPeriodicIO.aim = ShooterTarget.SPEAKER;
  }
  public void setNone(){
      mPeriodicIO.aim = ShooterTarget.NONE;
    }
  public void setAim(){
    if (mPeriodicIO.aim != ShooterTarget.NONE){
    if (mPeriodicIO.aim == ShooterTarget.SPEAKER)
      mPeriodicIO.aim = ShooterTarget.AMP;
    else 
      mPeriodicIO.aim = ShooterTarget.SPEAKER;
    }
  }
  
  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }
  public double getRPM(){
    return mPeriodicIO.shooter_rpm;
  }
  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
