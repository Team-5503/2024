package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkRelativeEncoder;

// import edu.wpi.first.math.util.Units;

// import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
// import frc.robot.Helpers;
// import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.subsystems.leds.LEDs;




public class Pivot extends Subsystem {
  private static Pivot mInstance;
  private PeriodicIO m_periodicIO;
  private CANSparkMax mArm;


  public static Pivot getInstance() {
    if (mInstance == null) {
      mInstance = new Pivot();
    }
    return mInstance;
  }

  
  /** Creates a new Pivot. */
  private Pivot() {
    super("Pivot");

    mArm = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mArm.restoreFactoryDefaults();
    mArm.setSmartCurrentLimit(45);
    mArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    mArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    mArm.setSoftLimit(SoftLimitDirection.kForward, -3);
    mArm.setSoftLimit(SoftLimitDirection.kReverse, -11);


    
    
    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    PivotState pivot_state = PivotState.STOW;


    // Output: Motor set values
    double intake_pivot_percent = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    // SOURCE,
    AMP,
    STOW
  }
  public enum PivotState {
    NONE,
    GROUND,
    // SOURCE,
    AMP,
    STOW
  }

  @Override
  public void periodic() {
    m_periodicIO.intake_pivot_percent = pivotTargetToPer(m_periodicIO.pivot_target);
  }

  @Override
  public void reset() {
  }

  @Override
  public void writePeriodicOutputs() {
    mArm.set(m_periodicIO.intake_pivot_percent);
  }

  @Override
  public void stop() {
    m_periodicIO.intake_pivot_percent = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putString("Pivot target", m_periodicIO.pivot_target.toString());
  }

  private double pivotTargetToPer(PivotTarget target) {
    switch (target) {
      case GROUND:
      m_periodicIO.pivot_state = PivotState.GROUND;
        return -0.25;
      // case SOURCE:
      //   return Constants.Intake.k_pivotAngleSource;
      
      case STOW:
      m_periodicIO.pivot_state = PivotState.STOW;
        return 0.23;
      default:
        // "Safe" default
        return 0;
    }
  }
  public void setStow(){
    m_periodicIO.pivot_target = PivotTarget.STOW;
  }
  
  public void setGround(){
    m_periodicIO.pivot_target = PivotTarget.GROUND;
  }

  public void setAmp(){
    m_periodicIO.pivot_target = PivotTarget.AMP;
    // if (m_periodicIO.pivot_target == PivotTarget.GROUND){
    //   mArm.setSoftLimit(SoftLimitDirection.kForward, -6);
    //   m_periodicIO.pivot_target = PivotTarget.STOW;
    // }
    // if (m_periodicIO.pivot_target == PivotTarget.GROUND){
    //   mArm.setSoftLimit(SoftLimitDirection.kReverse, -6);
    //   m_periodicIO.pivot_target = PivotTarget.GROUND;
    // }
    }

  public void setNone(){
    m_periodicIO.pivot_target = PivotTarget.NONE;
  }
  public void setTarget(PivotTarget target){
    m_periodicIO.pivot_target = target;
  }
  public PivotTarget getTarget(){
    return m_periodicIO.pivot_target;
  }
}
