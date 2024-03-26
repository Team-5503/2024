package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.leds.LEDs;

public class Intake extends Subsystem {

  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);

  public final LEDs m_leds = LEDs.getInstance();

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkMax mIntakeMotor;

  private Intake() {
    super("Intake");

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mIntakeMotor.setSmartCurrentLimit(40);


    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    putString("State", m_periodicIO.intake_state.toString());
  }

    
    

  @Override
  public void writePeriodicOutputs() {

    // mIntakeMotor.set(m_periodicIO.intake_speed);
    mIntakeMotor.set(m_periodicIO.intake_speed);

  }

  @Override
  public void stop() {
  
    m_periodicIO.intake_speed = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", intakeStateToSpeed(m_periodicIO.intake_state));

    putBoolean("Limit Switch", getIntakeHasNote());
  }

  @Override
  public void reset() {
  }

  

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return .35;
      case EJECT:
        return -.7;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return .1;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !m_IntakeLimitSwitch.get();
  }


  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }
  public IntakeState getState(){
    return m_periodicIO.intake_state;
  }


  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    
  }

  
}
