package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Pivot.PivotTarget;

public class IntakeTask extends Task {
  private Intake m_intake;
  public Pivot m_arm;
  private IntakeState m_intakeState;
  private PivotTarget m_pivotTarget;

  public IntakeTask(IntakeState intakeState, PivotTarget target) {
    m_intake = Intake.getInstance();
    m_arm = Pivot.getInstance();
    m_intakeState = intakeState;
    m_pivotTarget = target;
  }

  @Override
  public void start() {
    m_intake.setState(m_intakeState);
    m_arm.setTarget(m_pivotTarget);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
