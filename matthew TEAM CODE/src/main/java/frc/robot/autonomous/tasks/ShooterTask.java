package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Shooter;

public class ShooterTask extends Task {
  private Shooter m_shooter;
  private double m_speed;

  public ShooterTask() {
    m_shooter = Shooter.getInstance();
  }

  @Override
  public void start() {
    m_shooter.setSpeaker();;
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
