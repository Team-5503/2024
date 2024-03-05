package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class Drive extends Command {
  private final Drivetrain drivetrain;
  /** Creates a new Drive. */
  public Drive(Drivetrain dt) {
    drivetrain = dt;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.drive(RobotContainer.controller1.getRawAxis(0) * 1, RobotContainer.controller1.getRawAxis(1) * 1);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
