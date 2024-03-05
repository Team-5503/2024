package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private final Intake intake;
  public RunIntake(Intake i) {
    intake = i;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.getSwitchStatus();
    
    if (intake.intakeSwitchStatus == true) {
      intake.run(-0.5);
    } else {
      intake.run(0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
