package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final Drivetrain drivetrain;
  private final Drive drive;

  Command autonomousCommand;
  public static CommandXboxController controller1 =
      new CommandXboxController(OperatorConstants.controller1Port);

  public RobotContainer() {
    drivetrain = new Drivetrain();
    drive = new Drive(drivetrain);
    drivetrain.setDefaultCommand(drive);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
