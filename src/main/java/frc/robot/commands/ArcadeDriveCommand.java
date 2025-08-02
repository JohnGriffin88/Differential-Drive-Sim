// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDriveCommand extends Command {

  // TODO: Insert your class variables here...
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;

  /** Creates a new ArcadeDriveCommand. */
  // TODO: Insert your consutrctor parameters
  public ArcadeDriveCommand(DriveSubsystem drive, CommandXboxController controller) {
    // TODO: Insert your constructor code here...
      m_drive = drive;
      m_controller = controller;
      addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Insert your command code here...
    m_drive.arcadeDrive(m_controller.getLeftX(), m_controller.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
