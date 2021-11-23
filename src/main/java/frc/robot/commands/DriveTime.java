// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {
  private final double m_time;
  private final double m_speed;
  private final DriveSubsystem m_drive;
  private final Timer m_timer;

  /** Creates a new DriveTime. */
  public DriveTime(double time, double speed, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_time = time;
    m_speed = speed;
    m_drive = drive;
    m_timer = new Timer();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.stopDrive();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_time);
  }
}
