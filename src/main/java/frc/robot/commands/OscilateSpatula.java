// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class OscilateSpatula extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Pneumatics m_subsystem;
  private double startTime;
  private double t;
  private boolean isOver;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OscilateSpatula(Pneumatics subsystem, double oscilateTime) {
    m_subsystem = subsystem;
    t = oscilateTime;
    isOver = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  public void stop() {
    isOver = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis()-startTime >= t) {
      m_subsystem.toggleSpatulaSolenoid();
      startTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOver;
  }
}
