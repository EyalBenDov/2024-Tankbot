// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import utilities.IMUWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.lang.Math;

/** An example command that uses an example subsystem. */
public class MoveTime extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  private final double t;
  
  
  //position variables are measured in encoder ticks
  double startTime;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveTime(DriveTrain train, double timeToWait) {
    m_DriveTrain = train;
    t = timeToWait;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("INITIALIZED!!");
    startTime = System.currentTimeMillis();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_DriveTrain.setMotors();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AUTO SHOULD BE FINISHED");
    m_DriveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(System.currentTimeMillis()-startTime);
    return System.currentTimeMillis() - startTime >= t;
  }
}
