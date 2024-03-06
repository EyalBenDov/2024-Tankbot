// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveTime;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_dDriveTrain = new DriveTrain();
  // private final Joystick joystick1 = new Joystick(0);
  private final CommandJoystick joystick1 = new CommandJoystick(0);

  private final Trigger liftSolenoid = joystick1.button(6);
  private final Pneumatics m_pneumatics = new Pneumatics();

  // private Command TurnToNorth = new TurnToNorth(m_dDriveTrain);
  // private Command TurnToSouth = new TurnToSouth(m_dDriveTrain);
  // private Command TurnToEast = new TurnToEast(m_dDriveTrain);
  // private Command TurnToest = new TurnToWest(m_dDriveTrain);



  private Trigger upAccel = joystick1.button(8);
  private Trigger downAccel = joystick1.button(7);

  private Trigger toggleFL = joystick1.button(1);
  private Trigger toggleBL = joystick1.button(2);
  private Trigger toggleFR = joystick1.button(3);
  private Trigger toggleBR = joystick1.button(4);

  DriverStation.Alliance m_alliance;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // DriverStation.Alliance alliance = DriverStation.getAlliance();
    m_alliance = DriverStation.getAlliance().get();
    m_pneumatics.forwardLiftSolenoid();
    // Configure the button bindings
    configureButtonBindings();

    // CameraServer.startAutomaticCapture();
    if (m_alliance == DriverStation.Alliance.Blue) {
      m_dDriveTrain.setDefaultCommand(

        new RunCommand(
          () -> 
          m_dDriveTrain.doDrive(
            joystick1.getRawAxis(XboxController.Axis.kLeftY.value),
            joystick1.getRawAxis(XboxController.Axis.kRightY.value),
            true
            ),
        m_dDriveTrain)
      );
    } else {
      m_dDriveTrain.setDefaultCommand(

        new RunCommand(
          () -> 
          m_dDriveTrain.doDrive(
            -1*joystick1.getRawAxis(XboxController.Axis.kLeftY.value),
            -1*joystick1.getRawAxis(XboxController.Axis.kRightY.value),
            true
            ),
        m_dDriveTrain)
      );
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    liftSolenoid.onTrue(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    liftSolenoid.onFalse(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    // turnToNorth.whenPressed(TurnToNorth);
    // turnToEast.whenPressed(TurnToEast);
    // turnToSouth.whenPressed(TurnToSouth);
    // turnToWest.whenPressed(TurnToWest);
    
    // toggleBL.onTrue(new RunCommand(() -> {
    //   m_dDriveTrain.setBL();
    // }, m_dDriveTrain));
    // toggleBR.onTrue(new RunCommand(() -> {
    //   m_dDriveTrain.setBR();
    // }, m_dDriveTrain));
    // toggleFL.onTrue(new RunCommand(() -> {
    //   m_dDriveTrain.setFL();
    // }, m_dDriveTrain));
    // toggleFR.onTrue(new RunCommand(() -> {
    //   m_dDriveTrain.setFR();
    // }, m_dDriveTrain));

    upAccel.onTrue(new InstantCommand(() -> {
      m_dDriveTrain.upFactor();
    }));
    downAccel.onTrue(new InstantCommand(() -> {
      m_dDriveTrain.downFactor();
    }));

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new AutonomousCommandGroup(m_dDriveTrain);
    // System.out.println("Auto called!");

    return new MoveTime(m_dDriveTrain);
  }
}