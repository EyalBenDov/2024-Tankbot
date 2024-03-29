// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // private Victor FL = new Victor(4);
  // private Victor BL = new Victor(3);
  // private Victor FR = new Victor(2);
  // private Victor BR = new Victor(1);
  PIDController distancePIDController = new PIDController(Constants.velocityPIDkP, Constants.velocityPIDkI, Constants.velocityPIDkD);

  private CANSparkMax FL = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(3, MotorType.kBrushless);

  private CANSparkMax FR = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(1, MotorType.kBrushless);

  private RelativeEncoder lEncoder;
  private RelativeEncoder rEncoder;

  private MotorControllerGroup m_left = new MotorControllerGroup(FL, BL);
  private MotorControllerGroup m_right = new MotorControllerGroup(FR, BR);
  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  private int accelFactor = 0;
  private double accelVal = 0.05;
  public DriverStation.Alliance alliance;

  private final ADIS16448_IMU m_gyro;
  DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.FFkS, Constants.FFkV, Constants.FFkA);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  alliance = DriverStation.getAlliance().get();      
    
  // FL.setInverted(true);
  // BL.setInverted(true);
  lEncoder = FL.getEncoder();
  rEncoder = BR.getEncoder();
    m_pose = new Pose2d();
    m_gyro = new ADIS16448_IMU();
    resetEncoders();
    m_gyro.calibrate();
    
  // Creating my odometry object. Here,
  // our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing forward.
  m_odometry = new DifferentialDriveOdometry(
  new Rotation2d(m_gyro.getGyroAngleZ()*180.0/Math.PI),
  getDistance(lEncoder), getDistance(rEncoder)/*,
  new Pose2d(5.0, 13.5, new Rotation2d())*/);
  }

  private double getDistance(RelativeEncoder encoder) {
    return (encoder.getPosition()*Math.PI*2*frc.robot.Constants.wheelRadius/frc.robot.Constants.driveTrainGearRatio);
  }
  private double getVelocity(RelativeEncoder encoder) {
    return (encoder.getVelocity()*Math.PI*2*frc.robot.Constants.wheelRadius/frc.robot.Constants.driveTrainGearRatio);
  }

  public double getLDistance() {
    return getDistance(lEncoder);
  }
  public double getRDistance() {
    return getDistance(rEncoder);
  }

  public void stopMotors() {
    FL.set(0);
    BL.set(0);
    FR.set(0);
    BR.set(0);

    FL.setIdleMode(IdleMode.kCoast);
    BL.setIdleMode(IdleMode.kCoast);
    FR.setIdleMode(IdleMode.kCoast);
    BR.setIdleMode(IdleMode.kCoast);

    FL.burnFlash();
    BL.burnFlash();
    FR.burnFlash();
    BR.burnFlash();
  }



  public void resetEncoders()
  {
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);
  }

  public void upFactor() {
    accelFactor = Math.min(3, accelFactor + 1);
    System.out.println("Speed factor is now: " + accelFactor);
  }

  public void downFactor() {
    accelFactor = Math.max(0, accelFactor - 1);
    System.out.println("Speed factor is now: " + accelFactor);
  }

  public void setFL() {
    // System.out.println("front left motor toggled");
    if (FL.get() > 0.2) {
      FL.set(0);
    }
    else {
      FL.set(-0.3);
    }
  }
  public void setFR() {
    // System.out.println("front right motor toggled");
    if (FR.get() > 0.2) {
      FR.set(0);
    }
    else {
      FR.set(0.3);
    }
  }
  public void setBL() {
    // System.out.println("back left motor toggled");
    if (BL.get() > 0.2) {
      BL.set(0);
    }
    else {
      BL.set(-0.3);
    }
  }
  public void setMotors() {
    FL.setIdleMode(IdleMode.kBrake);
    BL.setIdleMode(IdleMode.kBrake);
    FR.setIdleMode(IdleMode.kBrake);
    BR.setIdleMode(IdleMode.kBrake);

    FL.burnFlash();
    BL.burnFlash();
    FR.burnFlash();
    BR.burnFlash();

    if (alliance == DriverStation.Alliance.Red) {
      FL.set(0.3);
      BL.set(0.3);
      BR.set(-0.3);
      FR.set(-0.3);
    } else {
      FL.set(-0.3);
      BL.set(-0.3);
      BR.set(0.3);
      FR.set(0.3);
    }
  }
  public void setBR() {
    System.out.println("back right motor toggled");
    if (BR.get() > 0.2) {
      BR.set(0);
    }
    else {
      BR.set(0.3);
    }
  }

  public void drive(double l, double r) {
    var lDistance = getDistance(lEncoder);
    var rDistance = getDistance(rEncoder);
    FL.set(distancePIDController.calculate(lDistance, l));
    FR.set(distancePIDController.calculate(rDistance, r));
    BL.set(distancePIDController.calculate(lDistance, l));
    BR.set(distancePIDController.calculate(rDistance, r));
  }

  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.maxWheelSpeed);
    m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond/Constants.maxWheelSpeed, wheelSpeeds.rightMetersPerSecond/Constants.maxWheelSpeed);
    System.out.println("right encoder: " + getVelocity(rEncoder) + " left encoder: " + getVelocity(lEncoder));
  }

  //always relative to the initial calibration

  public void setMotorsBasedOnZAngle(int targetAngle, double currentAngle) {
    // System.out.println("current angle: " + currentAngle);
    if (Math.abs(currentAngle - targetAngle) > 5) {
      // target is to the right
      if (targetAngle - currentAngle < 0) {
        doDrive(0.25, -.25);
      }
      else {
        doDrive(-.25, .25);
      }
    }

  }

  public double getPosition()
  {
    return (-lEncoder.getPosition() + rEncoder.getPosition()) / 2 / Constants.driveTrainGearRatio;
  }

  public void doDrive(double lThrottle, double rThrottle, boolean is_teleop) {
    
      if (accelFactor > 0 && Math.abs(lThrottle) > 0 && Math.abs(rThrottle) > 0) {
        boolean lneg = lThrottle < 0;
        boolean rneg = rThrottle < 0;  

        lThrottle = Math.pow(2, Math.pow(Math.abs(lThrottle), accelFactor)) - 1;
        rThrottle = Math.pow(2, Math.pow(Math.abs(rThrottle), accelFactor)) - 1;

        if (lneg) { lThrottle *= -1; }
        if (rneg) { rThrottle *= -1; }
      }
      if (alliance == DriverStation.Alliance.Red) {
        m_drive.tankDrive(-lThrottle, rThrottle);
      } else {
        m_drive.tankDrive(rThrottle, -lThrottle);
      }
      // System.out.println("Positions: " + lEncoder.getPosition()+ ", " + -rEncoder.getPosition());
      // System.out.println(String.format("I am tank driving with a lThrottle of %s and a rThrottle of %s", lThrottle, rThrottle));
  }
  public void doDrive(double lThrottle, double rThrottle) {

    // System.out.println("doDrive called!");
    doDrive(lThrottle, rThrottle, false);
  }

  public Pose2d getPose() {
    return m_pose; 
  }

  public void resetPose(Pose2d newPose2d) {
    m_odometry.resetPosition(new Rotation2d(m_gyro.getGyroAngleZ()*180.0/Math.PI), new DifferentialDriveWheelPositions(getDistance(lEncoder), getDistance(rEncoder)), newPose2d);
  }
  @Override
  public void periodic() {
    // Get the rotation of the robot from the gyro.
    var gyroAngle = new Rotation2d(m_gyro.getGyroAngleZ()*180.0/Math.PI);

    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
    getDistance(lEncoder),
    getDistance(rEncoder));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
