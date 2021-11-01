// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//all the code in this has come from https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/part-2-driving.html

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final int L = 0;
  public static final int W = 0;
  public static final int MAX_VOLTS = 0;
  private WheelDrive backRight = new WheelDrive(0, 1, 0);
  private WheelDrive backLeft = new WheelDrive(2, 3, 1);
  private WheelDrive frontRight = new WheelDrive(4, 5, 2);
  private WheelDrive frontLeft = new WheelDrive(6, 7, 3);

  private SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public class swerveDrive {


    public void drive(double x1, double y1, double x2) {
      double r = Math.sqrt((L * L) + (W * W));
      y1 *= -1;

      double a = x1 - x2 * (L / r);
      double b = x1 + x2 * (L / r);
      double c = y1 - x2 * (W / r);
      double d = y1 + x2 * (W / r);

      double backRightSpeed = Math.sqrt((a * a) + (d * d));
      double backLeftSpeed = Math.sqrt((a * a) + (c * c));
      double frontRightSpeed = Math.sqrt((b * b) + (d * d));
      double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

      double backRightAngle = Math.atan2(a, d) / Math.PI;
      double backLeftAngle = Math.atan2(a, c) / Math.PI;
      double frontRightAngle = Math.atan2(b, d) / Math.PI;
      double frontLeftAngle = Math.atan2(b, c) / Math.PI;
    }
    private WheelDrive backRight;
private WheelDrive backLeft;
private WheelDrive frontRight;
private WheelDrive frontLeft;

public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
}
  }

  public class WheelDrive {

    private TalonSRX angleMotor;
    private TalonSRX speedMotor;
    private PIDController pidController;

    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
      this.angleMotor = new TalonSRX (angleMotor);
      this.speedMotor = new TalonSRX(speedMotor);
      pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), (PIDOutput) this.angleMotor);
  
      pidController.setOutputRange (-1, 1);
      pidController.setContinuous ();
      pidController.enable ();
    }
    public void drive (double speed, double angle) {
      speedMotor.set (ControlMode.PercentOutput, speed);
  
      double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
      if (setpoint < 0) {
          setpoint = MAX_VOLTS + setpoint;
      }
      if (setpoint > MAX_VOLTS) {
          setpoint = setpoint - MAX_VOLTS;
      }
  
      pidController.setSetpoint (setpoint);
  }

  }
  


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
