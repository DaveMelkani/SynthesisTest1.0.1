/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import sun.security.jca.GetInstance;
import frc.robot.OI;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  //private static DriveTrain instance;
  
  /*
  public enum DrivetrainSide {
    left, right;
  }
  */

  private PWMTalonSRX left = new PWMTalonSRX(RobotMap.leftDrivePort);
  private PWMTalonSRX right = new PWMTalonSRX(RobotMap.rightDrivePort);
  private PWMTalonSRX lift = new PWMTalonSRX(RobotMap.liftPort);
  private PWMTalonSRX arm = new PWMTalonSRX(RobotMap.armPort);
  
  private static DriveTrain drive;
  //private boolean inverted = false;

 // private double leftPowerY, rightPowerY;
 // private double leftPowerX, rightPowerX;

  private double leftPower, rightPower;
  private boolean downArm, upArm;
  private boolean downLift, upLift;
  private boolean CGArm;

  //public Joystick arcJoy;

  public DriveTrain() {
    left.setInverted(false);
    right.setInverted(true);
    arm.setInverted(false);

    //leftEncoder.reset();
    //rightEncoder.reset();
    //leftEncoder.setDistancePerPulse(0.05);
    //rightEncoder.setDistancePerPulse(0.05);
    
    
    
    //leftEncoder.reset();
    //rightEncoder.reset();
    //gyro.reset();
  }

  public static DriveTrain getInstance() {
    if(drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  public void tankDrive(double leftPow, double rightPow) {
  /* 
    if (leftPow < 0.05 && leftPow > -0.05) {
      leftPow = 0;
    }
    if (rightPow < 0.05 && rightPow > -0.05) {
      rightPow = 0;
    }
*/
    left.set(leftPow);
    right.set(rightPow);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  /*
  public double returnDistance(){
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = leftEncoder.getDistance();
    return (leftDistance + rightDistance)/2;
  }

  public void printDistance(){
    System.out.println(returnDistance());
  }
*/
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
   // leftPowerY = OI.getArcJoy().getY();
   // rightPowerY = OI.getArcJoy().getY();
   // leftPowerX = OI.getArcJoy().getX();
   // rightPowerX = OI.getArcJoy().getX();
    downArm = OI.getArcJoy().getRawButton(3);
    upArm = OI.getArcJoy().getRawButton(4);
    downLift = OI.getArcJoy().getRawButton(5);
    upLift = OI.getArcJoy().getRawButton(6);
    CGArm = OI.getArcJoy().getRawButton(10);

    if (CGArm) {
      arm.set(0.2);
      lift.set(0.2);
    } else {
      arm.set(-0.2);
      lift.set(-0.2);
    }
    

    if (downArm) {
      arm.set(-0.2);
    } else if (upArm) {
      arm.set(0.2);
    } else {
      arm.set(0);
    }

    if(downLift) {
      lift.set(-0.2);
    } else if (upLift) {
      lift.set(0.2);
    } else {
      arm.set(0);
    }


      leftPower = OI.getArcJoy().getY();
      rightPower = OI.getArcJoy().getY();
    if (OI.getArcJoy().getX() < -0.2) {
      leftPower += -0.5;
      rightPower += 0.5;
    } else if (OI.getArcJoy().getX() > 0.2) {
      leftPower += 0.5;
      rightPower += -0.5;
    }

    tankDrive(leftPower * 0.3, rightPower * 0.3);


  }
}
