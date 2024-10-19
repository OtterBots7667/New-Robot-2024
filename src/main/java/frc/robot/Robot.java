// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String doNothing = "Do Nothing";
  private static final String red2Note = "Red 2 Note";
  // private static final String red1Note = "Red 1 Note";
  private static final String driveForward= "Drive Forward Until A-Stop is Hit";
  private static final String blue2Note = "Blue 2 Note";
  // private static final String blue1Note = "Blue1 Note";
  // private static final String red3Note = "Red 3 Note";
  // private static final String blue3Note = "Blue 3 Note";
  // private static final String redPushMiddle3 = "Red 3 Push Middle";
  // private static final String bluePushMiddle3 = "Blue 3 Push Middle";
  // private static final String redPushMiddle2 = "Red 2 Push Middle";
  // private static final String bluePushMiddle2 = "Blue 2 Push Middle";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


private Joystick joyDrive = new Joystick(0);
private Joystick joyButtons = new Joystick(1);
  
private TalonFX rightArm = new TalonFX(3);
private TalonFX leftArm = new TalonFX(2);
private TalonSRX climbLatch = new TalonSRX(10);
private TalonSRX intake = new TalonSRX(4);
private TalonSRX outtake = new TalonSRX(5);

private CANSparkMax leftDrive1 = new CANSparkMax(8, MotorType.kBrushless);
private CANSparkMax leftDrive2 = new CANSparkMax(9, MotorType.kBrushless);
private CANSparkMax rightDrive1 = new CANSparkMax(7, MotorType.kBrushless);
private CANSparkMax rightDrive2 = new CANSparkMax(6, MotorType.kBrushless);


private PIDController myPID = new PIDController(0, 0, 0);



Double driveSpeedLeft;
Double driveSpeedRight;

int autoCount = 0;
int autoCount2 = 100;
int autoCounty = 0;
Boolean autoVar1 = true;
Boolean autoVar2 = true;
Boolean autoVar3 = true;
int autoSequence = 0;
int autoSequence2 = 0;
Boolean autoBoo = true;
Double autoEncoderVar;
Double autoEncoderVar2 = 0.0;
int timer254 = 0;

//PID variables
Double f = 0.1;
Double p = 0.0000144;
Double i = 0.0;
Double d = 0.0;

Double target = -4000.0;
Double pidRaw = 0.0;

Double armUpPos = -4000.0;
Double armDownPos = 23000.0;
Double climbUpPos = -20000.0;
Double climbDownPos = -3000.0;

Boolean startingArmPhase = true;
int armPhaseCounter = 0;
Boolean armVar1 = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    UsbCamera myCam;
    myCam = CameraServer.startAutomaticCapture();
    myCam.setResolution(320, 240);
    myCam.setFPS(30);
    
    leftArm.setSelectedSensorPosition(0.0);
    rightArm.setSelectedSensorPosition(0.0);
    leftDrive1.getEncoder().setPosition(0.0);
    rightDrive1.getEncoder().setPosition(0.0);

    m_chooser.setDefaultOption("Do Nothing", doNothing);
    m_chooser.addOption("Red 2 Note", red2Note);
    // m_chooser.addOption("Red 1 Note", red1Note);
    m_chooser.addOption("Drive Forward Until A-Stop", driveForward);
    m_chooser.addOption("Blue 2 Note", blue2Note);
    SmartDashboard.putData("Auto choices", m_chooser);

  rightDrive1.setInverted(true);
  rightDrive2.setInverted(true);
  outtake.setInverted(true);

  autoCount = 0;
  autoCount2 = 100;
  autoVar1 = true;
  autoVar2 = true;
  autoVar3 = true;
  autoSequence = 0;
  autoSequence2 = 0;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    myPID.setPID(p, i, d);
    pidRaw = myPID.calculate(leftArm.getSelectedSensorPosition(), target);

if(startingArmPhase && target == armDownPos && leftArm.getSelectedSensorPosition() > 19500){
leftArm.set(TalonFXControlMode.PercentOutput, 0.0);
rightArm.set(TalonFXControlMode.PercentOutput, -0.0);
armVar1 = true;
}else if(startingArmPhase){
leftArm.set(TalonFXControlMode.PercentOutput, pidRaw - f);
rightArm.set(TalonFXControlMode.PercentOutput, -pidRaw + f);
}

if(armVar1 && startingArmPhase){
  armPhaseCounter ++;
  if(armPhaseCounter > 49){
  leftArm.setSelectedSensorPosition(0.0);
  rightArm.setSelectedSensorPosition(0.0);
  armDownPos = -200.0;
  armUpPos = -24000.0;
  target = armDownPos;
startingArmPhase = false;
  }
}

    myPID.setPID(p, i, d);
    pidRaw = myPID.calculate(leftArm.getSelectedSensorPosition(), target);


if(!startingArmPhase && target == armDownPos && leftArm.getSelectedSensorPosition() > -2000){
leftArm.set(TalonFXControlMode.PercentOutput, 0.0);
rightArm.set(TalonFXControlMode.PercentOutput, -0.0);

  }else if(!startingArmPhase){
leftArm.set(TalonFXControlMode.PercentOutput, pidRaw - f);
rightArm.set(TalonFXControlMode.PercentOutput, -pidRaw + f);
  }

SmartDashboard.putNumber("target", target);
SmartDashboard.putNumber("Left Arm Position", leftArm.getSelectedSensorPosition());
SmartDashboard.putNumber("Right Arm Position", -1 * rightArm.getSelectedSensorPosition());
SmartDashboard.putNumber("Left Encoder", leftDrive1.getEncoder().getPosition());
SmartDashboard.putNumber("Right Encoder", rightDrive1.getEncoder().getPosition());
SmartDashboard.putNumber("left motor percent", leftArm.getMotorOutputPercent());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", doNothing);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {

        // 355.3 tics is one wheel rotation
        // wheel circumfrence is 18.85 inches
        // 18.85 tics per inch, unbelievable, i know
        // about 1800 tics is a solid backup

      case red2Note:
        // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches

      /*    timer254++;
        if (timer254 < 400){
          
        } else  */
        if (rightDrive1.getEncoder().getPosition() > -24.9 && autoVar1) {
          target = armUpPos;
          leftDrive1.set(-0.15);
          leftDrive2.set(-0.15);
          rightDrive1.set(-0.15);
          rightDrive2.set(-0.15);
        } else if(rightDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
        } else if(rightDrive1.getEncoder().getPosition() > -49.2 && autoVar1){
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoCount2 = 0;
        } else if(autoCount2 < 30 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(-0.1);
          leftDrive2.set(-0.1);
          rightDrive1.set(-0.1);
          rightDrive2.set(-0.1);
        }else if(autoCount2 < 65 && autoVar1){
          outtake.set(TalonSRXControlMode.PercentOutput,1);
          intake.set(TalonSRXControlMode.PercentOutput,1);
        }else if(autoVar1){
          autoEncoderVar = rightDrive1.getEncoder().getPosition();
          outtake.set(TalonSRXControlMode.PercentOutput,0);
          intake.set(TalonSRXControlMode.PercentOutput,0);
          target = armDownPos;
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoVar1 = false;
        }


        if(!autoVar1 && rightDrive1.getEncoder().getPosition() > autoEncoderVar + 7.5 && autoVar2){
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 1;
        } else if(!autoVar1 && rightDrive1.getEncoder().getPosition() < autoEncoderVar - 6.8 && autoVar2 && autoSequence == 1){
          intake.set(TalonSRXControlMode.PercentOutput,1);
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && rightDrive1.getEncoder().getPosition() > autoEncoderVar + 4.2 && autoVar3) {
          intake.set(TalonSRXControlMode.PercentOutput,0);
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 2;
        } else if(!autoVar2 && rightDrive1.getEncoder().getPosition() < autoEncoderVar - 2.8 && autoVar3 && autoSequence ==2){
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoVar3 = false;
        }


        if (!autoVar3 && rightDrive1.getEncoder().getPosition() > autoEncoderVar + 4.2) {
          target = armUpPos;
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 3;
          autoCount = 0;
        } else if(!autoVar3 && autoSequence == 3 && autoCount < 250 && rightDrive1.getEncoder().getPosition() < autoEncoderVar - 20){
          // Arrived at the Amp again
          if(autoBoo){
            autoCount = 0;
            autoBoo = false;
          }
          leftDrive1.set(-0.1);
          leftDrive2.set(-0.1);
          rightDrive1.set(-0.1);
          rightDrive2.set(-0.1);
          outtake.set(TalonSRXControlMode.PercentOutput,1);
          intake.set(TalonSRXControlMode.PercentOutput,1);
          autoEncoderVar2 = rightDrive1.getEncoder().getPosition();
          autoSequence2 = 1;
        } else if(!autoVar3 && rightDrive1.getEncoder().getPosition() > autoEncoderVar2 - 10 && autoSequence2 == 1){
          outtake.set(TalonSRXControlMode.PercentOutput,0);
          intake.set(TalonSRXControlMode.PercentOutput,0);
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 5;
        }else if(!autoVar3 && autoSequence == 5){
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
        }
      break;


      case driveForward:
        // 355.3 tics is one wheel rotation
        // wheel circumfrence is 18.85 inches
        // 18.85 tics per inch, unbelievable, i know
        // about 1800 tics is a solid backup

        //Drive Straight Forward 8 ft
          target = armUpPos;
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
      break;


      case blue2Note:
  // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches 

      /*      timer254++;
        if (timer254 < 400){
          
        } else */
        if (leftDrive1.getEncoder().getPosition() > -24.9 && autoVar1) {
          target = armUpPos;
          leftDrive1.set(-0.15);
          leftDrive2.set(-0.15);
          rightDrive1.set(-0.15);
          rightDrive2.set(-0.15);
        } else if(leftDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
        } else if(leftDrive1.getEncoder().getPosition() > -49.2 && autoVar1){
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoCount2 = 0;
        } else if(autoCount2 < 30 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(-0.1);
          leftDrive2.set(-0.1);
          rightDrive1.set(-0.1);
          rightDrive2.set(-0.1);
        }else if(autoCount2 < 65 && autoVar1){
          outtake.set(TalonSRXControlMode.PercentOutput,1);
          intake.set(TalonSRXControlMode.PercentOutput,1);
        }else if(autoVar1){
          autoEncoderVar = leftDrive1.getEncoder().getPosition();
          outtake.set(TalonSRXControlMode.PercentOutput,0);
          intake.set(TalonSRXControlMode.PercentOutput,0);
          target = armDownPos;
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoVar1 = false;
        }


        if(!autoVar1 && leftDrive1.getEncoder().getPosition() > autoEncoderVar + 7.5 && autoVar2){
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoSequence = 1;
        } else if(!autoVar1 && leftDrive1.getEncoder().getPosition() < autoEncoderVar - 6.8 && autoVar2 && autoSequence == 1){
          intake.set(TalonSRXControlMode.PercentOutput,1);
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && leftDrive1.getEncoder().getPosition() > autoEncoderVar + 4.2 && autoVar3) {
          intake.set(TalonSRXControlMode.PercentOutput,0);
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 2;
          // About where it crashes
        } else if(!autoVar2 && leftDrive1.getEncoder().getPosition() < autoEncoderVar - 2.8 && autoVar3 && autoSequence == 2){
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoVar3 = false;
        }


        if (!autoVar3 && leftDrive1.getEncoder().getPosition() > autoEncoderVar + 4.2) {
          target = armUpPos;
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(-0.25);
          rightDrive2.set(-0.25);
          autoSequence = 3;
          autoCount = 0;
        } else if(!autoVar3 && autoSequence == 3 && autoCount < 250 && leftDrive1.getEncoder().getPosition() < autoEncoderVar - 20){
          // Arrived at the Amp again
          if(autoBoo){
            autoCount = 0;
            autoBoo = false;
          }
          leftDrive1.set(-0.1);
          leftDrive2.set(-0.1);
          rightDrive1.set(-0.1);
          rightDrive2.set(-0.1);
          outtake.set(TalonSRXControlMode.PercentOutput,1);
          intake.set(TalonSRXControlMode.PercentOutput,1);
          autoEncoderVar2 = leftDrive1.getEncoder().getPosition();
          autoSequence2 = 1;
        } else if(!autoVar3 && leftDrive1.getEncoder().getPosition() > autoEncoderVar2 - 10 && autoSequence2 == 1){
          outtake.set(TalonSRXControlMode.PercentOutput,0);
          intake.set(TalonSRXControlMode.PercentOutput,0);
          leftDrive1.set(-0.25);
          leftDrive2.set(-0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
          autoSequence = 5;
        }else if(!autoVar3 && autoSequence == 5){
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
        }
      break;


      case doNothing:
      default:
   // Do Nothing

        break;
    }
autoCount++;
autoCount2++;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // This is the drive code
  
if(joyDrive.getRawAxis(1) > -0.1 && joyDrive.getRawAxis(1) < 0.1){
driveSpeedLeft = 0.0;
}else if(joyDrive.getRawAxis(1) < 0){
driveSpeedLeft = joyDrive.getRawAxis(1) * joyDrive.getRawAxis(1);
}else{
driveSpeedLeft = joyDrive.getRawAxis(1) * joyDrive.getRawAxis(1) * -1;
}

if(joyDrive.getRawAxis(5) > -0.1 && joyDrive.getRawAxis(5) < 0.1){
driveSpeedRight = 0.0;
}else if(joyDrive.getRawAxis(5) < 0){
driveSpeedRight = joyDrive.getRawAxis(5) * joyDrive.getRawAxis(5);
}else{
driveSpeedRight = joyDrive.getRawAxis(5) * joyDrive.getRawAxis(5) * -1;
}

if(joyDrive.getRawButton(1)){
rightDrive1.set(driveSpeedLeft);
rightDrive2.set(driveSpeedLeft);
leftDrive1.set(driveSpeedLeft);
leftDrive2.set(driveSpeedLeft);
}else{
rightDrive1.set(driveSpeedRight);
rightDrive2.set(driveSpeedRight);
leftDrive1.set(driveSpeedLeft);
leftDrive2.set(driveSpeedLeft);
}



// Intake & Outtake code
if (joyButtons.getRawButton(1)) {
  intake.set(TalonSRXControlMode.PercentOutput,1);
} else if(joyButtons.getRawButton(4)){
  intake.set(TalonSRXControlMode.PercentOutput, -1);
} else {
    intake.set(TalonSRXControlMode.PercentOutput,0);
}

if (joyButtons.getRawButton(2)) {
  outtake.set(TalonSRXControlMode.PercentOutput,1);
} else {
    outtake.set(TalonSRXControlMode.PercentOutput,0);
}


if (!joyButtons.getRawButton(11) && !joyButtons.getRawButton(12)){
// Arm Up & Down code
if (joyButtons.getRawButton(8)) {
  target = armUpPos;
} else if(joyButtons.getRawButton(7)){
  target = armDownPos;
  }
} else if(joyButtons.getRawButton(11)){

// Manuel Mode
  if(joyButtons.getRawAxis(1) == -1){
target -= 300;
}else if(joyButtons.getRawAxis(1) == 1){
target += 300;
}

}else if(joyButtons.getRawButton(12)){
  
  // Fine Control Manuel
  if(joyButtons.getRawAxis(1) == -1){
target -= 50;
}else if(joyButtons.getRawAxis(1) == 1){
target += 50;
}
}

if(joyButtons.getRawButton(3)){
climbLatch.set(TalonSRXControlMode.PercentOutput,1);
}else{
  climbLatch.set(TalonSRXControlMode.PercentOutput,0);
}
}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
  // Members:
// Charlie
// Jackson
// Jose
// Tom
// Maya
// Matteo
// Lachlan
// Aki
// Hunter
  // Mentors
// Felix
// Rich
// Jessica
// Owen
// Andy
// Alejandra
// Jared
// Chris
// Eric
// Mark from PG&E
  // Mascots:
// Ottie
// Rosy
// Hank
// Bailey
// Luna