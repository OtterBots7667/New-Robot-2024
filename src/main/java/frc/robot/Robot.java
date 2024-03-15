// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private static final String driveForward= "Drive Forward 8ft";
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
Boolean autoVar4 = true;
Boolean autoVar5 = true;
Boolean autoVar6 = true;
int autoSequence = 0;
int autoSequence2 = 0;
Boolean autoBoo = true;
Double autoEncoderVar;
Double autoEncoderVar2;

//PID variables
Double f = 0.1149;
Double p = 0.0000144;
Double i = 0.0;
Double d = 0.0;

Double target = -4000.0;
Double pidRaw = 0.0;

Double armUpPos = -4000.0;
Double armDownPos = 23000.0;

Boolean startingArmPhase = true;
int armPhaseCounter = 0;
Boolean armVar1 = false;

// Double armUpPos = -24000.0;
// Double armDownPos = -200.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftArm.setSelectedSensorPosition(0.0);
    rightArm.setSelectedSensorPosition(0.0);
    leftDrive1.getEncoder().setPosition(0.0);
    rightDrive1.getEncoder().setPosition(0.0);

    m_chooser.setDefaultOption("Do Nothing", doNothing);
    m_chooser.addOption("Red 2 Note", red2Note);
    // m_chooser.addOption("Red 1 Note", red1Note);
    m_chooser.addOption("Drive Forward 8 ft", driveForward);
    m_chooser.addOption("Blue 2 Note", blue2Note);
    // m_chooser.addOption("Blue1 Note", blue1Note);
    // m_chooser.addOption("Red 3 Note", red3Note);
    // m_chooser.addOption("Blue 3 Note", blue3Note);
    // m_chooser.addOption("Red 3 Push Middle", redPushMiddle3);
    // m_chooser.addOption("Blue 3 Push Middle", bluePushMiddle3);
    // m_chooser.addOption("Blue 2 Push Middle", bluePushMiddle2);
    // m_chooser.addOption("Red 2 Push Middle", redPushMiddle2);
    SmartDashboard.putData("Auto choices", m_chooser);

  rightDrive1.setInverted(true);
  rightDrive2.setInverted(true);
  outtake.setInverted(true);

  autoCount = 0;
  autoCount2 = 100;
  autoVar1 = true;
  autoVar2 = true;
  autoVar3 = true;
  autoVar4 = true;
  autoVar5 = true;
  autoVar6 = true; 
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

// leftArm.set(TalonFXControlMode.PercentOutput, pidRaw - f);
// rightArm.set(TalonFXControlMode.PercentOutput, -pidRaw + f);


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

// var rightDrivePos = rightDrive1.get();
// Double dub = rightDrivePos.getValueAsDouble();


    switch (m_autoSelected) {

        // 355.3 tics is one wheel rotation
        // wheel circumfrence is 18.85 inches
        // 18.85 tics per inch, unbelievable, i know
        // about 1800 tics is a solid backup

      case red2Note:
        // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches 
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


        if(!autoVar1 && rightDrive1.getEncoder().getPosition() > autoEncoderVar + 2.2 && autoVar2){
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

  //     case red3Note:
        
  //       if (rightDrive1.getEncoder().getPosition() > -28.9 && autoVar1) {
  //         target = armUpPos;
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //       } else if(rightDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //       } else if(rightDrive1.getEncoder().getPosition() > -47.2 && autoVar1){
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoCount2 = 0;
  //       } else if(autoCount2 < 20 && autoVar1){
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //       }else if(autoCount2 < 55 && autoVar1){
  //         outtake.set(TalonSRXControlMode.PercentOutput,1);
  //       }else if(autoVar1){
  //         outtake.set(TalonSRXControlMode.PercentOutput,0);
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoVar1 = false;
  //       }


  //       if(!autoVar1 && rightDrive1.getEncoder().getPosition() > -32.0 && autoVar2){
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 1;
  //       } else if(!autoVar1 && rightDrive1.getEncoder().getPosition() < -41.1 && autoVar2 && autoSequence == 1){
  //         target = armDownPos;
  //         intake.set(TalonSRXControlMode.PercentOutput,1);
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoVar2 = false;
  //       }


  //       if (!autoVar2 && rightDrive1.getEncoder().getPosition() > -28.2 && autoVar3) {
  //         target = armUpPos;
  //         intake.set(TalonSRXControlMode.PercentOutput,0);
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 2;
  //       } else if(!autoVar2 && rightDrive1.getEncoder().getPosition() < -41.1 && autoVar3 && autoSequence == 2){
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoVar3 = false;
  //       }


  //       if (!autoVar3 && rightDrive1.getEncoder().getPosition() > -32.0 && autoVar4) {
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 3;
  //       } else if(!autoVar3 && rightDrive1.getEncoder().getPosition() < -47.2 && autoVar4 && autoSequence == 3){
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //         outtake.set(TalonSRXControlMode.PercentOutput,1);
  //         autoCount2 = 0;
  //         autoSequence = 4;
  //       } else if(!autoVar3 && autoCount2 > 35 && autoVar4 && autoSequence == 4){
  //         outtake.set(TalonSRXControlMode.PercentOutput,0);
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoVar4 = false;
  //       }

  //       if (!autoVar4 && rightDrive1.getEncoder().getPosition() > -1.5 && autoVar5) {
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoSequence = 5;
  //       } else if(!autoVar4 && rightDrive1.getEncoder().getPosition() < -10.6 && autoVar5 && autoSequence == 5){
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         target = armDownPos;
  //         intake.set(TalonSRXControlMode.PercentOutput,1);
  //         autoVar5 = false;
  //       }

  //       if (!autoVar5 && rightDrive1.getEncoder().getPosition() > 2.4 && autoVar6) {
  //         target = armUpPos;
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 6;
  //       } else if(!autoVar5 && rightDrive1.getEncoder().getPosition() < -10.6 && autoVar6 && autoSequence == 6) {
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoVar6 = false;
  //       }

  //       if (!autoVar6 && rightDrive1.getEncoder().getPosition() > -1.5) {
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 7;
  //       } else if(!autoVar6 && rightDrive1.getEncoder().getPosition() < -47.2 && autoSequence == 7) {
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //         autoCount2 = 0;
  //       } else if(!autoVar6 && autoCount2 > 15){
  //         outtake.set(TalonSRXControlMode.PercentOutput,1);
  //       } else if(!autoVar6 && autoCount2 > 45){
  //         outtake.set(TalonSRXControlMode.PercentOutput,0);
  //       }

  //     break;


  //     case red1Note:
  // if (rightDrive1.getEncoder().getPosition() > -28.9 && autoVar1) {
  //         target = armUpPos;
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //       } else if(rightDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //       } else if(rightDrive1.getEncoder().getPosition() > -47.2 && autoVar1){
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoCount2 = 0;
  //       } else if(autoCount2 < 20 && autoVar1){
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //       }else if(autoCount2 < 55 && autoVar1){
  //         outtake.set(TalonSRXControlMode.PercentOutput,1);
  //       }else if(autoVar1){
  //         outtake.set(TalonSRXControlMode.PercentOutput,0);
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         autoVar1 = false;
  //       }


  //       if (!autoVar1 && rightDrive1.getEncoder().getPosition() < -40.8) {
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //         autoSequence = 1;
  //       } else if(!autoVar1 && rightDrive1.getEncoder().getPosition() < -49.9 && autoSequence == 1){
  //         leftDrive1.set(-0.25);
  //         leftDrive2.set(-0.25);
  //         rightDrive1.set(-0.25);
  //         rightDrive2.set(-0.25);
  //       } else if(!autoVar1 && rightDrive1.getEncoder().getPosition() < -81.7 && autoSequence == 1){
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //         target = armDownPos;
  //         autoSequence = 2;
  //       } else if(!autoVar1 && autoCount > 19.7 && autoSequence == 2){
  //         leftDrive1.set(1);
  //         leftDrive2.set(1);
  //         rightDrive1.set(1);
  //         rightDrive2.set(1);

  //       }
  //     break;


  //     case redPushMiddle3:

  //       if (leftDrive1.getEncoder().getPosition() < 14.1) {
  //         target = armUpPos;
  //         leftDrive1.set(0.25);
  //         leftDrive2.set(0.25);
  //         rightDrive1.set(0.25);
  //         rightDrive2.set(0.25);
  //         outtake.set(TalonSRXControlMode.PercentOutput,1);
  //       }else if(leftDrive1.getEncoder().getPosition() < 86.0){
  //         leftDrive1.set(1);
  //         leftDrive2.set(1);
  //         rightDrive1.set(1);
  //         rightDrive2.set(1);
  //         outtake.set(TalonSRXControlMode.PercentOutput,0);
  //         target = armDownPos;
  //       } else if(leftDrive1.getEncoder().getPosition() < 143.4){
  //         leftDrive1.set(1);
  //         leftDrive2.set(1);
  //         rightDrive1.set(.872);
  //         rightDrive2.set(.872);
  //         intake.set(TalonSRXControlMode.PercentOutput,1);
  //       }else if(leftDrive1.getEncoder().getPosition() < 197.2){
  //         leftDrive1.set(1);
  //         leftDrive2.set(1);
  //         rightDrive1.set(1);
  //         rightDrive2.set(1);
  //       }else if(leftDrive1.getEncoder().getPosition() < 232.6){
  //         intake.set(TalonSRXControlMode.PercentOutput,0);
  //       }else{
  //         leftDrive1.set(0);
  //         leftDrive2.set(0);
  //         rightDrive1.set(0);
  //         rightDrive2.set(0);
  //       }

  //     break;


  //     case redPushMiddle2:

  //     if(leftDrive1.getEncoder().getPosition() < 14.1){
  //       leftDrive1.set(0.25);
  //       leftDrive2.set(0.25);
  //       rightDrive1.set(0.25);
  //       rightDrive2.set(0.25);
  //       target = armUpPos;
  //       outtake.set(TalonSRXControlMode.PercentOutput,1);
  //     } else if(leftDrive1.getEncoder().getPosition() < 57.4){
  //       target = armDownPos;
  //       outtake.set(TalonSRXControlMode.PercentOutput,0);
  //       leftDrive1.set(1);
  //       leftDrive2.set(1);
  //       rightDrive1.set(1);
  //       rightDrive2.set(1);
  //     }else if(leftDrive1.getEncoder().getPosition() < 177.5){
  //       leftDrive1.set(1);
  //       leftDrive2.set(1);
  //       rightDrive1.set(0.938);
  //       rightDrive2.set(0.938);
  //       intake.set(TalonSRXControlMode.PercentOutput,1);
  //     }else if(leftDrive1.getEncoder().getPosition() < 216.0){
  //       leftDrive1.set(1);
  //       leftDrive2.set(1);
  //       rightDrive1.set(1);
  //       rightDrive2.set(1);
  //       intake.set(TalonSRXControlMode.PercentOutput,0);
  //     }else{
  //       leftDrive1.set(0);
  //       leftDrive2.set(0);
  //       rightDrive1.set(0);
  //       rightDrive2.set(0);
  //     }

  //     break;


      case driveForward:
        // 355.3 tics is one wheel rotation
        // wheel circumfrence is 18.85 inches
        // 18.85 tics per inch, unbelievable, i know
        // about 1800 tics is a solid backup

        //Drive Straight Forward 8 ft
        if (leftDrive1.getEncoder().getPosition() < 45) {
          target = armUpPos;
          leftDrive1.set(0.25);
          leftDrive2.set(0.25);
          rightDrive1.set(0.25);
          rightDrive2.set(0.25);
        } else {
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }
      break;

      case blue2Note:
  // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches 
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


        if(!autoVar1 && leftDrive1.getEncoder().getPosition() > autoEncoderVar + 2.2 && autoVar2){
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
        } else if(!autoVar3){
          outtake.set(TalonSRXControlMode.PercentOutput,0);
          intake.set(TalonSRXControlMode.PercentOutput,0);
          // Everything turns off, we're all done.
          // (Or are we?)
        }
      break;
      

//       case blue3Note:
//         if (leftDrive1.getEncoder().getPosition() > -28.9 && autoVar1) {
//           target = armUpPos;
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//         } else if(leftDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//         } else if(leftDrive1.getEncoder().getPosition() > -47.2 && autoVar1){
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoCount2 = 0;
//         } else if(autoCount2 < 20 && autoVar1){
//           leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//         }else if(autoCount2 < 55 && autoVar1){
//           outtake.set(TalonSRXControlMode.PercentOutput,1);
//         }else if(autoVar1){
//           outtake.set(TalonSRXControlMode.PercentOutput,0);
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoVar1 = false;
//         }


//         if(!autoVar1 && leftDrive1.getEncoder().getPosition() > -32.0 && autoVar2){
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoSequence = 1;
//         } else if(!autoVar1 && leftDrive1.getEncoder().getPosition() < -41.1 && autoVar2 && autoSequence == 1){
//           target = armDownPos;
//           intake.set(TalonSRXControlMode.PercentOutput,1);
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoVar2 = false;
//         }


//         if (!autoVar2 && leftDrive1.getEncoder().getPosition() > -28.2 && autoVar3) {
//           target = armUpPos;
//           intake.set(TalonSRXControlMode.PercentOutput,0);
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoSequence = 2;
//         } else if(!autoVar2 && rightDrive1.getEncoder().getPosition() < -41.1 && autoVar3 && autoSequence == 2){
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoVar3 = false;
//         }


//         if (!autoVar3 && leftDrive1.getEncoder().getPosition() > -32.0) {
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoSequence = 3;
//         } else if(!autoVar3 && leftDrive1.getEncoder().getPosition() < -47.2 && autoSequence == 3){
//           leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//           outtake.set(TalonSRXControlMode.PercentOutput,1);
//           autoCount2 = 0;
//         } else if(autoCount2 > 35 && autoSequence == 3){
//           outtake.set(TalonSRXControlMode.PercentOutput,0);
//         }

//         if (!autoVar4 && leftDrive1.getEncoder().getPosition() > -1.5 && autoVar5) {
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoSequence = 4;
//         } else if(!autoVar4 && leftDrive1.getEncoder().getPosition() < -10.6 && autoVar5 && autoSequence == 4){
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           target = armDownPos;
//           intake.set(TalonSRXControlMode.PercentOutput,1);
//           autoVar5 = false;
//         }

//         if (!autoVar5 && leftDrive1.getEncoder().getPosition() > 2.4 && autoVar6) {
//           target = armUpPos;
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoSequence = 5;
//         } else if(!autoVar5 && leftDrive1.getEncoder().getPosition() < -10.6 && autoVar6 && autoSequence == 5) {
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoVar6 = false;
//         }

//         if (!autoVar6 && leftDrive1.getEncoder().getPosition() > -1.5) {
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoSequence = 6;
//         } else if(!autoVar6 && leftDrive1.getEncoder().getPosition() < -47.2 && autoSequence == 6) {
//           leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//           autoCount2 = 0;
//         } else if(!autoVar6 && autoCount2 > 15){
//           outtake.set(TalonSRXControlMode.PercentOutput,1);
//         } else if(!autoVar6 && autoCount2 > 45){
//           outtake.set(TalonSRXControlMode.PercentOutput,0);
//         }

//       break;


//       case blue1Note:
//   if (leftDrive1.getEncoder().getPosition() > -28.9 && autoVar1) {
//           target = armUpPos;
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//         } else if(leftDrive1.getEncoder().getPosition() > -38.0 && autoVar1) {
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//         } else if(leftDrive1.getEncoder().getPosition() > -47.2 && autoVar1){
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//           autoCount2 = 0;
//         } else if(autoCount2 < 20 && autoVar1){
// \          leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//         }else if(autoCount2 < 55 && autoVar1){
//           outtake.set(TalonSRXControlMode.PercentOutput,1);
//         }else if(autoVar1){
//           outtake.set(TalonSRXControlMode.PercentOutput,0);
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoVar1 = false;
//         }


//         if (!autoVar1 && leftDrive1.getEncoder().getPosition() < -40.8) {
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           autoSequence = 1;
//         } else if(!autoVar1 && leftDrive1.getEncoder().getPosition() < -49.9 && autoSequence == 1){
//           leftDrive1.set(-0.25);
//           leftDrive2.set(-0.25);
//           rightDrive1.set(-0.25);
//           rightDrive2.set(-0.25);
//         } else if(!autoVar1 && leftDrive1.getEncoder().getPosition() < -81.7 && autoSequence == 1){
//           leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//         } else if(!autoVar1 && autoCount > 19.7 && autoSequence == 1){
//           leftDrive1.set(1);
//           leftDrive2.set(1);
//           rightDrive1.set(1);
//           rightDrive2.set(1);
//         }
//       break;


//       case bluePushMiddle3:
//         if (rightDrive1.getEncoder().getPosition() < 14.1) {
//           target = armUpPos;
//           leftDrive1.set(0.25);
//           leftDrive2.set(0.25);
//           rightDrive1.set(0.25);
//           rightDrive2.set(0.25);
//           outtake.set(TalonSRXControlMode.PercentOutput,1);
//         }else if(rightDrive1.getEncoder().getPosition() < 86.0){
//           target = armDownPos;
//           leftDrive1.set(1);
//           leftDrive2.set(1);
//           rightDrive1.set(1);
//           rightDrive2.set(1);
//           outtake.set(TalonSRXControlMode.PercentOutput,0);
//         } else if(rightDrive1.getEncoder().getPosition() < 143.4){
//           leftDrive1.set(.872);
//           leftDrive2.set(.872);
//           rightDrive1.set(1);
//           rightDrive2.set(1);
//           intake.set(TalonSRXControlMode.PercentOutput,1);
//         }else if(rightDrive1.getEncoder().getPosition() < 197.2){
//           leftDrive1.set(1);
//           leftDrive2.set(1);
//           rightDrive1.set(1);
//           rightDrive2.set(1);
//         }else if(rightDrive1.getEncoder().getPosition() < 232.6){
//           intake.set(TalonSRXControlMode.PercentOutput,0);
//         }else{
//           leftDrive1.set(0);
//           leftDrive2.set(0);
//           rightDrive1.set(0);
//           rightDrive2.set(0);
//         }

//       break;

//       case bluePushMiddle2:
//       if(rightDrive1.getEncoder().getPosition() < 14.1){
//         target = armUpPos;
//         outtake.set(TalonSRXControlMode.PercentOutput,1);
//         leftDrive1.set(0.25);
//         leftDrive2.set(0.25);
//         rightDrive1.set(0.25);
//         rightDrive2.set(0.25);
//       }else if(rightDrive1.getEncoder().getPosition() < 57.4){
//         target = armDownPos;
//         outtake.set(TalonSRXControlMode.PercentOutput,0);
//         leftDrive1.set(1);
//         leftDrive2.set(1);
//         rightDrive1.set(1);
//         rightDrive2.set(1);
//       } else if(rightDrive1.getEncoder().getPosition() < 177.5){
//         leftDrive1.set(0.938);
//         leftDrive2.set(0.938);
//         rightDrive1.set(1);
//         rightDrive2.set(1);
//         intake.set(TalonSRXControlMode.PercentOutput,1);
//       }else if(rightDrive1.getEncoder().getPosition() < 216.0){
//         leftDrive1.set(1);
//         leftDrive2.set(1);
//         rightDrive1.set(1);
//         rightDrive2.set(1);
//         intake.set(TalonSRXControlMode.PercentOutput,0);
//       }else{
//         leftDrive1.set(0);
//         leftDrive2.set(0);
//         rightDrive1.set(0);
//         rightDrive2.set(0);
//       }
//         break;

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
  intake.set(TalonSRXControlMode.PercentOutput, -0.35);
} else {
    intake.set(TalonSRXControlMode.PercentOutput,0);
}

if (joyButtons.getRawButton(2)) {
  outtake.set(TalonSRXControlMode.PercentOutput,1);
} else {
    outtake.set(TalonSRXControlMode.PercentOutput,0);
}


if (!joyButtons.getRawButton(11)){
// Arm Up & Down code
if (joyButtons.getRawButton(8)) {
  target = armUpPos;
} else if(joyButtons.getRawButton(7)){
  target = armDownPos;
  }
} else if(joyButtons.getRawButton(11)){

  if(joyButtons.getRawAxis(1) == -1){
target -= 200;
}else if(joyButtons.getRawAxis(1) == 1){
target += 200;
}

}



//      else if(joyButtons.getRawButton(11)){
//     if(joyButtons.getRawAxis(1) == -1){
//       target += 50;
//     } else if(joyButtons.getRawAxis(1) == 1){
//       target -= 50;
//     }
//   }

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
leftArm.set(TalonFXControlMode.PercentOutput, joyDrive.getRawAxis(1) * -1);
rightArm.set(TalonFXControlMode.PercentOutput, joyDrive.getRawAxis(1));
System.out.println(leftArm.getMotorOutputPercent());
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
  // Members:
// Charles
// Jackson
// Jose
// Tom
// Maya
// Matteo
// Lachlan
// Aki
  // Mentors & Parents:
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