// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String doNothing = "Do Nothing";
  private static final String red2Note = "Red 2 Note";
  private static final String red1Note = "Red 1 Note";
  private static final String driveForward= "Drive Forward 8ft";
  private static final String blue2Note = "Blue 2 Note";
  private static final String blue1Note = "Blue1 Note";
  private static final String red3Note = "Red 3 Note";
  private static final String blue3Note = "Blue 3 Note";
  private static final String redPushMiddle3 = "Red 3 Push Middle";
  private static final String bluePushMiddle3 = "Blue 3 Push Middle";
  private static final String redPushMiddle2 = "Red 2 Push Middle";
  private static final String bluePushMiddle2 = "Blue 2 Push Middle";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


private Joystick joyDrive = new Joystick(1);
  
private TalonFX rightArm = new TalonFX(0);
private TalonFX leftArm = new TalonFX(0);
private TalonFX intake = new TalonFX(0);
private TalonFX outtake = new TalonFX(0);

private CANSparkMax leftDrive1 = new CANSparkMax(0, MotorType.kBrushless);
private CANSparkMax leftDrive2 = new CANSparkMax(0, MotorType.kBrushless);
private CANSparkMax rightDrive1 = new CANSparkMax(0, MotorType.kBrushless);
private CANSparkMax rightDrive2 = new CANSparkMax(0, MotorType.kBrushless);

private RelativeEncoder leftEncoder1;
private RelativeEncoder rightEncoder1;

private PIDController myPID = new PIDController(0, 0, 0);

DifferentialDrive driveTrain = new DifferentialDrive(
    (double output) -> {
        leftDrive1.set(output);
        leftDrive2.set(output);
    },
    (double output) -> {
        rightDrive1.set(output);
        rightDrive2.set(output);
    });


int autoCount = 0;
int autoCount2 = 100;
Boolean autoVar1 = true;
Boolean autoVar2 = true;
Boolean autoVar3 = true;
Boolean autoVar4 = true;
Boolean autoVar5 = true;
Boolean autoVar6 = true;
//PID variables

Double p = 0.0;
Double i = 0.0;
Double d = 0.0;

Double target = 0.0;
Double pidRaw = 0.0;

// Change these later
Double armUpPos = 0.0;
Double armDownPos = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Do Nothing", doNothing);
    m_chooser.addOption("Red 2 Note", red2Note);
    m_chooser.addOption("Red 1 Note", red1Note);
    m_chooser.addOption("Drive Forward 8 ft", driveForward);
    m_chooser.addOption("Blue 2 Note", blue2Note);
    m_chooser.addOption("Blue1 Note", blue1Note);
    m_chooser.addOption("Red 3 Note", red3Note);
    m_chooser.addOption("Blue 3 Note", blue3Note);
    m_chooser.addOption("Red 3 Push Middle", redPushMiddle3);
    m_chooser.addOption("Blue 3 Push Middle", bluePushMiddle3);
    m_chooser.addOption("Blue 2 Push Middle", bluePushMiddle2);
    m_chooser.addOption("Red 2 Push Middle", redPushMiddle2);
    SmartDashboard.putData("Auto choices", m_chooser);

  rightDrive1.setInverted(true);
  rightDrive2.setInverted(true);

  autoCount = 0;
  autoCount2 = 100;
  autoVar1 = true;
  autoVar2 = true;
  autoVar3 = true;
  autoVar4 = true;
  autoVar5 = true;
  autoVar6 = true; 
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
    var midMan = leftArm.getRotorPosition();
    var armPos = midMan.getValueAsDouble();
    pidRaw = myPID.calculate(armPos, target);

leftArm.set(pidRaw);
rightArm.set(-pidRaw);


leftEncoder1 = leftDrive1.getEncoder();
rightEncoder1 = rightDrive1.getEncoder();

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
      case red2Note:
        // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches 
        if (rightEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if(!autoVar1 && rightEncoder1.getPosition() > -1138 && autoVar2){
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar1 && rightEncoder1.getPosition() < -1460 && autoVar2){
          target = armDownPos;
          intake.set(1);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && rightEncoder1.getPosition() > -1000 && autoVar3) {
          target = armUpPos;
          intake.set(0);
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar2 && rightEncoder1.getPosition() < -1460 && autoVar3){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar3 = false;
        }


        if (!autoVar3 && rightEncoder1.getPosition() > -1138) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar3 && rightEncoder1.getPosition() < -1675){
          // Arrived at the Amp again
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          outtake.set(1);
          autoCount2 = 0;
        } else if(!autoVar3 && autoCount2 > 35){
          outtake.set(0);
          // Everything turns off, we're all done.
          // (Or are we?)
        }
      break;

      case red3Note:
  // Position 1, score 2 notes, pick up third 
        // 1,027 ticks is 54.5 inches 
        if (rightEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if(!autoVar1 && rightEncoder1.getPosition() > -1138 && autoVar2){
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar1 && rightEncoder1.getPosition() < -1460 && autoVar2){
          target = armDownPos;
          intake.set(1);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && rightEncoder1.getPosition() > -1000 && autoVar3) {
          target = armUpPos;
          intake.set(0);
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar2 && rightEncoder1.getPosition() < -1460 && autoVar3){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar3 = false;
        }


        if (!autoVar3 && rightEncoder1.getPosition() > -1138 && autoVar4) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar3 && rightEncoder1.getPosition() < -1675 && autoVar4){
          // Arrived at the Amp again
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          outtake.set(1);
          autoCount2 = 0;
        } else if(!autoVar3 && autoCount2 > 35 && autoVar4){
          outtake.set(0);
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoVar4 = false;
        }

        if (!autoVar4 && rightEncoder1.getPosition() > -54 && autoVar5) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(!autoVar4 && rightEncoder1.getPosition() < -376 && autoVar5){
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          target = armDownPos;
          intake.set(1);
          autoVar5 = false;
        }

        if (!autoVar5 && rightEncoder1.getPosition() > 84 && autoVar6) {
          target = armUpPos;
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar5 && rightEncoder1.getPosition() < -376 && autoVar6) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar6 = false;
        }

        if (!autoVar6 && rightEncoder1.getPosition() > -54) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar6 && rightEncoder1.getPosition() < -1675) {
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          autoCount2 = 0;
        } else if(!autoVar6 && autoCount2 > 15){
          outtake.set(1);
        } else if(!autoVar6 && autoCount2 > 45){
          outtake.set(0);
        }



      break;


      case red1Note:
        // Position 1, score 1 note, leave zone last second
  if (rightEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(rightEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if (!autoVar1 && rightEncoder1.getPosition() < -1450) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar1 && rightEncoder1.getPosition() < -1772){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar1 && rightEncoder1.getPosition() < -2900){
          // Robot stops against back wall, waits, biding its time
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        } else if(!autoVar1 && autoCount > 700){
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
          /*  
              The Robot's machanations come to fruition
              Robot zooms out of the starting zone at mach 10
              + 2 points (:
          */
        }
      break;


      case redPushMiddle3:
      // The mission of this auto is to get directly in the way
      // of a robot on the opposite alliance
      // that is running a 5, 6, 7 note auto


        if (leftEncoder1.getPosition() < 3054) {
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
        } else if(leftEncoder1.getPosition() < 5090){
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(.872);
          rightDrive2.set(.872);
          target = armDownPos;
          intake.set(1);
        }else if(leftEncoder1.getPosition() < 7000){
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
        }else if(leftEncoder1.getPosition() < 8257){
          intake.set(0);
          target = armUpPos;
        }else{
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }

      break;


      case redPushMiddle2:
      // annoying auto, starting in pos 2
      if(leftEncoder1.getPosition() < 2036){
        leftDrive1.set(1);
        leftDrive2.set(1);
        rightDrive1.set(1);
        rightDrive2.set(1);
        target = armDownPos;
      } else if(leftEncoder1.getPosition() < 6300){
        leftDrive1.set(1);
        leftDrive2.set(1);
        rightDrive1.set(0.938);
        rightDrive2.set(0.938);
        intake.set(1);
      }else if(leftEncoder1.getPosition() < 7667){
        leftDrive1.set(1);
        leftDrive2.set(1);
        rightDrive1.set(1);
        rightDrive2.set(1);
        intake.set(0);
        target = armUpPos;
      }else{
        leftDrive1.set(0);
        leftDrive2.set(0);
        rightDrive1.set(0);
        rightDrive2.set(0);
      }

      break;


      case driveForward:
        // 355.3 tics is one wheel rotation
        // wheel circumfrence is 18.85 inches
        // 18.85 tics per inch, unbelievable, i know
        // about 1800 tics is a solid backup

        //Drive Straight Forward 8 ft
        if (leftEncoder1.getPosition() < 1800) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
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
        if (leftEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(leftEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(leftEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if(!autoVar1 && leftEncoder1.getPosition() > -1138 && autoVar2){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(!autoVar1 && leftEncoder1.getPosition() < -1460 && autoVar2){
          target = armDownPos;
          intake.set(1);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && leftEncoder1.getPosition() > -1000 && autoVar3) {
          target = armUpPos;
          intake.set(0);
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar2 && rightEncoder1.getPosition() < -1460 && autoVar3){
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoVar3 = false;
        }


        if (!autoVar3 && leftEncoder1.getPosition() > -1138) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar3 && leftEncoder1.getPosition() < -1675){
          // Arrived at the Amp again
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          outtake.set(1);
          autoCount2 = 0;
        } else if(autoCount2 > 35){
          outtake.set(0);
          // Everything turns off, we're all done.
          // (Or are we?)
        }
      break;
      

      case blue3Note:
 // Position 1, score 2 notes
        // 1,027 ticks is 54.5 inches 
        if (leftEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(leftEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(leftEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if(!autoVar1 && leftEncoder1.getPosition() > -1138 && autoVar2){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(!autoVar1 && leftEncoder1.getPosition() < -1460 && autoVar2){
          target = armDownPos;
          intake.set(1);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar2 = false;
          // cruisin towards note #2
        }


        if (!autoVar2 && leftEncoder1.getPosition() > -1000 && autoVar3) {
          target = armUpPos;
          intake.set(0);
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar2 && rightEncoder1.getPosition() < -1460 && autoVar3){
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoVar3 = false;
        }


        if (!autoVar3 && leftEncoder1.getPosition() > -1138) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar3 && leftEncoder1.getPosition() < -1675){
          // Arrived at the Amp again
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          outtake.set(1);
          autoCount2 = 0;
        } else if(autoCount2 > 35){
          outtake.set(0);
        }

        if (!autoVar4 && leftEncoder1.getPosition() > -54 && autoVar5) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(!autoVar4 && leftEncoder1.getPosition() < -376 && autoVar5){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          target = armDownPos;
          intake.set(1);
          autoVar5 = false;
        }

        if (!autoVar5 && leftEncoder1.getPosition() > 84 && autoVar6) {
          target = armUpPos;
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar5 && leftEncoder1.getPosition() < -376 && autoVar6) {
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoVar6 = false;
        }

        if (!autoVar6 && leftEncoder1.getPosition() > -54) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar6 && leftEncoder1.getPosition() < -1675) {
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
          autoCount2 = 0;
        } else if(!autoVar6 && autoCount2 > 15){
          outtake.set(1);
        } else if(!autoVar6 && autoCount2 > 45){
          outtake.set(0);
        }

      break;


      case blue1Note:
// Position 1, score 1 note, leave zone last second
  if (leftEncoder1.getPosition() > -1027 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(leftEncoder1.getPosition() > -1349 && autoVar1) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(leftEncoder1.getPosition() > -1675 && autoVar1){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
          autoCount2 = 0;
        } else if(autoCount2 < 20 && autoVar1){
          // Arrives at Amp
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }else if(autoCount2 < 55 && autoVar1){
          outtake.set(1);
        }else if(autoVar1){
          outtake.set(0);
          leftDrive1.set(0.5);
          leftDrive2.set(0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
          autoVar1 = false;
        }


        if (!autoVar1 && leftEncoder1.getPosition() < -1450) {
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(0.5);
          rightDrive2.set(0.5);
        } else if(!autoVar1 && leftEncoder1.getPosition() < -1772){
          leftDrive1.set(-0.5);
          leftDrive2.set(-0.5);
          rightDrive1.set(-0.5);
          rightDrive2.set(-0.5);
        } else if(!autoVar1 && leftEncoder1.getPosition() < -2900){
          // Robot stops against back wall, waits, biding its time
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        } else if(!autoVar1 && autoCount > 700){
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
          /*  
              The Robot's machanations come to fruition
              Robot zooms out of the starting zone at mach 10
              + 2 points (:
          */
        }
      break;


      case bluePushMiddle3:
      // The mission of this auto is to get directly in the way
      // of a robot on the opposite alliance
      // that is running a 5, 6, 7 note auto


        if (rightEncoder1.getPosition() < 3054) {
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
        } else if(rightEncoder1.getPosition() < 5090){
          leftDrive1.set(.872);
          leftDrive2.set(.872);
          rightDrive1.set(1);
          rightDrive2.set(1);
          target = armDownPos;
          intake.set(1);
        }else if(rightEncoder1.getPosition() < 7000){
          leftDrive1.set(1);
          leftDrive2.set(1);
          rightDrive1.set(1);
          rightDrive2.set(1);
        }else if(rightEncoder1.getPosition() < 8257){
          intake.set(0);
          target = armUpPos;
        }else{
          leftDrive1.set(0);
          leftDrive2.set(0);
          rightDrive1.set(0);
          rightDrive2.set(0);
        }

      break;

      case bluePushMiddle2:
// annoying auto, starting in pos 2
      if(rightEncoder1.getPosition() < 2036){
        leftDrive1.set(1);
        leftDrive2.set(1);
        rightDrive1.set(1);
        rightDrive2.set(1);
        target = armDownPos;
      } else if(rightEncoder1.getPosition() < 6300){
        leftDrive1.set(0.938);
        leftDrive2.set(0.938);
        rightDrive1.set(1);
        rightDrive2.set(1);
        intake.set(1);
      }else if(rightEncoder1.getPosition() < 7667){
        leftDrive1.set(1);
        leftDrive2.set(1);
        rightDrive1.set(1);
        rightDrive2.set(1);
        intake.set(0);
        target = armUpPos;
      }else{
        leftDrive1.set(0);
        leftDrive2.set(0);
        rightDrive1.set(0);
        rightDrive2.set(0);
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
    driveTrain.tankDrive(joyDrive.getRawAxis(1), joyDrive.getRawAxis(5));

// Intake & Outtake code

if (joyDrive.getRawAxis(2) > 0.5) {
  intake.set(1);
} else {
    intake.set(0);
}

if (joyDrive.getRawAxis(3) > 0.5) {
  outtake.set(1);
} else {
    outtake.set(0);
}


// Arm Up & Down code
if (joyDrive.getRawButton(6)) {
  target = armUpPos;
} else if(joyDrive.getRawButton(5)){
  target = armDownPos;
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
  public void testPeriodic() {}
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
// Charles
// Jackson
// Jose
// Tom
// Maya
// Matteo
// Lachlan
// Aki
// Felix
// Rich
// Jessica
// Owen
// Andy
// Alejandra
// Jared
// Chris
// Mark from PG&E
// Ottie
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
// byeeee