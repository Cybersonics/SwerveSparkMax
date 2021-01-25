/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.subsystems.setSwerveModule;

public class swerveModule extends SubsystemBase {
  /**
   * Creates a new swerveModule.
   */
  
  private static final double STEER_P = .0035, STEER_I = 0.00003, STEER_D = 0.0000;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double currentPosition;
  private AnalogInput analogIn;
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 0.5;
  private PIDController steerPID;
  private CANPIDController steerCANPID;

  private CANAnalog steerAnalogEncoder;
  private CANEncoder steerMotorEncoder;

  private double lastEncoderVal = 0;
  private double numTurns = 0;
  private double maxEncoderVolts = 3.3;

  public swerveModule(int analogNum, int steerNum, int driveNum, boolean invertDrive, boolean invertSteer) {

    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setInverted(invertDrive);
		driveMotor.setOpenLoopRampRate(RAMP_RATE);
		driveMotor.setIdleMode(IdleMode.kBrake);

    analogIn = new AnalogInput(analogNum);

    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
		steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);

    steerMotorEncoder = steerMotor.getEncoder();

    steerAnalogEncoder = steerMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    //steerPID = new PIDController(STEER_P, STEER_I, STEER_D);
    //steerPID.disableContinuousInput();

    
    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    steerCANPID = steerMotor.getPIDController();
        /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    //steerCANPID.setFeedbackDevice(encoderVal);

    // PID coefficients
    kP = 0.08; 
    kI = 0.000;
    kD = 0.0000; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    steerCANPID.setP(kP);
    steerCANPID.setI(kI);
    steerCANPID.setD(kD);
    steerCANPID.setIZone(kIz);
    steerCANPID.setFF(kFF);
    steerCANPID.setOutputRange(kMinOutput, kMaxOutput);

    steerMotorEncoder.setPosition(getAnalogVal()/20);
  }
    
  
  public void setSwerve(double angle, double speed) {

    SmartDashboard.putNumber("Incoming Angle", angle);
    double currentAngle = getSteerMotorEncoder();
    SmartDashboard.putNumber("CurAngle", currentAngle);

    double targetAngle = -angle; //-angle;
    SmartDashboard.putNumber("TargetAngle", targetAngle);

    double deltaDegrees = targetAngle - currentAngle;
    SmartDashboard.putNumber("DeltaDegrees", deltaDegrees);

    // If we need to turn more than 180 degrees, it's faster to turn in the opposite
    // direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }

    // If we need to turn more than 90 degrees, we can reverse the wheel direction
    // instead and
    // only rotate by the complement

    //if (Math.abs(speed) <= MAX_SPEED){
      if (Math.abs(deltaDegrees) > 90.0) {
      	deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      	speed = -speed;
      }
	// }

    double targetPosition = currentAngle + deltaDegrees;
    SmartDashboard.putNumber("TargetPosition", targetPosition);

    double scaledPosition = (targetPosition / 20);
    SmartDashboard.putNumber("Steer Output", scaledPosition);


    driveMotor.set(speed);
    steerCANPID.setReference(scaledPosition, ControlType.kPosition);

    SmartDashboard.putNumber("currentPosition", currentAngle);
  }

  public double getAnalogIn() {
    double test = analogIn.pidGet();
    double scaledEncoder = (test / RobotController.getVoltage5V()) * 360;
    if ((lastEncoderVal % 360) > 270 && (scaledEncoder % 360) < 90) {
      numTurns += 1;
    }
    if ((lastEncoderVal % 360) < 90 && (scaledEncoder % 360) > 270) {
      numTurns -= 1;
    }
    lastEncoderVal = scaledEncoder;
    scaledEncoder += (360 * numTurns);
    return scaledEncoder;
  }

  //Using discontinous input 
  public double getAnalogVal(){
    double inValRaw = analogIn.pidGet();
    double scaledEncoder = (inValRaw / RobotController.getVoltage5V()) * 360;
    return scaledEncoder;
  }
  
  public double getDriveEncoder() {
    return driveMotor.getEncoder().getPosition();
  }
  
  public void setDriveEncoder(double position) {
    driveMotor.getEncoder().setPosition(position);
  }
  
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }
  
  public double getDriveSpeed() {
    return driveMotor.get();
  }
  
  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }
  
  public void setSteerSpeed(double speed) {
    steerMotor.set(speed);
  }
  
  public double getSteerSpeed() {
    return steerMotor.get();
  }

  public double nominalVolts(){
    return steerMotor.getVoltageCompensationNominalVoltage();
  }

  public double getAnalog(){
    double posRaw = steerAnalogEncoder.getPosition();
    if (posRaw > maxEncoderVolts) {
      maxEncoderVolts = posRaw;
    }
    double scaledEncoder = (posRaw / maxEncoderVolts) * 360;
  
    if ((lastEncoderVal % 360) > 270 && (scaledEncoder % 360) < 90) {
      numTurns += 1;
    }
    if ((lastEncoderVal % 360) < 90 && (scaledEncoder % 360) > 270) {
      numTurns -= 1;
    }
    lastEncoderVal = scaledEncoder;
    scaledEncoder += (360 * numTurns);
    return scaledEncoder;


  }

  public double getVolts(){
    return steerAnalogEncoder.getVoltage();
  }

  public double getSteerMotorEncoder(){
    double posRaw = steerMotorEncoder.getPosition() * 20;
    return posRaw;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
