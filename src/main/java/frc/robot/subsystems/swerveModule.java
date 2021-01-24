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
  //private static final double ENCODER_COUNT_PER_ROTATION = 4096.0;
  public double currentPosition;
  private AnalogInput analogIn;
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 0.5;
  //private setSwerveModule steerPID;
  private PIDController steerPID;

  private CANAnalog encoderVal;

  private double lastEncoderVal = 0;
  private double numTurns = 0;

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

    encoderVal = driveMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    //steerPID = new setSwerveModule();
    steerPID = new PIDController(STEER_P, STEER_I, STEER_D);
    steerPID.disableContinuousInput();
  }
    
  
  public void setSwerve(double angle, double speed) {

    SmartDashboard.putNumber("Incoming Angle", angle);
    double currentAngle = getAnalogVal() % 360;
    SmartDashboard.putNumber("CurAngle", currentAngle);
	
    /*The angle from the encoder is in the range [0, 360], but the swerve
    computations
    return angles in the range [-180, 180], so transform the encoder angle to
    this range*/

    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    SmartDashboard.putNumber("CurAngle -180 to 180", currentAngle);
	
    double targetAngle = -angle; //-angle;
    SmartDashboard.putNumber("TargetAngle", targetAngle);

    double deltaDegrees = targetAngle - currentAngle;
    SmartDashboard.putNumber("DeltaDegrees", deltaDegrees);

    // If we need to turn more than 180 degrees, it's faster to turn in the opposite
    // direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    SmartDashboard.putNumber("DeltaDegCorrected", deltaDegrees);
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

    steerPID.setSetpoint(targetPosition);
    double steerOutput = steerPID.calculate(currentAngle);
    steerOutput = MathUtil.clamp(steerOutput, -1, 1);
    SmartDashboard.putNumber("Steer Output", steerOutput);

    driveMotor.set(speed);
    steerMotor.set(steerOutput);

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
    return encoderVal.getPosition();
  }

  public double getVolts(){
    return encoderVal.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
