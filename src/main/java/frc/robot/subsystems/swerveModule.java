/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.controller.PIDController; //Use for Roborio PID
//import edu.wpi.first.wpiutil.math.MathUtil; // Use for RoboRio PID

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

//import frc.robot.subsystems.setSwerveModule;

public class swerveModule extends SubsystemBase {
  /**
   * Creates a new swerveModule.
   */
  
  public double currentPosition;
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 0.5;

  //Use the following two line if using PID in RoboRIO
  //private static final double STEER_P = .0035, STEER_I = 0.00003, STEER_D = 0.0000;
  //private PIDController steerPID;

  private AnalogInput analogIn; //Set up analog input for Roborio
 
  //Use the following two line if using PID in Spark Max
  private CANPIDController steerCANPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private CANAnalog steerAnalogEncoder; //Set up analog input to Spark Max
  private CANEncoder steerMotorEncoder; //Set up integrated Steering motor encoder in Spark Max/Neo
  private CANEncoder driveMotorEncoder; //Set up integrated Drive motor encoder in Spark Max/Neo
  
  private double lastEncoderVal = 0;
  private double numTurns = 0;
  private double maxEncoderVolts = 3.3;
  private static final double STEER_MOTOR_RATIO = 18; //Ratio between steering motor and Swerve pivot

  private double loopCounter = 0;
  private static final double MAXSTEERERROR = 5;

  public swerveModule(int analogNum, int steerNum, int driveNum, boolean invertDrive, boolean invertSteer) {

    //Create and configure a new Drive motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setInverted(invertDrive);
		driveMotor.setOpenLoopRampRate(RAMP_RATE);
		driveMotor.setIdleMode(IdleMode.kBrake);

    //Create and configure an analog input on a roborio port
    analogIn = new AnalogInput(analogNum);

    //Create and configure a new Steering motor
    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
		steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);

    //Create the built in motor encoders
    steerMotorEncoder = steerMotor.getEncoder();
    driveMotorEncoder = driveMotor.getEncoder();


    //Create an analog encoder to read values from Spark Max breakout board
    steerAnalogEncoder = steerMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

    //steerPID = new PIDController(STEER_P, STEER_I, STEER_D); // Use for RoboRio PID
    //steerPID.disableContinuousInput(); // Use for RoboRio PID
    
    /**
     * In order to use PID functionality for a Spark Max controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    steerCANPID = steerMotor.getPIDController();

    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    //steerCANPID.setFeedbackDevice(steerAnalogEncoder);
    
    // PID coefficients for a Spark Max
    kP = 0.08; 
    kI = 0.000;
    kD = 0.0000; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients into Spark Max
    steerCANPID.setP(kP);
    steerCANPID.setI(kI);
    steerCANPID.setD(kD);
    steerCANPID.setIZone(kIz);
    steerCANPID.setFF(kFF);
    steerCANPID.setOutputRange(kMinOutput, kMaxOutput);

    // Take a reading from the absolute encoder and preload the built in
    //motor encoder with the position when the system starts up
    setSteerMotorEncoder();  
  }
   
  public void setSwerve(double angle, double speed) {
    
    //double currentAngle = getAnalogIn() % 360.0; // Use for RoboRio PID
    double currentAngle = getSteerMotorEncoder();
    double targetAngle = angle; //-angle;
    double deltaDegrees = targetAngle - currentAngle;

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
	  //}
    //Add change in position to current position
    double targetPosition = currentAngle + deltaDegrees; 
    //Scale the new position to match the motor encoder
    double scaledPosition = (targetPosition / (360/STEER_MOTOR_RATIO)); 

    //steerPID.setSetpoint(targetPosition); // Use for RoboRio PID
    //double steerOutput = steerPID.calculate(currentAngle); // Use for RoboRio PID
    //steerOutput = MathUtil.clamp(steerOutput, -1, 1); // Use for RoboRio PID


    driveMotor.set(speed);
    steerCANPID.setReference(scaledPosition, ControlType.kPosition);
    //steerMotor.set(steerOutput); // Use for RoboRio PID

    //Use Dashboard items to help debug
    // SmartDashboard.putNumber("Incoming Angle", angle);
    // SmartDashboard.putNumber("CurAngle", currentAngle);
    // SmartDashboard.putNumber("TargetAngle", targetAngle);
    // SmartDashboard.putNumber("DeltaDegrees", deltaDegrees);
    // SmartDashboard.putNumber("TargetPosition", targetPosition);
    // SmartDashboard.putNumber("Steer Output", scaledPosition);
    // SmartDashboard.putNumber("currentPosition", currentAngle);
    // SmartDashboard.putNumber("Steer Output", steerOutput);
  }

  
  //Get analog reading from Roborio port. Value is nominal 0 to 5 vdc so divide by 
  //the Roborio bus voltage to get percentage of the range. Multipy the range percentage
  // by 360 to scale and read in degrees. Take a snap shot of the value and use it 
  //to establish if the encoder is going past the 360 to 0 transition point.
  //If the transition is 360 to 0 add 1 to the number of turns.
  //If the transition is 0 to 360 subtract 1 from the number of turns.
  //The number of turns is then multiplied by 360 and added to the position
  //to create a "continuous" encoder
  public double getAnalogIn() {
    double inValRaw = analogIn.pidGet();
    double scaledEncoder = (inValRaw / RobotController.getVoltage5V()) * 360;
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

  //Get analog reading from Roborio port. Value is nominal 0 to 5 vdc so divide by 
  //the Roborio bus voltage to get percentage of the range. Multipy the range percentage
  // by 360 to scale to read in degrees. This will be a discontinous input encoder 
  public double getAnalogVal(){
    double inValRaw = analogIn.pidGet();
    double scaledEncoder = (inValRaw / RobotController.getVoltage5V()) * 360;
    return scaledEncoder;
  }
  
  //Get the built in Spark/Neo Drive motor encoder position. Value is in motor revolutions.
  public double getDriveEncoder() {
    return driveMotor.getEncoder().getPosition();
  }
  
  //Set the position value of the Spark/Neo Drive motor encoder position. Position is in 
  //motor revolutions.
  public void setDriveEncoder(double position) {
    driveMotor.getEncoder().setPosition(position);
  }
  
  //Set the drive motor speed from -1 to 1 
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }
  
  //Get the drive motor speed.
  public double getDriveSpeed() {
    return driveMotor.get();
  }
  
  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }
  
  //Get analog reading from Spark Max Break out board. Value is nominal 0 to 3.3 vdc so divide by 
  //the maxEncoderVolts. Check to see if the input is higher then 3.3 and if so make that
  //the new maxEncodervolts. This corrects the error from the break out boards. Multipy the range
  //by 360 to scale and read in degrees. Take a snap shot of the value and use it 
  //to establish if the encoder is going past the 360 to 0 transition point.
  //If the transition is 360 to 0 add 1 to the number of turns.
  //If the transition is 0 to 360 subtract 1 from the number of turns.
  //The number of turns is then multiplied by 360 and added to the position
  //to create a "continuous" encoder
  public double getSteerAnalogEncoder(){
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

  //Get the built in steering motor encoder value and scale to read in degrees 
  public double getSteerMotorEncoder(){
    double posRaw = steerMotorEncoder.getPosition() * (360/STEER_MOTOR_RATIO); //steer motor encoder in degrees
    //double scaledAnalog = getSteerAnalogEncoder();// analog in from Spark Max Breakout board in degrees
    double scaledAnalog = getAnalogIn(); //analog in from Roborio in degrees
    //Periodic check to see if motor encoder aligns with analog encoder
    if (loopCounter <= 10) {
      loopCounter =+ 1;
    }
    else {
      loopCounter = 0;
      //If motor position is not within limits reset the motor encoder
      if ((posRaw > (scaledAnalog + MAXSTEERERROR)) || 
        (posRaw < (scaledAnalog - MAXSTEERERROR))) {
        steerMotor.getEncoder().setPosition(scaledAnalog);
        posRaw = scaledAnalog;
      }
    }
    return posRaw;
  }

  //Set the position value of the Spark/Neo Steer motor encoder position based on the 
  //position of absolute encoder connected to the Roborio.
  public void setSteerMotorEncoder(){
    double inValRaw = analogIn.pidGet();
    double scaledEncoder = (inValRaw / RobotController.getVoltage5V()) * STEER_MOTOR_RATIO;
    steerMotor.getEncoder().setPosition(scaledEncoder);
  }

  //Set the position value of the Spark/Neo Steer motor encoder position. The position 
  //is in motor revlutions.
  public void setSteerEncoder(double position){
    steerMotor.getEncoder().setPosition(position);
  }

  //Set the steer motor speed from -1 to 1 
  public void setSteerSpeed(double speed) {
    steerMotor.set(speed);
  }
  
  //Get the steer motor speed.
  public double getSteerSpeed() {
    return steerMotor.get();
  }

  public void stopSteerMotor(){
    steerMotor.stopMotor();
  }

  //Get the analog reading from Spark Max Break out board measured in volts. 
  public double getAnalogEncoderVolts(){
    return steerAnalogEncoder.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
