/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.commands.FieldCentricSwerveDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class Drive extends SubsystemBase {

	private static swerveModule frontLeft;
	private static swerveModule backLeft;
	private static swerveModule frontRight;
	private static swerveModule backRight;

	public static AHRS navX; 
    public double heading;
    public double angle;

	private static final double WHEEL_BASE_LENGTH = 22; // 28.0;
	private static final double WHEEL_BASE_WIDTH = 23.5; // 22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;
	
	private static final double WHEEL_DIAMETER = 4.0;
	// TO DO: Correct equation that uses MAX_SPEED
	public static final double MAX_SPEED = 0.3; // Max speed is 0 to 1
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.63);
	public static final double DEADZONE = 0.08;

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = false;
	private final boolean invertSteer = true;

	private ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");
	private NetworkTableEntry lfSetAngle = driveTab.addPersistent("LF Set Angle", 0).getEntry();
	private NetworkTableEntry lbSetAngle = driveTab.addPersistent("LB Set Angle", 0).getEntry();
	private NetworkTableEntry rfSetAngle = driveTab.addPersistent("RF Set Angle", 0).getEntry();
	private NetworkTableEntry rbSetAngle = driveTab.addPersistent("RB Set Angle", 0).getEntry();

	public Drive() {
		
		// frontLeft = new setSwerveModule(Constants.FL_STEER_ENCODER, Constants.FL_STEER_MOTOR, 
		// Constants.FL_DRIVE_MOTOR, invertDrive, invertSteer);

		// backLeft = new setSwerveModule(Constants.BL_STEER_ENCODER, Constants.BL_STEER_MOTOR, 
		// Constants.BL_DRIVE_MOTOR, invertDrive, invertSteer);

		frontRight = new swerveModule(Constants.FR_STEER_ENCODER, Constants.FR_STEER_MOTOR, 
		Constants.FR_DRIVE_MOTOR, invertDrive, invertSteer);

		// backRight = new setSwerveModule(Constants.BR_STEER_ENCODER, Constants.BR_STEER_MOTOR, 
		// Constants.BR_DRIVE_MOTOR, invertDrive, invertSteer);	

		navX = new AHRS(SPI.Port.kMXP);
		NetworkTableEntry myBool = 
		Shuffleboard.getTab("DriveTab").add("test001", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
	}

	// public void stopFrontLeft() {
	// 	frontLeft.stopDriveMotor();
	// }

	// public void stopBackLeft() {
	// 	backLeft.stopDriveMotor();
	// }

	public void stopFrontRight() {
		frontRight.stopDriveMotor();
	}

	// public void stopBackRight() {
	// 	backRight.stopDriveMotor();
	// }

	// public void setFrontLeft(double speed) {
	// 	frontLeft.setDriveSpeed(speed);
	// }

	// public void setBackLeft(double speed) {
	// 	backLeft.setDriveSpeed(speed);
	// }

	public void setFrontRight(double speed) {
		frontRight.setDriveSpeed(speed);
	}

	// public void setBackRight(double speed) {
	// 	backRight.setDriveSpeed(speed);
	// }

	public void swerveDrive(double strafe, double forward, double omega, boolean deadStick) {
		double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
		double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);

		// Compute the constants used later for calculating speeds and angles
		double A = strafe - omegaL2;
		double B = strafe + omegaL2;
		double C = forward - omegaW2;
		double D = forward + omegaW2;

		// Compute the drive motor speeds
		double speedFL = speed(B, D);
		double speedBL = speed(A, D);
		double speedFR = speed(B, C);
		double speedBR = speed(A, C);

		/* ... and angles for the steering motors
			Set the drive to face straight ahead and then either mechanically
			set the encoders to read zero, or mathematically correct the angle
			by reading the encoder value when the drive is pointed straight 
			ahead and adding or subtracting that value from the reading
		*/

		/* Get offset values from the driver station using NetworkTables. 
			Values are then input to "calibrate" the position of the drives
			mathematically rather then by mechanically positioning the drives
			and physically setting the encoder to zero.
		*/ 
		
		double lfOffset = lfSetAngle.getDouble(0.0);
		double lbOffset = lbSetAngle.getDouble(0.0);
		double rfOffset = rfSetAngle.getDouble(0.0);
		double rbOffset = rbSetAngle.getDouble(0.0);

		/* When drives are mechanically calibrated for zero position on encoders they
			can be at 90 degrees to the front of the robot. Adding or subtracting 90 degrees
			to the steering calculation can be used offset for initial position/calibration
			of the drives.
		
			For swerve and steer drives constants are 90 degrees out of phase when
			they are inserted in frames sideways.
			angleFL - 90
			angleBL + 90
			angleFR - 90
			angleBR + 90
		*/

		double angleFL = angle(B, D) + Constants.FL_STEER_OFFSET + lfOffset; 
		double angleBL = angle(A, D) + Constants.BL_STEER_OFFSET + lbOffset;
		double angleFR = angle(B, C) + Constants.FR_STEER_OFFSET + rfOffset;
		double angleBR = angle(A, C) + Constants.BR_STEER_OFFSET + rbOffset;
		// Compute the maximum speed so that we can scale all the speeds to the range
		// [0, 1]
		double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

		if (deadStick){

			//frontLeft.setSteerSpeed(0);
			//frontLeft.setDriveSpeed(0);
			//backLeft.setSteerSpeed(0);
			//backLeft.setDriveSpeed(0);
			frontRight.setSteerSpeed(0);
			frontRight.setDriveSpeed(0);
			//backRight.setSteerSpeed(0);
			//backRight.setDriveSpeed(0);

		} else {
			// Set each swerve module, scaling the drive speeds by the maximum speed
			// frontLeft.setSwerve(angleFL, speedFL / maxSpeed);
			// backLeft.setSwerve(angleBL, speedBL / maxSpeed);
			frontRight.setSwerve(angleFR, speedFR / maxSpeed);
			// backRight.setSwerve(angleBR, speedBR / maxSpeed);
		}
	}

	private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}

	public static double[] getEncoderVal() {
		double[] values = new double[] {
			//frontLeft.getAnalogIn(),
			//backLeft.getAnalogIn(),
			frontRight.getAnalogIn(),//,
			frontRight.getAnalogVal()
			//backRight.getAnalogIn()
		};

		return values;
	}

	public static double getNominalVoltage(){
		return frontRight.nominalVolts();
	}

	public static double getRFPosition(){
		return frontRight.getAnalog();
	}

	public static double getRFVolts(){
		return frontRight.getVolts();
	}

	public static double getRFEncoder(){
		return frontRight.getMotorEncoder();
	}

	public double getNavHeading() {
        this.heading = navX.getFusedHeading();
        return heading;
     }

     public double getNavAngle() {
         this.angle = navX.getAngle();
         return angle;
	 }
	 
	 public void zeroNavHeading() {
		 navX.zeroYaw();
	 }


	public void initDefaultCommand() {
		//setDefaultCommand(new FieldCentricSwerveDrive());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

	}
}
