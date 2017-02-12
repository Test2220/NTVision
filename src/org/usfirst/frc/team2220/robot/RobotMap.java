package org.usfirst.frc.team2220.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//t edu.wpi.first.wpilibj.Solenoid;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static CANTalon rightMaster;
	public static CANTalon leftMaster;
	public static CANTalon rightSlave;
	public static CANTalon leftSlave;

	public static DoubleSolenoid clawzSolenoid;
	
	
	
	public static void init()
	{
		clawzSolenoid = new DoubleSolenoid(0,1);
		rightMaster = new CANTalon(1);
		leftMaster  = new CANTalon(2);
		rightSlave = new CANTalon(3);
		leftSlave  = new CANTalon(4);
		
		rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightMaster.configEncoderCodesPerRev(128);
		rightMaster.setAllowableClosedLoopErr(5);
		
		rightMaster.setEncPosition(0);

		rightMaster.setF(119.47); //encoder ticks per 100ms -> 9.34 RPS
		rightMaster.setPID(2.0, 0, 50); //i->0.001 //p->2.4
		rightMaster.setMotionMagicAcceleration(1000);   //RPM/S
		rightMaster.setMotionMagicCruiseVelocity(500); //RPM

		
		leftMaster.reverseOutput(false);
		//leftSlave.reverseOutput(false);
		
		leftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftSlave.set(leftMaster.getDeviceID());
		rightSlave.set(rightMaster.getDeviceID());
		
	}
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}