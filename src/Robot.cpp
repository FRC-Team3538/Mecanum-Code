#include <iostream>
#include <memory>
#include <string>
#include "AHRS.h"
#include "WPILib.h"
#include "math.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

// Mecanum Code Example 2017
//RJ-Libraries
//#include "MultiSpeedController.h"

class Robot: public frc::IterativeRobot {

	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();

	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	AHRS *ahrs;

	DigitalInput DiIn9, DiIn8, DiIn7;
	float OutputX, OutputY, OutputZ;

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//manipulator



public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), ahrs(NULL), DiIn9(9), DiIn8(8), DiIn7(7), OutputX(0), OutputY(0), OutputZ(0)  {


	}

	void RobotInit() {




		//drive command averaging filter
		OutputX = 0, OutputY = 0, OutputZ = 0;



		//from NAVX mxp data monitor example
		try { /////***** Let's do this differently.  We want Auton to fail gracefully, not just abort. Remember Ariane 5
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP, 200);
			ahrs->Reset();
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		// This gives the NAVX time to reset.
		// It takes about 0.5 seconds for the reset to complete.
		// RobotInit runs well before the autonomous mode starts,
		//		so there is plenty of time.
		Wait(1);
	}

	void AutonomousInit() override {


	}

	void TeleopInit() {

		OutputX = 0, OutputY = 0, OutputZ = 0;
		if (ahrs) {
			ahrs->ZeroYaw();
		}


	}

	void RobotPeriodic() {
		//links multiple motors together
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());






		//displays sensor and motor info to smartDashboard
		SmartDashboardUpdate();
	}
	void DisabledPeriodic() {

	}
	void AutonomousPeriodic() { // ADLAI - This stuff probably shouldn't be in here, we shouldn't need to change program after autonomous begins?

	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DriveCreepSpeed = 0.5;


		//high gear & low gear controls
		if (Drivestick.GetRawButton(6))
						// High gear press RH bumper
		if (Drivestick.GetRawButton(5))
					// Low gear press LH bumper

		// Temporary high gear when right trigger pushed
		if (Drivestick.GetRawAxis(3) > Deadband) {

		}

		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger



		//drive controls

		/* Use the joystick X axis for lateral movement, Y axis for forward
			 * movement, and Z axis for rotation. This sample does not use
			 * field-oriented drive, so the gyro input is set to zero.
			 */
		double SpeedLinear = Drivestick.GetRawAxis(1) * -1; // get Left Yaxis value (forward movement)
		double SpeedLateral = Drivestick.GetRawAxis(4) * -1; // get Right Xaxis value (side movement)
		double SpeedLRotate =  Drivestick.GetRawAxis(2) * -1; // get Left Rotation value
		double SpeedRRotate = Drivestick.GetRawAxis(3) * -1; // get Right Rotation value

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedLateral) < Deadband)
			SpeedLateral = 0.0;
		// Set dead band for R and L Rotate
		if (fabs(SpeedLRotate) < Deadband)
				SpeedLRotate = 0.0;
		if (fabs(SpeedRRotate) < Deadband)
				SpeedRRotate = 0.0;

		//Reduce turn speed when left trigger is pressed
		if (Drivestick.GetRawAxis(2) > Deadband) {
			SpeedLinear = SpeedLinear * DriveCreepSpeed;  // Reduce turn speed
			SpeedLateral = SpeedLateral * DriveCreepSpeed;  // Reduce drive speed
		}

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedLateral);
		OutputZ = (0.8 * OutputZ) + (0.2 * (SpeedRRotate - SpeedLRotate));

		//Read Gyro Angle
		double DrivegyroAngle = ahrs->GetAngle();

		Adrive.MecanumDrive_Cartesian(OutputX, OutputY, OutputZ, DrivegyroAngle);

		//drive
		if (Drivestick.GetRawButton(4)) {
			//when y button pushed
				}

		/*
		 * MANIP CODE
		 */

		// Turn on the shooter when right hand trigger is pushed
		if (OperatorStick.GetRawAxis(3) > Deadband) {

			} else {

			}



		// Turn on Kicker Wheel, conveyor, and agitators when right trigger is pressed
		if (OperatorStick.GetRawAxis(3) > Deadband ) {
		} else {
		}

		//Spin intake when left trigger is pushed
		if (OperatorStick.GetRawAxis(2) > Deadband) {
		} else {
		}

		// RH Bumper - Retract Intake
		if (OperatorStick.GetRawButton(6)) {
		}
		// LH Bumper - Deploy Intake
		if (OperatorStick.GetRawButton(5)) {
		}

		//A button for intake un-jam
		if (OperatorStick.GetRawButton(1)){
		}
		else{

		}





	}
	void SmartDashboardUpdate() {


		// PWM displays
		SmartDashboard::PutNumber("Drive L0 Output", DriveLeft0.Get());
		SmartDashboard::PutNumber("Drive L1 Output", DriveLeft1.Get());
		SmartDashboard::PutNumber("Drive L2 Output", DriveLeft2.Get());
		SmartDashboard::PutNumber("Drive R0 Output", DriveRight0.Get());
		SmartDashboard::PutNumber("Drive R1 Output", DriveRight1.Get());
		SmartDashboard::PutNumber("Drive R2 Output", DriveRight2.Get());


	}



	void TestPeriodic() {
		lw->Run();
	}

private:

}
;

START_ROBOT_CLASS(Robot)

// Read Driver Joystick
//double DRIV_CONTROL_LX = Drivestick.GetRawAxis(0), DriveCONTROL_LY = Drivestick.GetRawAxis(1);
//double DRIV_CONTROL_LTRIG = Drivestick.GetRawAxis(2), DRIV_CONTROL_RTRIG = Drivestick.GetRawAxis(3) ;
//double Drivestick.GetRawAxis(4) = Drivestick.GetRawAxis(4), DRIV_CONTROL_RY=Drivestick.GetRawAxis(5);
//bool DRIV_CONTROL_A = Drivestick.GetRawButton(1), DRIV_CONTROL_B = Drivestick.GetRawButton(2);
//bool DRIV_CONTROL_X = Drivestick.GetRawButton(3), Drivestick.GetRawButton(4)= Drivestick.GetRawButton(4);
//bool DRIV_CONTROL_LBUM = Drivestick.GetRawButton(5), Drivestick.GetRawButton(6) = Drivestick.GetRawButton(6);
//bool DRIV_CONTROL_BACK = Drivestick.GetRawButton(7), DRIV_CONTROL_START = Drivestick.GetRawButton(8);
//bool DRIV_CONTROL_LCLICK = Drivestick.GetRawButton(9), DRIV_CONTROL_RCLICK = Drivestick.GetRawButton(10);

// Read Operator Joystick
//double OPER_CONTROL_LX = OperatorStick.GetRawAxis(0), OPER_CONTROL_LY = OperatorStick.GetRawAxis(1);
//double OPER_CONTROL_LTRIG = OperatorStick.GetRawAxis(2), OPER_CONTROL_RTRIG = OperatorStick.GetRawAxis(3);
//double OPER_CONTROL_RX = OperatorStick.GetRawAxis(4), OPER_CONTROL_RY = OperatorStick.GetRawAxis(5);
//bool OPER_CONTROL_A = OperatorStick.GetRawButton(1), OPER_CONTROL_B = OperatorStick.GetRawButton(2);
//bool OPER_CONTROL_X = OperatorStick.GetRawButton(3), OPER_CONTROL_Y = OperatorStick.GetRawButton(4);
//bool OPER_CONTROL_LBUM = OperatorStick.GetRawButton(5), OPER_CONTROL_RBUM = OperatorStick.GetRawButton(6);
//bool OPER_CONTROL_BACK = OperatorStick.GetRawButton(7), OPER_CONTROL_START = OperatorStick.GetRawButton(8);
//bool OPER_CONTROL_LCLICK = OperatorStick.GetRawButton(9), OPER_CONTROL_RCLICK = OperatorStick.GetRawButton(10);

