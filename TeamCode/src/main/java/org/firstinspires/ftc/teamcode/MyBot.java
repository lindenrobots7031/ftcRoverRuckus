/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 * Version 0.1.0
 *
 * This class can be used to define all the specific hardware for a single robot.
 * This is a helper class with all the hardware for a 6 wheel, drop wheel bot.
 *
 * This hardware class assumes the following device names have been configured on the robot controller:
 * Note:  
 *
 * Motor channel:  Left drive motor:      						"leftMotor"
 * Motor channel:  Right drive motor      						"rightMotor"
 * Motor channel:  Left Intake arm rotatation motor:  			"rotateL"
 * Motor channel:  Right Intake arm rotatation motor:  			"rotateR"
 * Motor channel:  Intake arm linear slide extension:  			"extend"
 * Motor channel:  Intake sweeper:  							"intake"
 * Motor channel:  Lift system motor:		  					"lift"

 *
 * Color sensor:	Rev color sensor:							"senscolordist"
 * Distance sensor: Rev distance sensor:        				"senscolordist"
 * Range sensor:	MR range sensor								"mrrange"
 * Rev IMU:                                     				"imu"
 * Rev 2m laser range sensor									"range"
 * Digital Channel												"llim"

 * Servo channel:  Servo to deploy team marker:  				"marker"
 * Servo channel:  Servo to open L latch: 						"latchL"
 * Servo channel:  Servo to open R latch: 						"latchR" 
 * Servo channel:  Servo to raise/lower flag if nav target found: 	"nav"
 * Servo Channel:  Servo to turn camera                         "cam"
 * Cr Servo channel:  liftCRServo								"liftVex"
 */

public class MyBot {

    /* Public OpMode members. */
    // drive motors
    public DcMotor leftDrive;
    public DcMotor rightDrive;
	// other motors
    public DcMotor rotateArmL;
	public DcMotor rotateArmR;
    public DcMotor extendArm;
	public DcMotor intakeMotor;
    public DcMotor liftMotor;


    // rev color and distance sensor
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public BNO055IMU imu;
    public ModernRoboticsI2cRangeSensor mrRangeSensor;
	public Rev2mDistanceSensor las1;
	public DistanceSensor revRange;
	private DigitalChannel liftlimit;


	// servos
    public Servo    markerServo    = null;
    public Servo    latchServo   = null;
    public Servo    navFlag   = null;
    public Servo    camServo = null;
	//public CRServo	liftCRServo = null;

    public static final double START_SERVO       =  0.0 ;
    public static final double CLOSE_LATCH       =  1.0 ;
    public static final double OPEN_LATCH       =  0.0 ;	
    /*  use to set up predetermined servo and motor powers
    public static final double MID_SERVO       =  0.5 ;
    */

    /***counts per inch, wheel d, gear eduction etc...****/
    //static final double cpr = 1120; // counts per revolution,  neverest 40= 28 ppr(shaft) * 40 gear reduction, 28*40=1120
    static final double cpr = 28;
    static final double gearReduction = 20.0;	// neverest sport 20:1
    static final double wheelDiameter = 4.0;	//4 inch wheel diam
    static final double cpi = (cpr*gearReduction)/(wheelDiameter*3.1415); // gives overall counts per inch, multiply this by distance to travel to get new encoder count.
	private int movepulses = 0;
    private double leftarmpulsesperrev = 1440 * 9;  // motor ppr *  gear ratio
    private double rightarmpulsesperrev = 1440 * 9;
    private double latchpulsesperinch = 145.6 * 25.4 / 8;  //gobilda ppr * (number of lead screw turns per inch)
    private double slideliftpulsesperinch = 1120 / (2.0 * 3.1415); //neverest 40 / (D *pi) D is the slide pulley diameter
	

    public static boolean motorsOn = false; // boolean flag for workaround of motor isbusy
	public static boolean eventDone = true;

    double drive; // used in manualDrive
	double turn; // used in manualDrive
	double rawTurn;
    double maxPower;

    private LinearOpMode myOpMode;

    //Stuff for PID controller
    private double lastTime = 0;
    private double integral = 0.0;
    private double derivative = 0.0;
    private double previousError = 0;
    private double dt;
    private double finalError;
    // PID coefficients - change to tune controller
    /*
    public double kP = 0.05;
    public double kI = 0.01;
    public double kD = 0.05;
    */

    public double kP = 0.035;
    public double kI = 0.000001;
    public double kD = 0.005;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     HEADING_THRESHOLD       = 1.5;      // May need to tweak
    private static final double     TURN_SPEED              = 0.35;     // Nominal half speed for better accuracy.
    public double headingResetValue;

    /* local OpMode members. */
    //HardwareMap hwMap           =  null;
    private ElapsedTime runTime  = new ElapsedTime();
    private ElapsedTime servoRunTimer = new ElapsedTime();
	private ElapsedTime latchTime = new ElapsedTime();
	
	private double servoDelta = 0.075;
	private double servDelayTime = 0.1;
	private double servPos;
	private double adjustedDelta;
	
	static final double motorDelayTime = 0.05;
	static final double INCREMENT   = 0.01;
	
	double leftArmPwr;
    double rightArmPwr;
	double rotate;
	
	double ext;
	double extendPwr;

	double sweepIn;
	double sweepOut;
	double sweepTotal;
	double intakePwr;
	double latchPower = 0.0;
	int latchBrakeState = 0;
	
	boolean latchOpen;

    /* Constructor
     *	In the opmode instantiate this class using:
     *	MyDropWheelBot robot   = new MyDropWheelBot();
     */
    public MyBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
      leftDrive =  myOpMode.hardwareMap.get(DcMotor.class, "leftMotor");
      rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightMotor");
//		rotateArmL = myOpMode.hardwareMap.get(DcMotor.class, "rotateL");
//		rotateArmR = myOpMode.hardwareMap.get(DcMotor.class, "rotateR");
//		extendArm = myOpMode.hardwareMap.get(DcMotor.class, "extend");
//		liftMotor = myOpMode.hardwareMap.get(DcMotor.class, "lift");
//		intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intake");


        /**** commented out all color, distance sensor stuff *****/
        // get reference to the distance sensor integrated on color sensor.
//		las1 = myOpMode.hardwareMap.get(Rev2mDistanceSensor.class, "range");
//		liftlimit = myOpMode.hardwareMap.get(DigitalChannel.class,"llim");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
		
		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set motors to brake at 0 power
		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		//rotateArmL.setDirection(DcMotor.Direction.FORWARD);
        //rotateArmR.setDirection(DcMotor.Direction.REVERSE);
        //rotateArmL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rotateArmR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		//extendArm.setDirection(DcMotor.Direction.FORWARD);		
        //extendArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //rotateArmL.setPower(0);
        //rotateArmR.setPower(0);
        //extendArm.setPower(0);
		//intakeMotor.setPower(0);		

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

 //       markerServo  = myOpMode.hardwareMap.get(Servo.class, "marker");
 //       markerServo.setPosition(START_SERVO);
		
        // Define and initialize ALL installed servos.
        /*
        markerServo  = myOpMode.hardwareMap.get(Servo.class, "marker");
        latchServoL = myOpMode.hardwareMap.get(Servo.class, "latchL");
		latchServoR = myOpMode.hardwareMap.get(Servo.class, "latchR");
        navFlag = myOpMode.hardwareMap.get(Servo.class, "nav"); // servo to raise navigation flag - notifies driver that nav target visible 
        camServo = myOpMode.hardwareMap.get(Servo.class, "cam"); //servo to turn camera
		
		latchServoL.setDirection(Servo.Direction.FORWARD);
        latchServoR.setDirection(Servo.Direction.FORWARD);
		
        markerServo.setPosition(START_SERVO); // or whatever start position we need
        latchServoL.setPosition(START_SERVO);
		latchServoR.setPosition(START_SERVO);
		navFlag.setPosition(START_SERVO);
		camServo.setPosition(START_SERVO);
        */
		
//		stopReset();

    }

    public void initIMU(){
        //configure and initialize the imu hardware
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0x18; // to swap y and z
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        myOpMode.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        myOpMode.sleep(100); //Changing modes again requires a delay
        headingResetValue = getHeading();


    }
	
	public void controlArm(){
		if (!myOpMode.opModeIsActive()){return;}
		
		/**  weapons master gamepad  **/
		
		// use right joysitck to rotate mineral arm up and down
		rotate  =  myOpMode.gamepad2.right_stick_y;
        leftArmPwr    = Range.clip(rotate, -0.4, 0.4);
        rightArmPwr   = Range.clip(rotate, -0.4, 0.4);
		
		rotateArmL.setPower(leftArmPwr);
        rotateArmR.setPower(rightArmPwr);
		
		// use left joystick to extend/retract arm - linear slide
		ext  =  myOpMode.gamepad2.left_stick_y;
        extendPwr    = Range.clip(ext, -0.4, 0.4);
		extendArm.setPower(extendPwr);
		
		
		if (myOpMode.gamepad2.right_trigger > 0.5 && myOpMode.gamepad2.left_trigger < 0.5) {
            intakeMotor.setPower(1.0);
        }

        if (myOpMode.gamepad2.left_trigger > 0.5 && myOpMode.gamepad2.right_trigger < 0.5) {
            intakeMotor.setPower(-1.0);
        }

        if (myOpMode.gamepad2.left_trigger < 0.5 && myOpMode.gamepad2.right_trigger < 0.5){
			intakeMotor.setPower(0.0);
		}
		
	}
	
	public void controlLift(){	
		if (!myOpMode.opModeIsActive()){return;}
		
		
		
		/**  weapons master gamepad   **/
		if(myOpMode.gamepad2.left_bumper && !myOpMode.gamepad2.right_bumper){
			latchTime.reset();
			latchBrakeState = 1;
			//latchBrake.setPosition(latchBrakeOpen);
            //timref = tim1.seconds();
            //while (tim1.seconds() - timref < 0.25) {
            //}
			
			while(myOpMode.opModeIsActive() && myOpMode.gamepad2.left_bumper && !myOpMode.gamepad2.right_bumper && !liftlimit.getState()){
					if (latchTime.time() > motorDelayTime){
					latchPower += INCREMENT;
					latchPower = Range.clip(latchPower,-1.0,1.0);
					liftMotor.setPower(latchPower);
					latchTime.reset();
				}
					
				
			}
			
			
		}else if(myOpMode.gamepad2.right_bumper && !myOpMode.gamepad2.left_bumper){
			latchTime.reset();
			latchBrakeState = 1;
			//latchBrake.setPosition(latchBrakeOpen);
            //timref = tim1.seconds();
            //while (tim1.seconds() - timref < 0.25) {
            //}
			
			while(myOpMode.opModeIsActive() && myOpMode.gamepad2.right_bumper && !myOpMode.gamepad2.left_bumper && !liftlimit.getState()){
					if (latchTime.time() > motorDelayTime){
					latchPower += INCREMENT;
					latchPower = Range.clip(latchPower,-1.0,1.0);
					liftMotor.setPower(-latchPower);
					latchTime.reset();
				}
					
				
			}
			
		}else{
			liftMotor.setPower(0.0);
			if(latchBrakeState == 1){
			//	latchBrake.setPosition(latchBrakeClose);
			}
			latchPower = 0.0;
			latchBrakeState = 0;
		}
		
		
	}
		
	public void autoLift(boolean up){
		// @parameter up ..... pass in true to extend lift, false to retract lift
		if(up){
			while(myOpMode.opModeIsActive() && !liftlimit.getState()){
				liftMotor.setPower(1.0);
			}
		}else if(!up){
			while(myOpMode.opModeIsActive() && !liftlimit.getState()){
				liftMotor.setPower(-1.0);
			}
		}else {
			liftMotor.setPower(0.0);
		}
				
	}
	
	public void autoScore(){
	    //**********************AUTO SCORE MINERAL

       // if(myOpMode.gamepad1.x && scoremineralauto_pressed==0){
       //     scoremineralauto_pressed=1;
       //     autoscore();

       // }

       // if(!myOpMode.gamepad1.x && scoremineralauto_pressed==1) {
       //     scoremineralauto_pressed=0;
       // }
	}
	

	


    private void autoscore() {

        double scoredistance= slideextension();
        scoredistance=Math.cos(Math.asin(19/scoredistance))*scoredistance+6;
		
        if(scoredistance<6){
			scoredistance=6;
		}
		//get distance from laser range sensor
        double distance = las1.getDistance(DistanceUnit.MM) / 25.4 - scoredistance;
        int move = (int) (cpi * distance);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        runToPositionMotors();
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + move);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + move);
        leftDrive.setPower(0.4);
        rightDrive.setPower(0.4);

    }	
	

    public void manualDrive() {


        /** drivers gamepad **/

        // In this mode the Left stick moves the robot fwd & back.
        // The Right stick rotates CCW and CW.

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        ;

        double max;
        double error;
        double steer;
        double leftPwr;
        double rightPwr;
        boolean firstRun = true;


        if (myOpMode.gamepad1.right_stick_x == 0.0) {
            //reset lastTime
            resetPID();
        }

        //Get joystick input

        drive = -myOpMode.gamepad1.left_stick_y * 0.8; // reduce sensitivity
		
		rawTurn = myOpMode.gamepad1.right_stick_x;
		turn = rawTurn * 0.8; // reduce senitivity		
        turn = Range.clip(turn,-0.3,0.3); // clip it's max to 0.3



        // If left trigger is pressed, then speed/power is reduced to move slower, easier to drive precise
        if (myOpMode.gamepad1.left_trigger > 0.6) {
            //slow speed
            maxPower = 0.3;
        } else {
        //normal speed
            maxPower = 0.6;
        }
        leftPwr = Range.clip(drive + turn, -maxPower, maxPower);
        rightPwr = Range.clip(drive - turn, -maxPower, maxPower);

        //Apply power to motors
        setMotPower(leftPwr,rightPwr);

        //myOpMode.telemetry.addData("Mode", "running");
       // myOpMode.telemetry.addData("stick", "  turn=" + turn + "  drive=" + drive);
       // myOpMode.telemetry.addData("power", "  left=" + leftPwr + "  right=" + rightPwr);
        //myOpMode.telemetry.update();


        //** This is intended to correct heading while driving straight, i.e. no right stick input



        if(turn == 0 && drive !=0){
            // adjust relative speed based on heading error.
            // determine turn power based on +/- error
            error = getError(currentAngle);

            dt = ((System.nanoTime() * 10E-9) - lastTime); // find delta t for derivative
            lastTime = (System.nanoTime() * 10E-9);

            // if first cycle, there is no previous error so just return the proportional error
            if(firstRun || dt > 0.8){
                firstRun = false;
                integral = 0.0;
                derivative = 0.0;
            } else {

                double potentialGain = error * dt *kI; // Calculate potential gain from I term

                // ensure that I term doesn't grow too much
                if (potentialGain > 0.9){
                    integral = 0.9/kI;
                }else if(potentialGain > -0.9){
                    integral = integral + (error * dt);
                }else{
                    integral = -0.9/kI;
                }

                derivative = (error - previousError) / dt; // calulate derivative term
            }

            //calculate the final error for this cycle
            finalError = (kP * error + kI * integral + kD * derivative);

            // set the previous error for next cycle
            previousError = error;

            // limit range to -0.9 to 0.9 - not maxing out motors
            steer = Range.clip(finalError, -maxPower, maxPower);

            // if driving in reverse, the motor correction also needs to be reversed
            if (-myOpMode.gamepad1.left_stick_y < 0)
                steer *= -1.0;

            //leftPwr = drive - steer;
            //rightPwr = drive + steer;

            leftPwr = Range.clip(drive - steer, -maxPower, maxPower);
            rightPwr = Range.clip(drive + steer, -maxPower, maxPower);

            // set motor power with corrections
            setMotPower(leftPwr, rightPwr);

            // Display drive status for the driver.
        //    myOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        //    //telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
        //    myOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftPwr, rightPwr);
        //    myOpMode.telemetry.update();

        }


    }

    /*
     * Example usage in opmode
     * This will drive the robot in reverse 12 inches at 0.6 power:
     * MyDropWheelBot robot   = new MyDropWheelBot();
     * robot.gyroDrive(0.6, -12);
     */
    public void gyroDrive(double motPwr, double inches){
		if (!myOpMode.opModeIsActive()){return;}

        double  max;
        double  error;
        double  steer;
        double  leftPwr;
        double  rightPwr;
		boolean firstRun = true;

        double currentAngle = imu.getAngularOrientation().firstAngle;

        //set the motors to run to position and reset the PID
        runToPositionMotors();
		resetPID();

        motPwr = Range.clip(Math.abs(motPwr), 0.0, 1.0);

        // Drive fore aft
        // negative "inches" drives robot in reverse, positive drives forward
        movepulses = ((int) (inches * cpi));
        int newLeftTarget = leftDrive.getCurrentPosition() + movepulses;
        int newRightTarget = rightDrive.getCurrentPosition() + movepulses;
		
		int rightThreshold = Math.abs(newRightTarget) - (int)Math.round(Math.abs(newRightTarget) * 0.1); //set threshold to 10 percent of target
		int leftThreshold = Math.abs(newLeftTarget) - (int)Math.round(Math.abs(newLeftTarget) * 0.1);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
		
		int currentRightPos;
		int currentLeftPos;

        //Start moving
        setMotPower(motPwr, motPwr);
        motorsOn = true;
		eventDone = false;

        // keep looping while we are still active, and ALL motors are running.
        /** TODO  collision avoidance??  maybe mini state machine inside loop?
         //check for imminent collision using MR range sensor
         if (mrRangeSensor.getDistance(DistanceUnit.INCH) < 5){
         setMotPower(0, 0);

         //loop through sleep cycle for maximum of 6 seconds (4 * 1500ms = 6s)
         for (int i=0, 1 < 5; i++){
         myOpMode.sleep(1500);
         if(mrRangeSensor.getDistance(DistanceUnit.INCH) >= 5){
         setMotPower(motPwr, motPwr);
         i=5;
         }
         }
         }

         */ 

        while (myOpMode.opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()){
			if (!myOpMode.opModeIsActive()){return;}
			
            // adjust relative speed based on heading error.

            // determine turn power based on +/- error
            error = getError(currentAngle);

            dt = ((System.nanoTime() * 10E-9) - lastTime); // find delta t for derivative
            lastTime = (System.nanoTime() * 10E-9);

            // if first cycle, there is no previous error so just return the proportional error
            if(firstRun || dt > 0.8){
				firstRun = false;
                integral = 0.0;
                derivative = 0.0;
            } else {
				
				double potentialGain = error * dt *kI; // Calculate potential gain from I term
				
				// ensure that I term doesn't grow too much
				if (potentialGain > 0.9){
					integral = 0.9/kI;
				}else if(potentialGain > -0.9){ 
                    integral = integral + (error * dt);  
				}else{
					integral = -0.9/kI;
				}
				
                derivative = (error - previousError) / dt; // calulate derivative term
            }

            //calculate the final error for this cycle
            finalError = (kP * error + kI * integral + kD * derivative);

            // set the previous error for next cycle
            previousError = error;

            // limit range to -0.9 to 0.9 - not maxing out motors
            steer = Range.clip(finalError, -0.9, 0.9);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                steer *= -1.0;

            leftPwr = motPwr - steer;
            rightPwr = motPwr + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftPwr), Math.abs(rightPwr));
            if (max > 1.0){
                leftPwr /= max;
                rightPwr /= max;
            }
			
			currentRightPos = rightDrive.getCurrentPosition();
			currentLeftPos = leftDrive.getCurrentPosition();
			
			if(currentRightPos >= rightThreshold){
				leftPwr = Range.clip(leftPwr, -0.3, 0.3);
				rightPwr = Range.clip(rightPwr, -0.3, 0.3);
			}

            // set motor power with corrections
            setMotPower(leftPwr, rightPwr);

            // Display drive status for the driver.
            myOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            //telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            myOpMode.telemetry.addData("Actual",  "%7d:%7d",      leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            myOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftPwr, rightPwr);
            myOpMode.telemetry.update();
        }
		

        setMotPower(0,0);
        previousError = 0.0;
        motorsOn = false;
		eventDone = true;
        runWithoutEncoders();


    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed 	Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {
		if (!myOpMode.opModeIsActive()){return;}
		
		eventDone = false;

        /********************/
        //Do we need to get target angle from current heading????
        double newAngle = (getHeading() - angle);

        //double newAngle = angle;

		//reset the PID
		resetPID();

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && !onHeading(speed, newAngle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            myOpMode.telemetry.update();
        }

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param //angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return onTarget will be true or false
     */

    // **** onHeading modified to use PID controller
    // Don't use PCoeff, instead use kP, kI, kD

    boolean onHeading(double speed, double targetAngle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(targetAngle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {

            //** angle error is within our predefined threshold, stop motors **/
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
			motorsOn = false;
			eventDone = true;

        }else {

            dt = ((System.nanoTime() * 10E-9) - lastTime); // find delta t for derivative
            lastTime = (System.nanoTime() * 10E-9);

            // if first cycle, there is no previous error so just return the proportional error
            if(previousError == 0.0){
                integral = 0.0;
                derivative = 0.0;
            } else {
                integral += error * dt;  // calulate integral term
                derivative = (error - previousError) / dt; // calulate derivative term
            }

            //calculate the final error for this cycle
            finalError = (kP * error + kI * integral + kD * derivative);

            // set the previous error for next cycle
            previousError = error;


            // limit range to -0.9 to 0.9 - not maxing out motors
            steer = Range.clip(finalError, -0.9, 0.9);

            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
			
			motorsOn = true;
        }
		
			if(error < 10){
				rightSpeed = Range.clip(rightSpeed, -0.25, 0.25);
				leftSpeed = Range.clip(leftSpeed, -0.25, 0.25);
			}


        // Send corrected speeds to motors.
        setMotPower(leftSpeed,rightSpeed);


        // Display it for the driver.
        myOpMode.telemetry.addData("Target", "%5.2f", targetAngle);
        myOpMode.telemetry.addData("Angle", "%5.2f", getHeading());
        myOpMode.telemetry.addData("RelAngle", "%5.2f", getRelativeHeading());
        myOpMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        myOpMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;


    }

    private void setMotTarget(int lftar, int rftar) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + lftar);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + rftar);
    }


    private void setTargetPositionToCurrent(){
        // Set all motor positions to current
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());

    }


    public void driveStraight(double inches) {
        movepulses = ((int) (inches * cpi));

    }

    public void setMotPower(double lfp, double rfp) {
        leftDrive.setPower(lfp);
        rightDrive.setPower(rfp);
    }

    private void runToPositionMotors() {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runWithoutEncoders() {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetPID(){
        previousError = 0.0;
        lastTime = (System.nanoTime() * 10E-9);
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - imu.getAngularOrientation().firstAngle;

        //robotError = targetAngle - getHeading();
        robotError = targetAngle - getRelativeHeading();

         while (robotError > 180)  robotError -= 360;  // equivalent of robotError = robotError - 360,
         while (robotError <= -180) robotError += 360;
/*
        if (robotError > 180) {
            robotError -= 360; // equivalent of robotError = robotError - 360
        } else if(robotError <= -180) {
            robotError += 360; // equivalent of robotError = robotError + 360
        }
        */

        return robotError;
    }

    private double getRelativeHeading(){
        return getHeading() - headingResetValue;
    }

    public double getHeading() {

        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return imu.getAngularOrientation().firstAngle;
    }

    /**
     * returns desired steering force.  +/- 0.9 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -0.9, 0.9);
    }

    public double reducePower(double speed){
        return Range.clip(speed, -0.2, 0.2);
    }

    public void imuTest(){
        double checkHeading = getHeading();
        myOpMode.telemetry.addData("heading: ", checkHeading);
        myOpMode.telemetry.update();
    }

    public void setServo(Servo myServ, double pos){
        myServ.setPosition(pos);
    }
	
	public void setSlowServo(Servo myServ, double pos){
		if (!myOpMode.opModeIsActive()){return;}
		servPos = myServ.getPosition();
		runTime.reset();
		servoRunTimer.reset();
		if(servPos > pos){
			adjustedDelta = -1*servoDelta;
			while(myOpMode.opModeIsActive() && myServ.getPosition() >= pos && servoRunTimer.time() < 1.5){
				if (runTime.time() > servDelayTime){
					servPos += adjustedDelta;
					servPos = Range.clip(servPos,0,1);
					myServ.setPosition(servPos);
					runTime.reset();
				}

			}
		}else{
			adjustedDelta = servoDelta;
			while(myOpMode.opModeIsActive() && myServ.getPosition() <= pos && servoRunTimer.time() < 1.5){
				if (runTime.time() > servDelayTime){
					servPos += adjustedDelta;
					servPos = Range.clip(servPos,0,1);
					myServ.setPosition(servPos);
					runTime.reset();
				}
			}
		}	
	}

	public void deployMarker() {
        // Deploy marker
        setSlowServo(markerServo, 1.0);
        //setServo(markerServo,1.0);
        myOpMode.sleep(800);
        setServo(markerServo, 0.7);
        myOpMode.sleep(200);
        setServo(markerServo, 1.0);
        myOpMode.sleep(500);
        setServo(markerServo, 0.7);
        myOpMode.sleep(200);
        setServo(markerServo, 1.0);
        myOpMode.sleep(200);
        setServo(markerServo, 0.0);
    }
	
	private void stopReset() {
        rotateArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
	
	public void initAllMotors() {
		
		leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
		rotateArmL.setDirection(DcMotorSimple.Direction.FORWARD);
        rotateArmR.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		extendArm.setDirection(DcMotorSimple.Direction.FORWARD);
		liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
			
        rotateArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);		
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateArmL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotateArmR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
	
	public void brakeDriveMotors(){
		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // set motors to brake at 0 power
		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}
	
    private double slideextension() {
        return(extendArm.getCurrentPosition() / slideliftpulsesperinch * 2 + 13);
    }
    private void setlinearslide(double extension){
        extension=(extension-13)/slideliftpulsesperinch;
        extendArm.setTargetPosition((int) (extension));
    }	

}

