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
 * 
 */

public class MyDropWheelBot {

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
    public Servo    latchServoL   = null;
    public Servo    latchServoR   = null;
    public Servo    navFlag   = null;
    public Servo    camServo = null;
    public Servo    latchBrake = null;
	//public CRServo	liftCRServo = null;

    public static final double START_SERVO       =  0.0 ;
    public static final double CLOSE_LATCH       =  1.0 ;
    public static final double OPEN_LATCH       =  0.0 ;	
    /*  use to set up predetermined servo and motor powers
    public static final double MID_SERVO       =  0.5 ;
    */

    /***Counts per inch, wheel d, gear eduction etc...****/
    //static final double cpr = 1120; // counts per revolution,  neverest 40= 28 ppr(shaft) * 40 gear reduction, 28*40=1120
    static final double cpr = 28;
    static final double gearReduction = 20.0;	// neverest sport 20:1
    static final double wheelDiameter = 4.0;	//4 inch wheel diam
    static final double cpi = (cpr*gearReduction)/(wheelDiameter*3.1415); // gives overall counts per inch, multiply this by distance to travel to get new encoder count.
	private int movepulses = 0;
    private double leftarmpulsesperrev = 1440 * 9;  // motor ppr *  gear ratio
    private double rightarmpulsesperrev = 1440 * 9;
    private double latchpulsesperinch = 145.6 * 25.4 / 8;  //gobilda ppr * (number of lead screw turns per inch)
    private double slideliftpulsesperinch = 3.7*7*27 / (2.0 * 3.1415); //neverest 40 / (D *pi) D is the slide pulley diameter
	

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
	// used to tune the PID controller
    private double kP = 0.001;
    private double kI = 0.00;
    private double kD = 0.000002;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked
    private static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     HEADING_THRESHOLD       = 1.5;      // May need to tweak
    private static final double     TURN_SPEED              = 0.35;     // Nominal half speed for better accuracy.
    private double headingResetValue;
    Orientation lastAngles = new Orientation();
    private double globalAngle;
	private double minSpeed = 0.25;
	private double addSpeed = 0.08; // Proportional component
	private double gyroRange = 1.75;

    /* local OpMode members. */
    //HardwareMap hwMap           =  null;
    private ElapsedTime runTime  = new ElapsedTime();
    private ElapsedTime servoRunTimer = new ElapsedTime();
	private ElapsedTime latchTime = new ElapsedTime();
	private ElapsedTime tim1 = new ElapsedTime();
	
	private double servoDelta = 0.075;
	private double servDelayTime = 0.1;
	private double servPos;
	private double adjustedDelta;
	
	static final double motorDelayTime = 0.05;
	static final double INCREMENT   = 0.01;
	
	private double speedCalFactor = 0.6;  //speed cal factor
	
	private double leftArmPwr;
    private double rightArmPwr;
	private double rotate;
	
	private double ext;
	private double extendPwr;

	private double sweepIn;
	private double sweepOut;
	private double sweepTotal;
	private double intakePwr;
	private double latchPower = 0.0;
	private int latchBrakeState = 0;
	
	public boolean latchOpen;
	
	    //DECLARE CONTROL LOGIC VARIABLES


    private int holdslide_pressed = 0;
    private int holdslide_on = 0;
    private int holdslide_released = 0;
    private int holdarm_pressed = 0;
    private int holdarm_on = 0;
    private int holdarm_released = 0;
    private int scoremineralauto_pressed=0;
    private int scoremineralauto_on=0;
    private int scoremineralauto_release=0;
	
    private double armangle = 0;
    private double armpivotheight = 16;
    private double maxslidelength = 38;
    private double minslideangle = Math.acos(armpivotheight / maxslidelength);
    private double mincountforslide = minslideangle * leftarmpulsesperrev / (2 * 3.1415);

    private double slideMotorMaxPower = 0.7;
    private double slideMotorOutPower = 1.0;
    private double slideMotorInPower = 0.3;
    private double leftArmMotorMaxPower = 0.4;
    private double rightArmMotorMaxPower = 0.4;
    private double scale = 0;
    private double slidepower = 0;
    private int holdrelief = 0;
    private int maxarmtargetposition=(int) (210/360*1440);
    private int minarmtargetposition=(int) (1440/4);
    private int armtargetposition=0;
    private double latchPowerMax=0.7;
    private double fullextension=maxslidelength;
    private double armtowerheight=armpivotheight;
    private double heighttoscore=30-armtowerheight;
    private double lenthofintakesystem=7;
    private double intakeextension=fullextension-lenthofintakesystem;
    private double scoredistance_fullextension=Math.sqrt(intakeextension*intakeextension-heighttoscore*heighttoscore); // inches away from score unit	
	
	private int switchdrive_pressed=0;
    private int switchdrive_toggle=1;
    private int xdelivermineral_pressed = 0;
	
	private int dpad_mode = 0;
    //private int d_forward = 0;
    //private int d_backward = 0;
    //private int d_right = 0;
    //private int d_left = 0;
    
    private int dpad_increment = 40;
    private int switchdrive=1;
    private int switchdrive_on;
    private int switched_toggle;
    private double leftarmstart;
    private double rightarmstart;
    private double slidestart;
    private double latchstart;

    private double leftLatchServoOpen = 0.0;  //off
    private double rightLatchServoOpen = 0.0;
    private double latchBrakeOpen = 0.0;
    private double markerArmOpen = 0.0;
    private double markerGripperOpen = 0.0;

    private double leftLatchServoClose = 1.0;  //on
    private double rightLatchServoClose = 1.0;
    private double latchBrakeClose = 1.0;
    private double markerArmCLose = 1.0;
    private double markerGripperClose = 1.0;

	private int correctCount = 0;

	VuForiaClass vuf;


    /* Constructor
     *	In the opmode instantiate this class using:
     *	MyDropWheelBot robot   = new MyDropWheelBot();
     */
    public MyDropWheelBot(){

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
//        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        markerServo  = myOpMode.hardwareMap.get(Servo.class, "marker");
        markerServo.setPosition(START_SERVO);
		
        // Define and initialize ALL installed servos.
        latchBrake = myOpMode.hardwareMap.get(Servo.class, "latchL");

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

		stopReset();
        runToPositionMotors();

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



        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        //byte AXIS_MAP_CONFIG_BYTE = 0x18; // to swap y and z
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



        headingResetValue = getAbsoluteHeading();


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
//*****************//*****************//*****************//*****************//*****************//*****************
		
		        //******************OPERATE SLIDE only ACTIVE IF IN THE AIR
        slidepower = myOpMode.gamepad2.left_stick_y;
        if (rotateArmL.getCurrentPosition() > mincountforslide || rotateArmR.getCurrentPosition() > mincountforslide) {
            if (slidepower > slideMotorOutPower) {
                slidepower = slideMotorOutPower;
            } else if (slidepower < slideMotorInPower) {
                slidepower = slideMotorInPower;
            } else {
                slidepower = 0.0;
            }
        }


        if (myOpMode.gamepad2.a && holdslide_pressed == 0) {    //press a to hold slide position
            holdslide_pressed = 1;
            switch (holdslide_released) {
                case 1:
                    holdslide_released = 0;
                    holdslide_on = 0;
                    //  runonPower(glyphLift, liftPower);
                    break;
                case 0:
                    holdslide_released = 1;
                    holdslide_on = 1;
                    //  holdPosition(glyphLift);
                    break;
            }
        }

        if (holdslide_on == 1) {    //  ******reset the control loop to get rid of the integral error
            holdrelief++;
            if (holdrelief < 10) {
                extendArm.setPower(slideMotorMaxPower);
            } else if (holdrelief > 10) {
                extendArm.setTargetPosition(extendArm.getTargetPosition());
                holdrelief = 0;
            }
        }

        if (!myOpMode.gamepad2.a && holdslide_pressed == 1) {
            holdslide_pressed = 0;
        }

        // **************************END OF SLIDE FUNCTION

        //******************OPERATE SLIDE ARM BETWEEN 0 and 210 degrees with 0 being start position
        if (rotateArmL.getCurrentPosition() > mincountforslide || rotateArmR.getCurrentPosition() > mincountforslide) {

            if (myOpMode.gamepad2.right_stick_y > 0.3 || myOpMode.gamepad2.right_stick_y < -0.3) {
                armtargetposition += 10 * Math.signum(myOpMode.gamepad2.right_stick_y);
                if (armtargetposition < maxarmtargetposition && armtargetposition > minarmtargetposition) {
         //           armmotors_runtoposition();
                    rotateArmL.setTargetPosition(armtargetposition);
                    rotateArmR.setTargetPosition(armtargetposition);
                    rotateArmL.setPower(leftArmMotorMaxPower);
                    rotateArmR.setPower(rightArmMotorMaxPower);
                } else {

                    rotateArmL.setPower(0.0);
                    rotateArmR.setPower(0.0);
        //            armmotors_runwithoutencoder();
                }

            }


            if (myOpMode.gamepad2.b && holdarm_pressed == 0) {    //press a to hold arm position
                holdarm_pressed = 1;
                switch (holdarm_released) {
                    case 1:
                        holdarm_released = 0;
                        holdarm_on = 0;
                        //  runonPower(glyphLift, liftPower);
                        break;
                    case 0:
                        holdarm_released = 1;
                        holdarm_on = 1;
                        //  holdPosition(glyphLift);
                        break;
                }
            }


            if (!myOpMode.gamepad2.b && holdarm_pressed == 1) {
                holdarm_pressed = 0;
            }
        }
		
		
		
		
		
		//***** Code for sweeper **************/
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
			latchBrake.setPosition(latchBrakeOpen);
            tim1.reset();
            while (tim1.seconds() < 0.25) {
            }
			
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
			latchBrake.setPosition(latchBrakeOpen);
            tim1.reset();
            while (tim1.time() < 0.25) {
            }
			
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
				latchBrake.setPosition(latchBrakeClose);
			}
			latchPower = 0.0;
			latchBrakeState = 0;
		}
		
		
	}
		
	public void autoLift(boolean extend){
		eventDone = false;
		// @parameter up ..... pass in true to extend lift, false to retract lift
		if(extend){
			while(myOpMode.opModeIsActive() && !liftlimit.getState()){
				liftMotor.setPower(1.0);
			}
			liftMotor.setPower(0.0);
			eventDone = true;
		}else if(!extend){
			while(myOpMode.opModeIsActive() && !liftlimit.getState()){
				liftMotor.setPower(-1.0);
			}
			liftMotor.setPower(0.0);
			eventDone = true;
		}else {
			liftMotor.setPower(0.0);
			
		}
				
	}
	
	public void toggleAutoScore(){
	    //**********************AUTO SCORE MINERAL

        if(myOpMode.gamepad1.x && scoremineralauto_pressed==0){
            scoremineralauto_pressed=1;
            autoscore2();

        }

        if(!myOpMode.gamepad1.x && scoremineralauto_pressed==1) {
            scoremineralauto_pressed=0;
        }
	}
	
	public void autoLatch(boolean open){
		eventDone = false;
		tim1.reset();
		// @parameter open .....pass in true to open latch, false to close latch
		if(myOpMode.opModeIsActive() && open && !latchOpen){
			latchOpen = true;
			latchServoR.setPosition(OPEN_LATCH);
			latchServoL.setPosition(OPEN_LATCH);			
		}else if (myOpMode.opModeIsActive() && !open && latchOpen){
			latchOpen = false;
			latchServoR.setPosition(CLOSE_LATCH);
			latchServoL.setPosition(CLOSE_LATCH);
			while (myOpMode.opModeIsActive() && tim1.time() < 0.75){}
			eventDone = true; 
		}
	}
	
	public void lockLift(boolean lock){
		
		// parameter lock .... pass in true to lock the lift, false to unlock
		if(lock){
			latchBrake.setPosition(latchBrakeClose);
		}else{
			latchBrake.setPosition(latchBrakeOpen);
		}
		
	}
	
	public void controlLatch(){
		
		if(myOpMode.opModeIsActive() && myOpMode.gamepad2.x && !latchOpen){
			//open latch
			latchOpen = true;
			latchServoR.setPosition(OPEN_LATCH);
			latchServoL.setPosition(OPEN_LATCH);
			
		}else if(myOpMode.opModeIsActive() && myOpMode.gamepad2.x && latchOpen){
			//close latch
			latchOpen = false;
			latchServoR.setPosition(CLOSE_LATCH);
			latchServoL.setPosition(CLOSE_LATCH);

			
		}
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
        tim1.reset();
        while (myOpMode.opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy()) || tim1.time() < 2){
            myOpMode.telemetry.addData("Positioning", las1.getDistance(DistanceUnit.INCH));
            myOpMode.telemetry.update();
        }
    }	
	
	private void autoscore2() {

        double distance = las1.getDistance(DistanceUnit.MM)/25.4;
        double armextension=slideextension();
        int armtarget = (int) (140/360*leftarmpulsesperrev);
        if(distance<=scoredistance_fullextension){
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
            runToPositionMotors();
            leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition()+distance*cpi));
            leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition()+distance*cpi));
            leftDrive.setPower(0.4);
            rightDrive.setPower(0.4);
            distance=scoredistance_fullextension;
            armextension=fullextension;
        }
            armangle=Math.atan(19/Math.abs(distance));
            armextension=Math.sqrt(heighttoscore*heighttoscore+ distance*distance);
            rotateArmL.setTargetPosition(armtarget);
            rotateArmR.setTargetPosition(armtarget);

        extendArm.setTargetPosition((int)((armextension+lenthofintakesystem-13)*slideliftpulsesperinch));

		tim1.reset();
		
        while(Math.abs(rotateArmL.getCurrentPosition()-armtarget)>50||(tim1.time()<3)){}


        armtarget=(int) ((180+armangle)*leftarmpulsesperrev);
        rotateArmR.setTargetPosition(armtarget);

        while(Math.abs(rotateArmL.getCurrentPosition()-armtarget)>50||(tim1.time()<3)){}
        runWithoutEncoders();



    }

    public void manualDrive() {

        runWithoutEncoders();
        floatDriveMotors();
        /** drivers gamepad **/

        // In this mode the Left stick moves the robot fwd & back.
        // The Right stick rotates CCW and CW.

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        double max;
        double error;
        double steer;
        double leftPwr;
        double rightPwr;
        boolean firstRun = true;
		
		// use right trigger to turn on phoen flashlight for vuforia if needed
        /*
		if (myOpMode.gamepad1.right_trigger > 0.6) {
			vuf.flameOn(true);
		}else{
			vuf.flameOn(false);
		}
		*/

/*
        if (myOpMode.gamepad1.right_stick_x == 0.0) {
            //reset lastTime
            resetPID();
        }
		
		if(myOpMode.gamepad1.y && switchdrive_pressed==0 && switchdrive_toggle==0){
			switchdrive_pressed=1;
			switchdrive_toggle=1;
			switchdrive=-1; // front is robot lift
		}else if(myOpMode.gamepad1.y && switchdrive_pressed==0 && switchdrive_toggle==1){
			switchdrive_pressed=1;
			switchdrive_toggle=0;
			switchdrive=1; // mineral arm is front
		}

		if(!myOpMode.gamepad1.y && switchdrive_pressed==1) {
			switchdrive_pressed=0;
		}

*/


        //drive with stick or pad
/*
        if (myOpMode.gamepad1.x && xdelivermineral_pressed == 0) {
            switch (xdelivermineral_pressed) {
                case 0:
                    xdelivermineral_pressed = 1;
                    switch (dpad_mode) {
                        case 1:   // means turn off autoscore
                            dpad_mode = 0;
                            setMotPower(0,0);
                            runWithoutEncoders();
                            //floatmotors();
                            break;

                        case 0:   // means turn on autoscore
                            dpad_mode = 1;
                            //settargettocurrent();
                            runToPositionMotors();
                            //setmotorpower(0.6);
                            brakeDriveMotors();
                            break;
                    }
                    break;
                case 1:
                    break;
            }

        }

        if (!myOpMode.gamepad1.x && xdelivermineral_pressed == 1) xdelivermineral_pressed = 0;
*/

        //Get joystick input

        drive = -myOpMode.gamepad1.left_stick_y * speedCalFactor * switchdrive; // reduce sensitivity by speedCalFactor
		
		rawTurn = myOpMode.gamepad1.right_stick_x;
		turn = rawTurn * speedCalFactor; // reduce senitivity		
        turn = Range.clip(turn,-0.3,0.3); // clip it's max to 0.3


  /*
		if (dpad_mode == 1) {
            myOpMode.telemetry.addData("runnning dpad mode ", dpad_mode);
            if (myOpMode.gamepad1.dpad_up) {
  //              dpad_move_up();
            }
            if (myOpMode.gamepad1.dpad_down) {
  //              dpad_move_down();
            }
     //       if (Math.abs(xStick) > 0.3) {
     //           dpad_rotate();
     //       }

        }else{

*/

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

//		}
		
		
		
		
		


        //** This is intended to correct heading while driving straight, i.e. no right stick input


/*
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

 */


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

        //double currentAngle = imu.getAngularOrientation().firstAngle;
        double currentAngle = getAngle();

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

/*            // Display drive status for the driver.
            myOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            //telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            myOpMode.telemetry.addData("Actual",  "%7d:%7d",      leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            myOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftPwr, rightPwr);
            myOpMode.telemetry.update();
 */
        }
		

        setMotPower(0,0);
        previousError = 0.0;
        motorsOn = false;
		eventDone = true;
        runWithoutEncoders();


    }
	
	public void imuDrive(double motPwr, double inches){

		double  steer;
		double  leftSpeed;
        double  rightSpeed;
		double gyroTarget = 0.0; // We want heading to be straight, 0 degrees.
		runToPositionMotors();
		movepulses = ((int) (inches * cpi));
        int newLeftTarget = leftDrive.getCurrentPosition() + movepulses;
        int newRightTarget = rightDrive.getCurrentPosition() + movepulses;
		
		leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
		
		//Start moving
		motPwr = Range.clip(motPwr, -0.9, 0.9); // to allow some +/- at upper end of power
        setMotPower(motPwr, motPwr);
        motorsOn = true;
		eventDone = false;
 

		while (myOpMode.opModeIsActive() && leftDrive.isBusy() && rightDrive.isBusy()){
			double delta = (gyroTarget - getRelativeHeading() + 360.0) % 360.0; //the difference between target and actual mod 360
			if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
			if (Math.abs(delta) > 3) { //checks if delta is out of range
				double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
				if (Math.abs(gyroMod) > 1.0) gyroMod = 0.3 * (Math.signum(gyroMod)); //set gyromod to 1 or -1 if the error is more than 45 degrees
				steer = minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod;
				leftSpeed = motPwr - steer;
				rightSpeed = motPwr + steer;
				setMotPower(leftSpeed, rightSpeed);
			}
		}

		setMotPower(0,0);
        motorsOn = false;
		eventDone = true;
        runWithoutEncoders();
		
	}
	
	public void imuTurn( double minSpd, double gyroTarget){
		resetGlobalAngle();
		eventDone = false;
        runWithoutEncoders();
        brakeDriveMotors();
        myOpMode.sleep(100);
        //boolean drive = gyroCorrect(gyroTarget, minSpd) < 10;
		while (myOpMode.opModeIsActive() && gyroCorrect(gyroTarget, minSpd) < 1){
			myOpMode.telemetry.update();
		}

        resetGlobalAngle();
		eventDone = true;
	}

	
	 /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param //gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param //gyroActual The current heading in degrees, between 0 and 360
     * @param minSpd The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param //addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double minSpd) {
        double delta = (gyroTarget - getRelativeHeading() + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) gyroMod = 0.5 * (Math.signum(gyroMod)); //set gyromod to 1 or -1 if the error is more than 45 degrees

            turn(minSpd * Math.signum(gyroMod) + addSpeed * gyroMod);

            /*
            myOpMode.telemetry.addData("Delta", delta);
            myOpMode.telemetry.addData("HeadingReset",headingResetValue);
            myOpMode.telemetry.addData("RelativeHeading", getRelativeHeading());
            myOpMode.telemetry.addData("power", minSpd * Math.signum(gyroMod) + addSpeed * gyroMod);
            */

        }
        else {

            correctCount++;
            turn(0.0);
        }

        return correctCount;
    }
    public void turn(double sPower) {

        sPower = Range.clip(sPower, -0.4, 0.4);
        setMotPower(-sPower, sPower);
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
//        double newAngle = angle;
//        resetAngle();

        //Set motors to run without encoders
        runWithoutEncoders();

		//reset the PID
		resetPID();

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && !onHeading(speed, newAngle)) {
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
     * @param //PCoeff    Proportional Gain coefficient
     * @return onTarget will be true or false
     */

    // **** onHeading modified to use PID controller
    // Don't use PCoeff, instead use kP, kI, kD

    boolean onHeading(double speed, double targetAngle) {
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
            // Send corrected speeds to motors.
            setMotPower(leftSpeed,rightSpeed);

            onTarget = true;
			motorsOn = false;
			eventDone = true;
            myOpMode.telemetry.addData("Target", "%5.2f", targetAngle);
            myOpMode.telemetry.addData("Angle", "%5.2f", getHeading());
            myOpMode.telemetry.addData("RelAngle", "%5.2f", getRelativeHeading());
            myOpMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            myOpMode.telemetry.addData("Power.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            myOpMode.telemetry.addData("GlobalAngle.", "%5.2f", getAngle());
            myOpMode.telemetry.update();

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



            if(error < 10){
                rightSpeed = Range.clip(rightSpeed, -0.3, 0.3);
                leftSpeed = Range.clip(leftSpeed, -0.3, 0.3);
            }

            if(rightSpeed < 0){
                rightSpeed = Range.clip(rightSpeed, -0.5, -0.25);
            }else if(rightSpeed > 0 ){
                rightSpeed = Range.clip(rightSpeed, 0.25, 0.5);
            }
            if(leftSpeed < 0){
                leftSpeed = Range.clip(leftSpeed, -0.5, -0.25);
            }else if(leftSpeed > 0 ){
                leftSpeed = Range.clip(leftSpeed, 0.25, 0.5);
            }
            // Display it for the driver.
            myOpMode.telemetry.addData("Target", "%5.2f", targetAngle);
            myOpMode.telemetry.addData("Angle", "%5.2f", getHeading());
            myOpMode.telemetry.addData("RelAngle", "%5.2f", getRelativeHeading());
            myOpMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            myOpMode.telemetry.addData("Power.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            myOpMode.telemetry.addData("GlobalAngle.", "%5.2f", getAngle());


            // Send corrected speeds to motors.
            setMotPower(leftSpeed,rightSpeed);

        }

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
        //robotError = targetAngle - getAngle();


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

    //private double getRelativeHeading(){
    //    return getHeading() - headingResetValue;
    //}

	private double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    private double getRelativeHeading() {
        return getAbsoluteHeading() - headingResetValue;
    }
	
    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void resetGlobalAngle(){
        headingResetValue = getAbsoluteHeading();
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getHeading() {

        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //return imu.getAngularOrientation().firstAngle;

        Orientation headingAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return headingAngles.firstAngle;


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
	
	private void holdPosition(DcMotor mot) {
        int position = mot.getCurrentPosition();
        mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mot.setTargetPosition(position);
        mot.setPower(0.5);

    }
	
	private void runonPower(DcMotor mot, double motP) {
        mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot.setPower(motP);
    }
	
	public void setarmMotoruntoposition() {

        rotateArmL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
        rotateArmL.setTargetPosition(rotateArmL.getCurrentPosition());
        rotateArmR.setTargetPosition(rotateArmR.getCurrentPosition());
		
        rotateArmL.setPower(leftArmMotorMaxPower);
        rotateArmR.setPower(rightArmMotorMaxPower);
    }


	
	public void stopReset() {
/*
        rotateArmL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateArmR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/

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

    public void floatDriveMotors(){
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // set motors to brake at 0 power
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
	
    private double slideextension() {
        return(extendArm.getCurrentPosition() / slideliftpulsesperinch * 2 + 13);
    }
    private void setlinearslide(double extension){
        extension=(extension-13)/slideliftpulsesperinch;
        extendArm.setTargetPosition((int) (extension));
    }
    private void linearservopos(Servo serv, double pos, double min, double max) {
        pos = (max - min)/2.0 * pos + (max + min)/2.0;
        serv.setPosition(pos);
    }	

}

