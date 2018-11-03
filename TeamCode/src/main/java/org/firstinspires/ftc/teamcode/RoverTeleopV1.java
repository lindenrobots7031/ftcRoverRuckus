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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Rover Teleop v1.0", group="Arcade")
//@Disabled
public class RoverTeleopV1 extends LinearOpMode {

    /* Declare OpMode members. */
    MyDropWheelBot robot = new MyDropWheelBot(); // use the drop wheel hardware helper class
    VuForiaClass vuf = new VuForiaClass(); // use the vuforia vision helper class
	
	private int step = 1; // init the first state for state machine
						
	public double distCL = 36;  //TODO need to measure distance to drive
	public double distToLatch = 18; //TODO need to measure
	public boolean padOne = false;
	public boolean padTwo = false;
	public int loopCount;
	public double turnAngle;
	public double markDist = 0;
	public double markLateral = 0;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        vuf.initVuforia(this, robot);
		robot.initDrive(this);
		robot.initIMU();
        // Activate Vuforia (this takes a few seconds)
        vuf.activateTracking();
		
		robot.latchOpen = false;

		while (!isStarted()) {
			if(!robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU not calibrated");
			}else if(robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU calibrated");

			}


			// Display any Nav Targets while we wait for the match to start
			vuf.targetsAreVisible();
			vuf.addNavTelemetry();
		telemetry.update();
		}

		//
		// robot.setarmMotoruntoposition();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

		
        // run loop until the end of the match (driver presses STOP)

			/**
			/*to activate auto latch routine
			/* drive bot to vuforia nav target and face it
			/* press and hold A button on driver's gamepad
			/* the navigation variables
			/* vuf.robotX = 0;
			/* vuf.robotY = 0;
			/* vuf.targetRange = 0;
			/* vuf.targetBearing = 0;
			/* vuf.robotBearing = 0;
			/* vuf.relativeBearing = 0;
			**/

        telemetry.addData("Status", "Started");    
        telemetry.update();
			
        while (opModeIsActive()) {

			//vuf.targetsAreVisible();
			//vuf.addNavTelemetry();

			
			
			// Do these things every loop cycle
			// check if any input on the joysticks -- may need to add additional checks depending on what functions the buttons do
			
			if(gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0){
				padOne = true; // true if there is joystick input
			}else if(gamepad1.right_stick_x == 0 || gamepad1.right_stick_y == 0 || gamepad1.left_stick_x == 0 || gamepad1.left_stick_y == 0){
				padOne = false;
			}
			//Uncomment when 2nd joystick plugged in
			//if(gamepad2.right_stick_x != 0 || gamepad2.right_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0){
			//	padTwo = true;
			//}else if(gamepad2.right_stick_x == 0 || gamepad2.right_stick_y == 0 || gamepad2.left_stick_x == 0 || gamepad2.left_stick_y == 0){
			// padTwo = false;
			//}
			
			// Temporary simulate 2nd joystick with boolean button on joystick 1
			if(gamepad1.b){
				padTwo = true;
			}


			telemetry.addData("Current State: ", step);
			telemetry.addData("mark dist: ", markDist);
			telemetry.update();

			switch (step) {
				case 1:
					
					if (vuf.targetsAreVisible() && gamepad1.a && !padOne) {	
						turnAngle = -vuf.robotBearing;
						markDist = vuf.robotX - 10;
						markLateral = vuf.robotY;
					
						// TODO retract mineral arm extenstion
						// TODO lower (or raise??) mineral arm to stowage position for auton moving
						loopCount = 0;
						step = 2;
					} else {
					robot.manualDrive(); // use gamecontroller 1 to drive - not in auto latch routine
					// *** Uncomment when mineral arm is on robot  
					//robot.controlArm();
					//robot.controlLift()
					//robot.controlLatch();
					//robot.toggleAutoScore();
						//telemetry.addData("padOne: ", padOne);
						//telemetry.addData("Current State: ", step);

					}

					break;

				case 2:	
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){				
						robot.imuTurn(0.22, turnAngle); // turn to square up to vuforia nav taget
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 3;
						break;
					}
					
					break;
				case 3:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){				
						robot.gyroDrive(0.5, markDist); // TODO  drive to 6 inches from wall - will need to determine is this is correct distance
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 4;
						break;
					}
					break;
				case 4:			
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){				
					robot.imuTurn(0.22, 90); // turn to the left 90 deg
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 5;
						break;
					}
					break;					
				case 5:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0 && gamepad1.a){			
						robot.gyroDrive(0.5, markLateral); // drive to nav target centerline
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 6;
						break;
					}
					break;					
				case 6:

					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){			
						robot.imuTurn(0.22, -45); // turn to the right 45 deg
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 7;
					}
					break;					
				case 7:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){		
						robot.gyroDrive(0.6, -distCL); //drive reverse to the latch centerline
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 8;
					}
					break;
				case 8:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){	
						robot.imuTurn(0.22, -90); // turn to right 90 deg to square up to latch face of lander, facing outward
						loopCount = 1;					
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 9;
					}
					break;
				case 9:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){			
						robot.gyroDrive(0.5, -distToLatch); //drive reverse to back up to latch, maybe use the las1 - REV laser range sensor
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 10;
						break;
					}
					break;
				case 10:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){													
			//			robot.autoLift(true); // true to extend lift up
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 11;
						break;
					}
					break;
				case 11:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){								
			//			robot.autoLatch(false); // false to close latch
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 12;
						break;
					}
					break;					
				case 12:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){				
										
			//			robot.autoLift(false); // false to retract
						loopCount = 1;
						break;
					}else if (loopCount > 0 && robot.eventDone){
						loopCount = 0;
						step = 12;
						break;
					}
					break;					
				case 13:
					if (!gamepad1.a || padOne || padTwo) {
						step = 1;
						break;
					}else if (loopCount == 0){						
			//			robot.lockLift(true); // true to lock the lift
						loopCount = 1;
						break;					
					}
					break;					

				default:
					robot.manualDrive();
					//robot.controlArm();
					//robot.controlLift()
					break;

				} // end switch



            // Pause for 40 mS each cycle = update 25 times a second.  Do we need this???
            sleep(40);
        } // end whileopmodeactive loop
		
		
    }
}
