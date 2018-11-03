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

//change this package
package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.BattleBlazerGoldCubeDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;


@Autonomous(name = "Rover Auton", group = "Auton")
//@TeleOp(name="Rover Auton", group="Arcade")
//@Disabled                            // Comment this out to add to the opmode list
public class RoverAutonV1 extends LinearOpMode {


    MyDropWheelBot robot = new MyDropWheelBot(); // use the drop wheel hardware helper class
	
    VuForiaClass vuf = new VuForiaClass(); // use the vuforia vision helper class
	//    VuForiaWebcamClass vuf = new VuForiaWebcamClass(); // use vuforia with webcam

	GoldAlign goldDetect = new GoldAlign();


	
    private ElapsedTime myRunTime  = new ElapsedTime();
    boolean calib = true;
    String myLatch;
	double goldPos;
	double width;
	double area;

    
    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
//        vuf.initVuforia(this, robot);

		goldDetect.initGoldAlign(this);

        // Activate Vuforia (this takes a few seconds)
//		vuf.activateTracking();

		// initialize the IMU
		robot.initIMU();
		
		//robot.latchOpen = false;
		robot.brakeDriveMotors();


        // While wait for the game to start (driver presses PLAY)
        while (!isStarted()) {	

			if(!robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU not calibrated");
			}else if(robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU calibrated");

			}


//			if (!vuf.vuforiaClosed) {
				// Display any Nav Targets while we wait for the match to start
//				vuf.targetsAreVisible();
//				vuf.addNavTelemetry();
//			}
			telemetry.update();

        }

        // wait for the start button to be pressed.
        waitForStart();
        myRunTime.reset();

		goldDetect.flash(true);
		
		// Unlock latchbrake,  false to unlock
		robot.lockLift(false);
		sleep(500);
		
		// extend lift, true to extend
		robot.autoLift(true);
		sleep(4000); //  how long to extend lift???
		
		// open latch, true to open latch
		robot.autoLatch(true);
		sleep(500);
		
		// retract lift
		robot.autoLift(false); 
		
		/**********   Assume on gold latch - facing depot ********/
		
		// check if gold in center
		if(goldDetect.getAlign()){
			
			goldDetect.flash(false);
			
			//disable the dogecv detector
			goldDetect.stopAlign();
			
			//***** run the center gold routine  *******/
			
			// drive forward to push block into depot
			robot.gyroDrive(0.8, 50);
			sleep(2000);
			
			// deploy marker
			robot.deployMarker();
			sleep(600);
			
			// reverse to clear the block, so it will stay in depot and not be pushed out when turning
			robot.gyroDrive(0.8, -15);
			sleep(100);
			
			// turn left 90 degrees
			robot.imuTurn(0.22, 90.0); // should be split between 2 45 degrees? is 90 degree turn accurate enough
			sleep(800);
			
			// forward towards walls, stop with just enough room to clear wall when turning left again
			robot.gyroDrive(0.8, 12);
			sleep(800);
			
			// turn left 45 degrees
			robot.imuTurn(0.22, 45.0);
			sleep(800);
			
			// forward to crater
			robot.gyroDrive(0.8, 50);
		
		
		}else{
			// turn left 30?? deg
			robot.imuTurn(0.22, 30.0); // is this left?? verify which direction negative/positive
			sleep(800);
			
			//check if gold is aligned (i.e. gold in left position)
			if(goldDetect.getAlign()){
				
				//disable the dogecv detector
				goldDetect.stopAlign();
				
				//***** run the left gold routine *******/
				
				//forward to push gold block off 
				robot.gyroDrive(0.8, 24);
				sleep(2000);
				
				// turn right 30 degrees
				robot.imuTurn(0.22, -30.0);
				sleep(600);
				
				//forward into depot
				robot.gyroDrive(0.8, 24);
				sleep(1500);
				
				//deploy marker
				robot.deployMarker();
				sleep(600);
				
				// turn right xx degrees
				robot.imuTurn(0.22, -30.0);
				sleep(600);

				//reverse towards walls, get close to wall but clear it
				robot.gyroDrive(0.8, 24);
				sleep(1500);
				
				//turn left to square up parallel to wall
				robot.imuTurn(0.22, 30.0);
				sleep(600);
				
				//reverse to crater
				robot.gyroDrive(0.8, -50);
				
				
			
			}else{
				robot.imuTurn(0.22, -60.0);
				sleep(800);
				
				//disable the dogecv detector
				goldDetect.stopAlign();
				
				// no need to verify if gold there,  just assume since it was not detected at center or left.
				
				//***** run right gold routine *******/
				
				// forward to push mineral off
                robot.gyroDrive(0.8, 24);
				sleep(1500);
				
				// turn left towards depot
                robot.imuTurn(0.22,40);
				sleep(600);
				
				// forward into depot
                robot.gyroDrive(0.8, 24);
				sleep(1500);
				
				//deplow marker
				robot.deployMarker();
				sleep(600);
				
				// reverse to clear the block and marker
				robot.gyroDrive(0.8, -10);
				
				// turn left
				robot.imuTurn(0.22,10);
				sleep(600);
				
				//forward to wall
				robot.gyroDrive(0.8, 30);
				sleep(1500);
				
				//turn to be parallel with wall
				robot.imuTurn(0.22,30);
				sleep(600);
				
				//forward to crater
				robot.gyroDrive(0.8, 50);
						
				
			}
			
			
		}
		
		/********************************************************/
		
		// check if gold in center
		if(goldDetect.getAlign()){
			
			goldDetect.flash(false);
			
			//disable the dogecv detector
			goldDetect.stopAlign();
			
			//***** run the center gold routine  *******/
			
			// drive forward to push mineral
			robot.gyroDrive(0.8, 30);
			sleep(2000);
			
			robot.gyroDrive(0.6, -20);
			sleep(1500);
			
			//turn right 90 degrees - does this need to be split into 45 degrees for more accurate turn?
			robot.imuTurn(0.22, 90);
			sleep(400);	
				
			//forward towards wall
			robot.gyroDrive(0.8, 30);
			sleep(2000);
			
			// turn left to be parallel with wall
			robot.imuTurn(0.2,45);
			sleep(400);
			
			//drive to depot
			robot.gyroDrive(1.0, 52);
			sleep(2500);
			
			// deploy marker
			robot.deployMarker();
			sleep(600);			
			

			//reverse back to crater
			robot.gyroDrive(0.6,-60);
					
		
		}else{
			// turn left 30?? deg
			robot.imuTurn(0.22, 30.0); // is this left?? verify which direction negative/positive
			sleep(800);
			
			//check if gold is aligned (i.e. gold in left position)
			if(goldDetect.getAlign()){
				
				//disable the dogecv detector
				goldDetect.stopAlign();
				
				//***** run the left gold routine *******/
				
				// forward to push mineral off
                robot.gyroDrive(0.8, 24);
				sleep(1500);
				
				
				//turn left
				robot.imuTurn(0.22, 15);
				sleep(400);
				
				// forward
				robot.gyroDrive(0.8, 12);
				sleep(1500);
				
				//turn left
				robot.imuTurn(0.2, 45);
				sleep(400);
				
				//forward
				robot.gyroDrive(0.8, 12);
				sleep(1200);
				
				//left turn
				robot.imuTurn(0.2, 45);
				sleep(400);
				
				//forward to depot 
                robot.gyroDrive(0.8, 56);
				sleep(2500);
				
				//deploy marker			
				robot.deployMarker();
				sleep(400);
				
				// head to crater
				robot.gyroDrive(1.0, 60);
							
			}else{
				robot.imuTurn(0.22, -60.0);
				sleep(800);
				
				//disable the dogecv detector
				goldDetect.stopAlign();
				
				// no need to verify if gold there,  just assume since it was not detected at center or left.
				
				//***** run right gold routine *******/
				
				//forward to push off mineral
				robot.gyroDrive(0.8, 35);
				sleep(1500);
				
				//reverse so that we can turn without hitting other minerals
				robot.gyroDrive(0.8, -15);
				sleep(1000);
				
				//turn left - need to broken up??				
				robot.imuTurn(0.2, 120);
				sleep(800);
				
				//forward towards wall
				robot.gyroDrive(0.8, 48);
				sleep(1500);
				
				//turn to be parallel with wall
				robot.imuTurn(0.2, 45);
				
				//forward to depot
				robot.gyroDrive(0.8, 50);
				sleep(2500);
				
				//deploy marker
				robot.deployMarker();
				sleep(500);
				
				//head to crater in reverse
				robot.gyroDrive(0.6,-60);
				
		
			}
			
			
		}

		

            //**************  how to move robot  ************/
            /** to move robot fore-aft
              *  robot.gyroDrive(power, distance);
              *  example:
              *  robot.gyroDrive(0.6, -12);
              *  0.6 power, 12 inches backward, negative drives reverse, positive forward
            **/

            /** to turn robot
             *  robot.gyroTurn(power, angle);
             *  example:
             *  robot.gyroTurn(0.3, -90);
             *  0.3 power, 90 degrees CW, negative angle turns robot CW, positive turns robot CCW
            **/

            //***********************************************/

		/****************************************************  			



        }else{
            if(goldPos == "LEFT"){
                robot.gyroTurn(0.2, 30);
                robot.gyroDrive(0.8, 24);
				robot.gyroTurn(0.2, 15);
				robot.gyroDrive(0.8, 12);
				robot.gyroTurn(0.2, 45);
				robot.gyroDrive(0.8, 12);
				robot.gyroTurn(0.2, 45);
                robot.gyroDrive(0.8, 56);
				robot.gyroDrive(0.6, -6);
				robot.gyroTurn(0.2, -90);
				
				robot.deployMarker();
				
				// head to crater
				robot.gyroTurn(0.2, -90);
				robot.gyroDrive(1.0, 52);

            }else if(goldPos == "CENTER"){
				robot.gyroDrive(0.6, 24);
				robot.gyroDrive(0.6, -20);
				robot.gyroTurn(0.2, 90);
				robot.gyroDrive(0.8, 30);
				robot.gyroTurn(0.2,45);
				robot.gyroDrive(1.0, 52);
				robot.gyroTurn(0.2, -90);
				
				robot.deployMarker();
				
				//head to crater
				robot.gyroTurn(0.2,-90);
				robot.gyroDrive(0.6,60);

            }else if(goldPos == "RIGHT"){
				robot.gyroTurn(0.2, -30);
				robot.gyroDrive(0.8, 24);
				robot.gyroDrive(0.8 -15);
				robot.gyroTurn(0.2, 120);
				robot.gyroDrive(0.8, 48);
				robot.gyroTurn(0.2, 45);
				robot.gyroDrive(0.8, 50);
				robot.gyroTurn(0.2, -90);
				
				robot.deployMarker();
				
				//head to crater
				robot.gyroTurn(0.2,-90);
				robot.gyroDrive(0.6,60);				

            }else {
				// don't see gold, so just assume middle mineral
				robot.gyroDrive(0.6, 24);
				robot.gyroDrive(0.6, -20);
				robot.gyroTurn(0.2, 90);
				robot.gyroDrive(0.8, 30);
				robot.gyroTurn(0.2,45);
				robot.gyroDrive(1.0, 52);
				robot.gyroTurn(0.2, -90);
				
				robot.deployMarker();
				
				//head to crater
				robot.gyroTurn(0.2,-90);
				robot.gyroDrive(0.6,60);
            }
        } //end if statement for silver path routines
		
		// need to call this after done with the gold detector
        goldDetect.stopAlign();

*********************************************************/


        // need to call this after done with the gold detector
        //goldDetect.stopAlign();
		
		//testing gyro turns
/*
		robot.imuTurn(0.22, 30.0);
		sleep(3000);

//		robot.gyroDrive(0.3, 2);
//		sleep(3000);

		robot.imuTurn(0.22,30.0);
		sleep(3000);
//		robot.gyroDrive(0.3, 2);
//		sleep(3000);
		robot.imuTurn(0.22,30.0);
		sleep(3000);

		robot.imuTurn(0.22,-90.0);
		sleep(3000);

		robot.imuTurn(0.22,-45.0);

		//robot.deployMarker();
		sleep(3000);
		// head to crater
		robot.imuTurn(0.22, -45.0);
		*/

//		robot.imuDrive(0.4, 24);
//		sleep(3500);
		//robot.imuTurn(0.22,-90.0);
//		sleep(800);
		//robot.imuTurn(0.22,-90.0);







 //      while (opModeIsActive()) {
/*		if(robot.getAngle() > 90){
			robot.resetAngle();
		}
       	telemetry.addData("heading",robot.getAngle());
       	telemetry.update();

 */


            /*
            if (robot.imu.isGyroCalibrated() && calib) {
                BNO055IMU.CalibrationData calibrationData = robot.imu.readCalibrationData();
                String filename = "BNO055IMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                telemetry.log().add("saved to '%s'", filename);
                calib = false;
            }
            if (!calib) {
                telemetry.addLine("done");
            }

            telemetry.addData("Status", robot.imu.getSystemStatus().toShortString());
            telemetry.addData("Calib Status", robot.imu.getCalibrationStatus().toString());
            telemetry.addData("Gyro Calib?", robot.imu.isGyroCalibrated());
            telemetry.addData("heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            */
           

//			if (!vuf.vuforiaClosed) {
//				vuf.targetsAreVisible();
//				vuf.addNavTelemetry();
//				telemetry.update();
//			}

		/*

		   vuf.targetsAreVisible();
		   vuf.addNavTelemetry();


			if(myRunTime.time() > 20 ) {
				if (!vuf.vuforiaClosed){
					vuf.closeVuf();
					telemetry.addData("Vuf: ", "closed");
				}

			}
			if(myRunTime.time() > 20 && myRunTime.time() <20.5) {
				if (goldDetect.goldDetectClosed){
			//		telemetry.addData("Gold Align: ", goldDetect.goldDetectClosed);
					goldDetect.initGoldAlign(this);
				}
			}
			if(myRunTime.time() > 20.5 ) {
				if (!goldDetect.goldDetectClosed){
					goldPos = goldDetect.getAlign();
					width = goldDetect.getWide();
					area = goldDetect.getArea();

					telemetry.addData("xpos", goldPos);
					telemetry.addData("width", width);
					telemetry.addData("area", area);

			//		telemetry.addData("xpos", goldDetect.getAlign());
				}
			}
			if(myRunTime.time() > 29.5) {
				goldDetect.stopAlign();
			}
			telemetry.update();


			*/


//        } // end while opmodeisactive
		



    }
	

}