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


@Autonomous(name = "Gold Latch Auton", group = "Auton")
//@Disabled                            // Comment this out to add to the opmode list
public class GoldLatchRoverAuton extends LinearOpMode {


    MyDropWheelBot robot = new MyDropWheelBot(); // use the drop wheel hardware helper class
	
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

		// initialize the IMU
		robot.initIMU();
		
		//robot.latchOpen = false;
		robot.stopReset();
		robot.brakeDriveMotors();


        // While wait for the game to start (driver presses PLAY)
        while (!isStarted()) {	

			if(!robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU not calibrated");
			}else if(robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU calibrated");

			}

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

            //**************  how to move robot  ************/
            /** to move robot fore-aft
              *  robot.gyroDrive(power, distance);
              *  example:
              *  robot.gyroDrive(0.6, -12);
              *  0.6 power, 12 inches backward, negative drives reverse, positive forward
            **/

            /** to turn robot
             *  robot.imuTurn(minPower, angle);
             *  example:
             *  robot.gyroTurn(0.3, -90);
             *  0.3 minimum power, 90 degrees CW, negative angle turns robot CW, positive turns robot CCW
            **/

            //***********************************************/

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
				
				goldDetect.flash(false);
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
				
				goldDetect.flash(false);
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
		



    }
	

}