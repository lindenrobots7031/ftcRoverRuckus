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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//@Autonomous(name = "Rover Vision Tele", group = "Auton")
@TeleOp(name="Rover Vision Tele", group="Arcade")
@Disabled                            // Comment this out to add to the opmode list
public class RoverVisionTele extends LinearOpMode {


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
        vuf.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        vuf.activateTracking();

		// initialize the IMU
		robot.initIMU();
		
		robot.latchOpen = false;
		robot.brakeDriveMotors();
		
        // While wait for the game to start (driver presses PLAY)
        while (!isStarted()) {	

			if(!robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU not calibrated");
			}else if(robot.imu.isGyroCalibrated()){
				telemetry.addData("Status:", "IMU calibrated");

			}

			if (!vuf.vuforiaClosed) {
				// Display any Nav Targets while we wait for the match to start
				//vuf.targetsAreVisible();
				//vuf.addNavTelemetry();
			}
			telemetry.update();

        }

        // wait for the start button to be pressed.
        waitForStart();
        myRunTime.reset();
        vuf.flameOn(true);

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

            /** to determine what quadrant, i.e. what color (gold or silver) latch robot is starting on
             *  myLatch = vuf.getLatchColor();
             *  This will return either "gold" or "silver". Use this to run specific auton path
            **/
            //***********************************************/

		/****************************************************  			
        //Turn camera to see vuforia nav target
        robot.setServo(robot.camServo,1.0);
        sleep(300);

        //determine which latch color we are on based on nav target
        myLatch = vuf.getLatchColor();
		
		//turn off vuforia so we can use dogeCV
		if (!vuf.vuforiaClosed){
			vuf.closeVuf();
			telemetry.addData("Vuf: ", "closed");
		}
		// initialize dogecv
		if (goldDetect.goldDetectClosed){
			goldDetect.initGoldAlign(this);
		}

        //turn camera back around to see mineral samples
        robot.setServo(robot.camServo, 0.0);
        sleep(1000);
	
		//save the gold position: LEFT, RIGHT, CENTER
		goldPos = goldDetect.getSector();
		
        if(myLatch == "gold") {
            if (goldPos == "LEFT") {
                robot.gyroTurn(0.2, 30);
                robot.gyroDrive(0.8, 24);
				robot.gyroTurn(0.2,-30);
                robot.gyroDrive(0.8, 24);
                robot.gyroDrive(0.8, -6);
                robot.gyroTurn(0.2,-135);
				
                robot.deployMarker();
				
				// head to crater
                robot.gyroTurn(0.3, -45);
				robot.gyroDrive(1.0, 52);
				

            } else if (goldPos == "CENTER") {
                robot.gyroDrive(1.0, 55);
                robot.gyroDrive(1.0,-8);
                robot.gyroTurn(0.3, -45);
				
                robot.deployMarker();
				
				//head to crater
                robot.gyroDrive(1.0, -20);
                robot.gyroTurn(0.2, 135);
                robot.gyroDrive(1.0, 52);

            } else if (goldPos == "RIGHT") {
                robot.gyroTurn(0.2, -30);
                robot.gyroDrive(0.8, 24);
                robot.gyroTurn(0.2,30);
                robot.gyroDrive(0.8, 12);
				robot.gyroTurn(0.2,45);
				robot.gyroDrive(0.8, 12);
                robot.gyroDrive(0.8, -6);
                robot.gyroTurn(0.2, -45);
                robot.deployMarker();

				// navigate path to crater
				robot.gyroTurn(0.2, -45);
				robot.gyroDrive(1.0, -20);
				robot.gyroTurn(0.2, 45);
				robot.gyroDrive(1.0, -52);
				
            } else {
                // no gold found so just move middle mineral
                robot.gyroDrive(1.0, 55);
                robot.gyroDrive(1.0,-8);
                robot.gyroTurn(0.3, -45);
                robot.deployMarker();
                //now drive to crater
                robot.gyroDrive(1.0, -20);
                robot.gyroTurn(0.2, 135);
                robot.gyroDrive(1.0, 52);

            }
		// end if statement for gold routines
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
		robot.gyroTurn(0.3, 30);
		sleep(4000);

		robot.gyroTurn(0.3,-30);
		sleep(4000);

		robot.gyroTurn(0.3,-135);

		//robot.deployMarker();
		sleep(4000);
		// head to crater
		robot.gyroTurn(0.3, -45);
		*/



			

       while (opModeIsActive()) {



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


			if(myRunTime.time() > 8 ) {
				if (!vuf.vuforiaClosed){
					vuf.closeVuf();
					telemetry.addData("Vuf: ", "closed");
				}

			}
			if(myRunTime.time() > 8 && myRunTime.time() <9.5) {
				if (goldDetect.goldDetectClosed){
			//		telemetry.addData("Gold Align: ", goldDetect.goldDetectClosed);
					goldDetect.initGoldAlign(this);
				}
			}
			if(myRunTime.time() > 8.2 ) {
				if (!goldDetect.goldDetectClosed){
				//	goldPos = goldDetect.getAlign();
				//	width = goldDetect.getWide();
				//	area = goldDetect.getArea();

				//	telemetry.addData("xpos", goldPos);
				//	telemetry.addData("width", width);
				//	telemetry.addData("area", area);

			//		telemetry.addData("xpos", goldDetect.getAlign());
				}
			}
			if(myRunTime.time() > 60) {
				goldDetect.stopAlign();
			}
			telemetry.update();


        } // end while opmodeisactive
		



    }
	

}