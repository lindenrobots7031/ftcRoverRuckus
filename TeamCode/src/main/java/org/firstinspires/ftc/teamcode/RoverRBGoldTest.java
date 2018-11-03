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


@TeleOp(name = "RB Rover Vision", group = "Tele")
@Disabled
public class RoverRBGoldTest extends LinearOpMode {


    MyBot robot = new MyBot(); // use the drop wheel hardware helper class
	
    //VuForiaClass vuf = new VuForiaClass(); // use the vuforia vision helper class
	//    VuForiaWebcamClass vuf = new VuForiaWebcamClass(); // use vuforia with webcam

	GoldAlign goldDetect = new GoldAlign();
	RedBlue rbDetect = new RedBlue();


	
    private ElapsedTime myRunTime  = new ElapsedTime();
    boolean calib = true;
    String myLatch;
	double goldPos = 0.0;
	boolean found = false;

    
    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        //robot.initDrive(this);

		//robot.initIMU();
		
		//robot.latchOpen = false;
		//robot.brakeDriveMotors(); // set the zero power to brake for drive motors, helps to control during gyrodrive
		

		
        // While wait for the game to start (before driver presses PLAY)
        while (!isStarted()) {	

			//if(!robot.imu.isGyroCalibrated()){
			//	telemetry.addData("Status:", "IMU not calibrated");
			//}else if(robot.imu.isGyroCalibrated()){
			//	telemetry.addData("Status:", "IMU calibrated");

			//}


			//goldPos = goldDetect.getAlign();
			//found = rbDetect.getFound();

			//telemetry.addData("RB isFound" , found);
			//telemetry.addData("Gold Xpos", goldPos);
			//telemetry.update();
		}
//		goldDetect.initGoldAlign(this);
		rbDetect.initRB(this);
        // wait for the start button to be pressed.
        waitForStart();
        myRunTime.reset();

 

		while (opModeIsActive()) {

//			goldPos = goldDetect.getAlign();
			found = rbDetect.getFound();

        	telemetry.addData("RB isFound" , found);
			telemetry.addData("Gold Xpos", goldPos);

			
			if(myRunTime.time() > 45) {
//				goldDetect.stopAlign();
				rbDetect.stopAlign();
			}
			telemetry.update();


		} // end while opmodeisactive
		



    }
	

}