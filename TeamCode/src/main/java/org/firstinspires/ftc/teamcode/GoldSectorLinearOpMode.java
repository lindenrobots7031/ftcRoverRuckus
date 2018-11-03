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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.disnodeteam.dogecv.detectors.roverrukus.BattleBlazerGoldCubeSector;


@TeleOp(name="Auton GoldDoge", group="Auton")

public class GoldSectorLinearOpMode extends LinearOpMode
{
    MyDropWheelBot robot = new MyDropWheelBot(); // use the drop wheel hardware helper class
//    VuForiaClass vuf = new VuForiaClass(); // use the vuforia vision helper class

    WebcamName webcamName;

    private BattleBlazerGoldCubeSector detector;

    public ElapsedTime myTimer = new ElapsedTime();
    public double elapseTime;
    public boolean vufStart;


    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry.addData("Status", "BBDogeCV 2018.0 - Gold Align Example");

        detector = new BattleBlazerGoldCubeSector();


        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 210; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.7;

        detector.enable();

        // wait for the start button to be pressed.
        waitForStart();

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            elapseTime = myTimer.seconds();
            if (elapseTime <= 30) {
                telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
                telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
                telemetry.addData("Sector", detector.whichSector().toString());
                telemetry.update();

                if (detector.whichSector().toString() == "LEFT"){
                    //TODO left mineral path
                } else if (detector.whichSector().toString() == "RIGHT"){
                    //TODO right mineral path
                } else {
                    // either it's center or cant see it.  If can't see it just do center path
                    //TODO center mineral path
                }

            } else {

 /*               if(!vufStart) {
                    vufStart = true;
                    detector.disable();
                    vuf.initVuforia(this, robot);
                    // Activate Vuforia (this takes a few seconds)
                    vuf.activateTracking();
                    sleep(3000);
                }
                // Display any Nav Targets while we wait for the match to start
                vuf.targetsAreVisible();
                vuf.addNavTelemetry();
                telemetry.update();
                */
            }


            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }

}
