/* Copyright (c) 2018 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class VuForiaWebcamClass {

    // Constants
    private static final int     MAX_TARGETS    =   4;
    private static final double  ON_AXIS        =  10;      // Within 1.0 cm of target center-line
    private static final double  CLOSE_ENOUGH   =  20;      // Within 2.0 cm of final target standoff

    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private MyDropWheelBot     myRobot;        // Access to the Robot hardware


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfaV9a//////AAABmZNEZ0PRWEKpknjiQqLtf/IxD+7VgrEwGvS2i8GN7JprMaGS645uuq97ReMUKGjZx8OZBuddlkK7c5nCHtZoqs96twah1kKgE4sDOBzrRA7kv40vw04Vc0fTW68U26WbFzTXimNRcxCxqLb1qUVFRWaBsWNRbJsIUo+ir0Db0yQwgsIQbX8vIUb8GwQWgy+G7sTkZjJJ3Q9bpPFa3wNmrEivWld8khRa3QdffUjZyGjKJRRCbo4JAzUwA1iBBKRuWTJabwBlh8vMHZLDPgtUjRZlmY4Dtg+ozoWjJ84lOnvI1WPkeo8TSoU2JP0b29+B4YkeFdnASBLJQtKBr1ILPnHqwtrpMbWni82/PwfulkXZ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final ClosableVuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private VuforiaTrackables   targetsRoverRuckus;        // List of active targets

    // Navigation data is only valid if targetFound == true;
    public boolean             targetFound;    // set to true if Vuforia is currently tracking a target
    public String              targetName;     // Name of the currently tracked target
    public double              robotX;         // X displacement from target center
    public double              robotY;         // Y displacement from target center
    public double              robotZ;
    public double              robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    public double              targetRange;    // Range from robot's center to target in mm
    public double              targetBearing;  // Heading of the target , relative to the robot's unrotated center
    public double              relativeBearing;// Heading to the target from the robot's current bearing.
    public String              latchColor;     // color of latch at start of match, determined by vuforia image
    //   eg: a Positive RelativeBearing means the robot must turn CCW to point at the target image.

	public boolean vuforiaClosed = false;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /* Constructor */
    public VuForiaWebcamClass(){

        targetFound = false;
        targetName = null;
        targetsRoverRuckus = null;

        robotX = 0;
        robotY = 0;
        robotZ = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;

    }

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    //VuforiaLocalizer vuforia;
    ClosableVuforiaLocalizer vuforia;
    WebcamName webcamName;

    /***
     * Send telemetry data to indicate navigation status
     */
    public void addNavTelemetry() {
        if (targetFound)
        {
            // Display the current visible target name, robot info, target info, and required robot action.
            myOpMode.telemetry.addData("Visible", targetName);
//            myOpMode.telemetry.addData("servoPos", myRobot.markerServo.getPosition());
            myOpMode.telemetry.addData("Robot", "[X][Y][Z](B) [%5.0f],[%5.0f],[%5.0f] (%4.0f°)",
                    robotX, robotY, robotZ, robotBearing);
            myOpMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0f] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);
            myOpMode.telemetry.addData("- Turn to square up  ", "%s %4.0f°",  robotBearing < 0 ? ">>> CCW " : "<<< CW", Math.abs(robotBearing));
            myOpMode.telemetry.addData("- Strafe  ", "%s %5.0fin", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            myOpMode.telemetry.addData("- Distance", "%5.0fin", Math.abs(robotX));
        }
        else
        {
            myOpMode.telemetry.addData("Visible", "- - - -" );
        }
    }

    /**
     * return the target name, can be used to determine what quadrant robot is in for auton,
     * can be used to tell if starting on the silver latch or gold latch
     */
    public String getLatchColor(){
        if (targetFound) {
            //return targetName;

            //assumes camera is mounted to look towards right front from robot chassis

            if(targetName.equals("Front-Craters") || targetName.equals("Back-Space")) {
                latchColor = "silver";
            }else {
                latchColor = "gold";
            }
            return latchColor;
        } else {
            latchColor = "None";
            return latchColor;
        }
    }


    /***
     * Start tracking Vuforia images
     */
    public void activateTracking() {

        // Start tracking any of the defined targets
        if (targetsRoverRuckus != null)
            targetsRoverRuckus.activate();
    }

    /***
     * Initialize the Target Tracking and navigation interface
     * @param opMode    pointer to OpMode
     * @param robot     pointer to Robot hardware class
     */
    public void initVuforia(LinearOpMode opMode, MyDropWheelBot robot) {

        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        myRobot = robot;
        webcamName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor.
         * We also indicate which camera on the RC that we wish to use.
         */

        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);  // Use this line to see camera display
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();                             // OR... Use this line to improve performance
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);

		// Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
		// Valid choices are:  BACK or FRONT
		//private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
		//private static final ClosableVuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
		
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

		vuforia = new ClosableVuforiaLocalizer(parameters);
		//or this??
        //ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        //VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 0, 150)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));



        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
/*        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);
*/
        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
/*        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);
*/
        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
/*        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);
*/
        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        /*
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /*

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

		 // TODO  Need to measure this
        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

/*        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? -90 : 90, 0, 0));
*/
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));


        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            trackable.setLocation(targetOrientation);

            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    /***
     * See if any of the vision targets are in sight.
     *
     * @return true if any target is found
     */
    public boolean targetsAreVisible()  {

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

        return (targetFound);
    }

    /***
     * Determine if specified target ID is visible and
     * If it is, retreive the relevant data, and then calculate the Robot and Target locations
     *
     * @param   targetId
     * @return  true if the specified target is found
     */
    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targetsRoverRuckus.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;



        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            //signal the driver that target is found

            //robot.navFlag.setPosition(0.5); //TODO or whatever we deem is raise flag posisiton
            //if(!targetFound) {
                //myRobot.markerServo.setPosition(1.0);
            //}

            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking information
            location  = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0)/ mmPerInch; // in inches
                robotY = trans.get(1) / mmPerInch; // in inches
                robotZ = trans.get(2) / mmPerInch;

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = (targetBearing - robotBearing);//  - 180;
            }



            targetFound = true;
        }
        else  {
            // Indicate that there is no target visible
			//robot.navFlag.setPosition(0.0); //TODO or whatever is lowered flag position
          //  if(targetFound) {
                //myRobot.markerServo.setPosition(0.0);
          //  }

            targetFound = false;
            targetName = "None";
        }


        return targetFound;
    }
    public void flameOn(boolean on){
        if(on) {
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true); //Turns flashlight on.
        }else{
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false); //Turns flashlight on.
        }
    }

	    public void closeVuf()
    {
		vuforiaClosed = true;
        vuforia.close();
        //myOpMode.telemetry.addData("Vuforia", "Closed");
        //myOpMode.telemetry.update();
    }

 
}
