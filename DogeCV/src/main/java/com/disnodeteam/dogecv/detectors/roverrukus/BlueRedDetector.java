package com.disnodeteam.dogecv.detectors.roverrukus;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/10/2018.
 */

public class BlueRedDetector extends DogeCVDetector {

    // Enum to describe gold location
    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    // Which area scoring method to use
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

    //Create the scorers used for the detector
    public RatioScorer ratioScorer             = new RatioScorer(1.0,5);
    public MaxAreaScorer maxAreaScorer         = new MaxAreaScorer(0.01);
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(10000,0.05);

    //Create the filters used
    public DogeCVColorFilter redFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED,100);
    public DogeCVColorFilter blueFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE,50);


    // Results for the detector
    private GoldLocation currentOrder = GoldLocation.UNKNOWN;
    private GoldLocation lastOrder    = GoldLocation.UNKNOWN;
    private boolean      isFound      = false;

    // Create the mats used
    private Mat workingMat  = new Mat();
    private Mat displayMat  = new Mat();
    private Mat redMask  = new Mat();
    private Mat blueMask   = new Mat();
    private Mat hiarchy     = new Mat();

    public BlueRedDetector() {
        super();
        this.detectorName = "Blue Red Detector";
    }

    @Override
    public Mat process(Mat input) {

        // Copy input mat to working/display mats
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();

        // Generate Masks
        redFilter.process(workingMat.clone(), redMask);
        blueFilter.process(workingMat.clone(), blueMask);


        // Blur and find the countours in the masks
        List<MatOfPoint> contoursRed = new ArrayList<>();
        List<MatOfPoint> contoursBlue = new ArrayList<>();

        Imgproc.blur(blueMask,blueMask,new Size(2,2));
        Imgproc.blur(redMask,redMask,new Size(2,2));

        Imgproc.findContours(redMask, contoursRed, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursRed,-1,new Scalar(230,70,70),2);

        Imgproc.findContours(blueMask, contoursBlue, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlue,-1,new Scalar(230,70,70),2);


        // Prepare to find best red (gold) results
        Rect   chosenRedRect  = null;
        double chosenRedScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursRed){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);
            double redRectArea = rect.area();

            double diffrenceScore = calculateScore(points);

            if(diffrenceScore < chosenRedScore && diffrenceScore < maxDifference && redRectArea > 10000){
                chosenRedScore = diffrenceScore;
                chosenRedRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if( area > 500){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }
		
        // Prepare to find best blue (gold) results
        Rect   chosenBlueRect  = null;
        double chosenBlueScore = Integer.MAX_VALUE;

        //MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursBlue){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double blueRectArea = rect.area();
            double diffrenceScore = calculateScore(points);

            if(diffrenceScore < chosenBlueScore && diffrenceScore < maxDifference && blueRectArea > 10000){
                chosenBlueScore = diffrenceScore;
                chosenBlueRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if( area > 500){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }		
		
		
        //Draw found red element
        if(chosenRedRect != null){
            Imgproc.rectangle(displayMat,
                    new Point(chosenRedRect.x, chosenRedRect.y),
                    new Point(chosenRedRect.x + chosenRedRect.width, chosenRedRect.y + chosenRedRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenRedScore, (double)chosenRedRect.x),
                    new Point(chosenRedRect.x - 5, chosenRedRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

        }
        //Draw found Blue element
        if(chosenBlueRect != null){
            Imgproc.rectangle(displayMat,
                    new Point(chosenBlueRect.x, chosenBlueRect.y),
                    new Point(chosenBlueRect.x + chosenBlueRect.width, chosenBlueRect.y + chosenBlueRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenBlueScore, (double)chosenBlueRect.x),
                    new Point(chosenBlueRect.x - 5, chosenBlueRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

        }		
		
		
        // If enough elements are found, compute gold position
        if(chosenBlueRect != null  || chosenRedRect != null){

            isFound = true;


        }else{

            isFound = false;
        }

        //Display Debug Information
//        Imgproc.putText(displayMat,"Gold Position: " + lastOrder.toString(),new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);
//        Imgproc.putText(displayMat,"Current Track: " + currentOrder.toString(),new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);

        return displayMat;
    }

    @Override
    public void useDefaults() {
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
        addScorer(ratioScorer);
    }

    /**
     * Is both elements found?
     * @return if the elements are found
     */
    public boolean isFound() {
        return isFound;
    }

    /**
     * Returns the current gold pos
     * @return current gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getCurrentOrder() {
        return currentOrder;
    }

    /**
     * Returns the last known gold pos
     * @return last known gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldLocation getLastOrder() {
        return lastOrder;
    }
}
