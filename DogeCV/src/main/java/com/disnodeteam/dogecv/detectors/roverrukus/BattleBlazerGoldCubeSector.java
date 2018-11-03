package com.disnodeteam.dogecv.detectors.roverrukus;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.BattleBlazerFilter;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/17/2018.
 */

public class BattleBlazerGoldCubeSector extends DogeCVDetector {

    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    private Mat workingMat = new Mat();
    private Mat blurredMat  = new Mat();
    private Mat maskYellow = new Mat();
    private Mat hiarchy  = new Mat();
    private Mat structure = new Mat();
    private Size stretchKernal = new Size(10,10);
    private Size newSize = new Size();


    private boolean found = false;
    private boolean aligned = false;
    private double goldXPos = 0;
    private GoldLocation goldSector = GoldLocation.UNKNOWN;


    public boolean debugAlignment = true;
    public boolean debugContours  = true;
    public boolean stretch        = true;
    //public double minArea = 50;
	public double minArea = 250;
    public double alignPosOffset = 0;
    public double alignSize = 100;
	public double areaMin = 400; // added to check for minimum area	

    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
    public Scalar upper=new Scalar(150,255,255);
    public Scalar lower=new Scalar(10,150,150);
    public DogeCVColorFilter yellowFilter   = new BattleBlazerFilter(lower,upper);
    public RatioScorer      ratioScorer        = new RatioScorer(2.5, 3);
    public MaxAreaScorer    maxAreaScorer      = new MaxAreaScorer( 0.01);
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(1000,0.05);

    @Override
    public Mat process(Mat input) {
        if(input.channels() < 0 || input.cols() <= 0){
            Log.e("DogeCV", "Bad INPUT MAT!");

        }
        input.copyTo(workingMat);
        input.release();

        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(),maskYellow);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(maskYellow, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursYellow,-1,new Scalar(75,255,255),2);


        Rect bestRect = null;
        double bestDiffrence = Double.MAX_VALUE;

        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont);


            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);

            Imgproc.rectangle(workingMat, rect.tl(), rect.br(), new Scalar(45,200,115),2);

			double area = Imgproc.contourArea(cont);
			if(score < bestDiffrence && area > areaMin){
            // original ****   if(score < bestDiffrence){
                bestDiffrence = score;
                bestRect = rect;
            }
        }


        double alignX = (getAdjustedSize().width / 2) + alignPosOffset;
        double alignXMin = alignX - (alignSize / 2);
        double alignXMax = alignX +(alignSize / 2);
        double xPos = 0;



        if(bestRect != null){
            Imgproc.rectangle(workingMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(workingMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;
            Imgproc.circle(workingMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            if(xPos < alignXMax && xPos > alignXMin){
                aligned = true;
            }else{
                aligned = false;
            }

            // Check if gold x position is in left third, center third or right third of screen
            if(xPos > alignXMax){
                goldSector = GoldLocation.RIGHT;
            } else if (xPos < alignXMin){
                goldSector = GoldLocation.LEFT;
            }else if (xPos < alignXMax && xPos > alignXMin){
                goldSector = GoldLocation.CENTER;
            } else {
                goldSector = GoldLocation.UNKNOWN;
            }


            Imgproc.line(workingMat,new Point(xPos, getAdjustedSize().height), new Point(xPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);
            Imgproc.putText(workingMat,"Current X: " + bestRect.x,new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);

            found = true;
        }else{
            found = false;
            aligned = false;
        }
        if(debugAlignment){

            Imgproc.line(workingMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
            Imgproc.line(workingMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        }





        //Imgproc.putText(workingMat,"Result: " + aligned,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(workingMat,"Sector: " + goldSector,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);



        Imgproc.putText(workingMat," Gold Align: " + getAdjustedSize().toString() + " - " + speed.toString() ,new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    public boolean getAligned(){
        return aligned;
    }

    public double getXPosition(){
        return goldXPos;
    }


    public boolean isFound() {
        return found;
    }

    public GoldLocation whichSector(){ return goldSector; }
}
