package com.disnodeteam.dogecv.filters;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Victo on 1/1/2018.
 */

public class BattleBlazerFilter extends DogeCVColorFilter{

    private Scalar upper = new Scalar(150,255,255);
    private Scalar lower = new Scalar(75,255,255);

    public BattleBlazerFilter(Scalar lower, Scalar upper){
        this.upper = upper;
        this.lower = lower;
    }

    public void updateSettings(Scalar lower, Scalar upper){
        this.upper = upper;
        this.lower = lower;
    }

    @Override
    public void process(Mat input, Mat mask) {
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.GaussianBlur(input,input,new Size(5,5),0);

        //Scalar lower = new Scalar(perfect.val[0] - range.val[0], perfect.val[1] - range.val[1],perfect.val[2] - range.val[2]);
        //Scalar upper = new Scalar(perfect.val[0] + range.val[0], perfect.val[1] + range.val[1],perfect.val[2] + range.val[2]);
        Core.inRange(input,lower,upper,mask);
        input.release();
    }
}
