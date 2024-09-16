package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.INTER_CUBIC;
import static org.opencv.imgproc.Imgproc.MORPH_ELLIPSE;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.blur;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.getStructuringElement;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;
import static org.opencv.imgproc.Imgproc.resize;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * We will need to two pipelines for vision code
 * <p>
 * We don't need to get have a update loop because of the pipelines
 */
public class Vision {
    // openCV webcam
    private OpenCvCamera webcam;
    // The int of the cube location that will be used for auto
    private static int cubeLocation;

    /**
     * These are the sample locations (where the robot will look for the stones)
     */

    public static double errorLeft = 0;
    public static double errorRight = 0;


    public static double h = 0;
    public static double s = 0;
    public static double v = 0;

    private static double sample1X = 60;
    private static double sample2X = 205;
    private static double sample3X = 240;

    private static double sample1Y = 130;
    private static double sample2Y = 120;
    private static double sample3Y = 140;

    //these are how you set teh sample
    public static double getSample1X() {
        return sample1X;
    }

    public static double getSample2X() {
        return sample2X;
    }

    public static double getSample3X() {
        return sample3X;
    }

    public static double getSample1Y() {
        return sample1Y;
    }

    public static double getSample2Y() {
        return sample2Y;
    }

    public static double getSample3Y() {
        return sample3Y;
    }

    public static void setSample1X(double x) {
        sample1X = x;
    }

    public static void setSample2X(double x) {
        sample2X = x;
    }

    public static void setSample3X(double x) {
        sample3X = x;
    }

    public static void setSample1Y(double y) {
        sample1Y = y;
    }

    public static void setSample2Y(double y) {
        sample2Y = y;
    }

    public static void setSample3Y(double y) {
        sample3Y = y;
    }

    /**
     * Vision constructor
     *
     * @param webcam - instance
     */
    public Vision(OpenCvCamera webcam) {
        this.webcam = webcam;
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        //webcam.setPipeline(new VisionTest.Pipeline2());

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);*/
    }

    public void openCameraDevice() {
        webcam.openCameraDevice();
    }

    public void setPipeline(OpenCvPipeline openCvPipeline) {
        webcam.setPipeline(openCvPipeline);
    }

    public void startStreaming(int width, int height, OpenCvCameraRotation openCvCameraRotation) {
        webcam.startStreaming(width, height, openCvCameraRotation);
    }

    /**
     * pausing the vision viewport to save data usage
     */
    public void pauseVisionViewport() {
        webcam.pauseViewport();
    }

    /**
     * resume the vision viewport from pausing it
     */
    public void resumeVisionViewport() {
        webcam.resumeViewport();
    }

    /**
     * stop streaming from the webcam in a opmode
     */
    public void stopStreamingVision() {
        webcam.stopStreaming();
    }

    /**
     * gets the cube location from auto init loop
     *
     * @return integer of which pos it is in
     */
    public static int getCubeLocation() {
        return cubeLocation;
    }

    public static Mat mat = new Mat();

    public static boolean weAreOnRightSide = false;

    public static boolean isRed = false;

    /**
     * Auto pipeline
     */
    public static class AutoVisionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            double scaleSize = 8;
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB, 3);

            Mat processingImage = new Mat();

            resize(input, processingImage, new Size(), 1 / scaleSize, 1 / scaleSize, INTER_CUBIC);
            blur(processingImage, processingImage, new Size(3, 3));

            Mat hsv = new Mat();
            Imgproc.cvtColor(processingImage, hsv, Imgproc.COLOR_RGB2HSV, 3);

            double minHue = 25.0;
            double maxHue = 30.0;

            // #1
            int hSum = 0;
            int sSum = 0;
            int vSum = 0;
            int pixelCount = 0;

            double sampleRadius = 20;
            int sampleX_start = (int) ((sample1X - sampleRadius) / scaleSize);
            int sampleX_end = (int) ((sample1X + sampleRadius) / scaleSize);
            int sampleY_start = (int) ((sample1Y - sampleRadius) / scaleSize);
            int sampleY_end = (int) ((sample1Y + sampleRadius) / scaleSize);

            for (int y = sampleY_start; y < sampleY_end; y++) {
                for (int x = sampleX_start; x < sampleX_end; x++) {
                    hSum += hsv.get(y, x)[0];
                    sSum += hsv.get(y, x)[1];
                    vSum += hsv.get(y, x)[2];
                    pixelCount++;
                }
            }

            double hAverage1 = (double) hSum / pixelCount;
            double sAverage1 = (double) sSum / pixelCount;
            double vAverage1 = (double) vSum / pixelCount;

            rectangle(input, new Point(sampleX_start * (int) scaleSize, sampleY_start * (int) scaleSize),
                    new Point(sampleX_end * (int) scaleSize, sampleY_end * (int) scaleSize),
                    new Scalar(255, 255, 0), 3);

            // #2
            sampleX_start = (int) ((sample2X - sampleRadius) / scaleSize);
            sampleX_end = (int) ((sample2X + sampleRadius) / scaleSize);
            sampleY_start = (int) ((sample2Y - sampleRadius) / scaleSize);
            sampleY_end = (int) ((sample2Y + sampleRadius) / scaleSize);
            hSum = 0;
            sSum = 0;
            vSum = 0;
            pixelCount = 0;
            for (int y = sampleY_start; y < sampleY_end; y++) {
                for (int x = sampleX_start; x < sampleX_end; x++) {
                    hSum += hsv.get(y, x)[0];
                    sSum += hsv.get(y, x)[1];
                    vSum += hsv.get(y, x)[2];
                    pixelCount++;
                }
            }
            //convert the hAverage back to degrees so we can use it
            double hAverage2 = (double) hSum / pixelCount;
            double sAverage2 = (double) sSum / pixelCount;
            double vAverage2 = (double) vSum / pixelCount;
            //draws where the sample was taken
            rectangle(input, new Point(sampleX_start * (int) scaleSize, sampleY_start * (int) scaleSize),
                    new Point(sampleX_end * (int) scaleSize, sampleY_end * (int) scaleSize),
                    new Scalar(255, 255, 0), 3);


            /*Imgproc.rectangle(input, new Point(sampleX_start * (int) scaleSize, sampleY_start * (int) scaleSize),
                    new Point(sampleX_end * (int) scaleSize, sampleY_end * (int) scaleSize),
                    new Scalar(0, 0, 255), 3);*/

            // #3
            sampleX_start = (int) ((sample3X - sampleRadius) / scaleSize);
            sampleX_end = (int) ((sample3X + sampleRadius) / scaleSize);
            sampleY_start = (int) ((sample3Y - sampleRadius) / scaleSize);
            sampleY_end = (int) ((sample3Y + sampleRadius) / scaleSize);

            hSum = 0;
            sSum = 0;
            vSum = 0;
            pixelCount = 0;
            for (int y = sampleY_start; y < sampleY_end; y++) {
                for (int x = sampleX_start; x < sampleX_end; x++) {
                    hSum += hsv.get(y, x)[0];
                    sSum += hsv.get(y, x)[1];
                    vSum += hsv.get(y, x)[2];
                    pixelCount++;
                }
            }
            //convert the hAverage back to degrees so we can use it
            double hAverage3 = (double) hSum / pixelCount;
            double sAverage3 = (double) sSum / pixelCount;
            double vAverage3 = (double) vSum / pixelCount;
            //draws where the sample was taken
            //rectangle(input, new Point(sampleX_start * (int) scaleSize, sampleY_start * (int) scaleSize),
            //        new Point(sampleX_end * (int) scaleSize, sampleY_end * (int) scaleSize),
            //        new Scalar(255, 255, 0), 3);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            double targetHue = 95;
            double targetV = 28;
            double targetS = 130;

            if (isRed) {
                targetHue = 10;
                targetV = 113;
                targetS = 131;
            }

            h = hAverage1;
            s = sAverage1;
            v = vAverage1;

            double error1 =
                    Math.abs(hAverage1 - targetHue) + Math.abs(sAverage1 - targetS) + Math.abs(vAverage1 - targetV);
            double error2 =
                    Math.abs(hAverage2 - targetHue) + Math.abs(sAverage2 - targetS) + Math.abs(vAverage2 - targetV);

            errorLeft = error1;
            errorRight = error2;
            //double error3 =
            //        Math.abs(hAverage3 - targetHue) + Math.abs(sAverage3 - targetS) + Math.abs(vAverage3 - targetV);

            //merror1 = error1;
            //merror2 = error2;
            //merror3 = error3;

            /*if (error1 <= 200 && error1 <= error2) {
                circle(input, new Point(sample1X, sample1Y), 20, new Scalar(255, 255, 0), -1);
                whereIsCube = 0;
            } else {
                if (error2 <= 200 && error2 <= error1) {
                    circle(input, new Point(sample2X, sample2Y), 20, new Scalar(255, 255, 0), -1);
                    whereIsCube = 1;
                } else {
                    circle(input, new Point(sample3X, sample3Y), 20, new Scalar(255, 255, 0), -1);
                    whereIsCube = 2;
                }
            }*/

            double errorTarget = isRed ? 125 : 185;

            /*
            see which value is the highest then determine the sample pos
             */
            if (!weAreOnRightSide) {
                if (error1 < errorTarget) {
                    circle(input, new Point(sample1X - 5, sample1Y - 5), 20, new Scalar(255, 255, 0), -1);
                    cubeLocation = 0;
                } else if (error2 < errorTarget) {
                    circle(input, new Point(sample2X - 5, sample2Y - 5), 20, new Scalar(255, 255, 0), -1);
                    cubeLocation = 1;
                } else {
                    cubeLocation = 2;
                }/*else if (error3 > 300) {
                circle(input, new Point(sample3X - 5, sample3Y - 5), 20, new Scalar(255, 255, 0), -1);
                cubeLocation = 2;
            }*/
            } else {
                if (error1 < errorTarget) {
                    circle(input, new Point(sample1X - 5, sample1Y - 5), 20, new Scalar(255, 255, 0), -1);
                    cubeLocation = 1;
                } else if (error2 < errorTarget) {
                    circle(input, new Point(sample2X - 5, sample2Y - 5), 20, new Scalar(255, 255, 0), -1);
                    cubeLocation = 2;
                } else {
                    cubeLocation = 0;
                }
            }

            Mat booleanImage = new Mat();

            Core.inRange(hsv, new Scalar(minHue, 100, 75), new Scalar(maxHue, 255, 255), booleanImage);

            return input;
        }
    }

    public static double angle;
    public static double moments;

    /**
     * StoneDetector pipeline
     */
    public static class StoneDetector extends OpenCvPipeline {
        List<MatOfPoint> contours = new ArrayList<>();

        int iLowH = 95;
        int iHighH = 107;
        int iLowS = 125;
        int iHighS = 255;
        int iLowV = 0;
        int iHighV = 255;

        @Override
        public Mat processFrame(Mat input) {
            // Convert the captured frame from BGR to HSV
            Mat imgHSV = new Mat();
            cvtColor(input, imgHSV, COLOR_BGR2HSV);

            Mat imgThresholded = new Mat();
            Core.inRange(imgHSV, new Scalar(iLowH, iLowS, iLowV), new Scalar(iHighH, iHighS, iHighV), imgThresholded);

            // Noise Reduction using Mathematical Morphology
            // Morphological Opening (Removes small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));

            // Morphological Closing (Removes small holes from the foreground)
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));

            contours = new ArrayList<>();

            // Calculate the Moments of the Thresholded Image
            Moments oMoments = moments(imgThresholded);

            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;

            moments = dArea;

            // If the area <= 10000, consider that it's because of the noise
            if (dArea > 10000) {
                // Calculate the Centroid of the Object
                int posX = (int) (dM10 / dArea);
                int posY = (int) (dM01 / dArea);

                // Draw a Red Circle tracking the Object
                int R = (int) Math.sqrt((dArea / 255) / 3.14);
                if (posX >= 0 && posY >= 0) {
                    circle(input, new Point(posX, posY), 10, new Scalar(0, 0, 255), 2);
                    angle = (posX - 160) * 0.24375; // TODO: FIX
                }

                // Calculate the Distance between the Object and the camera
                int realR = 4;
                int f = -10;
                int fpx = 750;
                int d = (realR * fpx) / R + f;
                putText(input, "Distance: " + d, new Point(100, 100), 1, 2,
                        new Scalar(0, 0, 255), 1);
            } else {
                angle = 0;
            }

            findContours(imgThresholded, contours, new Mat(), RETR_LIST, CHAIN_APPROX_SIMPLE);

            for (int i = 0; i < contours.size(); i++) {
                drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, 8);
            }

            //byte[] return_buff = new byte[(int) (input.total() * input.channels())]; input.get(0, 0, return_buff);

            input.copyTo(mat);

            return input;
        }
    }
}