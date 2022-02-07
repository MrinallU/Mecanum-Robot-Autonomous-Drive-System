package org.firstinspires.ftc.teamcode.T3_2022.Modules;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;

public class T3_Camera {
    private static final String VUFORIA_KEY = "AWnPBRj/////AAABmaYDUsaxX0BJg7/6QOpapAl4Xf18gqNd7L9nALxMG8K2AF6lodTZQ78nnksFc2CMy/3KmeolDEFGmp0CQJ7c/5PKymmJYckCfsg16B6Vnw5OihuD2mE7Ky0tT1VGdit2KvolunYkjWKDiJpX15SFMX//Jclt+Xt8riZqh3edXpUdREIXxS9tmdF/O6Nc5mUI7FEfAJHq4xUaqSY/yta/38qirjy3tdqFjDGc9g4DmgPE6+6dGLiXeUJYu32AgoefA1iFRF+ZVNJEc1j4oyw3JYQgWwfziqyAyPU2t9k9UDgqEkyxGxl4xS70KN/SBEUZeq4CzYfyon2kSSvKK/6/Vt4maMzG3LXfLt0PMiEPI1z+";
    private VuforiaLocalizer vuforia;
    private HardwareMap hardwareMap;

    private int blueThreshold = 100;
    private int redThreshold = 70;

    public String outStr = "";


    // Rings Are YELLOW (255 R and 255 G)
    // Just look at blue value

    public T3_Camera(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        initVuforia();
    }

    public int readBarcode(String auto) throws InterruptedException{
        int position = 0;
        Bitmap bm;
        double[] posOne = new double[4];
        double[] posTwo = new double[4];
        double[] posThree = new double[4];

        try {
            VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();
            bm = vuforia.convertFrameToBitmap(closeableFrame);
        } catch (Exception e){
            return 1;
        }

        if(auto == "redPrimary"){
            posOne = calculateAverageRGB(bm, 113, 76, 136, 90);
            posTwo = calculateAverageRGB(bm, 512, 69, 600, 120);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "redSecondary"){
            posOne = calculateAverageRGB(bm, 448, 48, 550, 105);
            posTwo = calculateAverageRGB(bm, 36, 52, 134, 113);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "bluePrimary"){
            posOne = calculateAverageRGB(bm, 420, 16, 535, 75);
            posTwo = calculateAverageRGB(bm, 27 , 26, 127, 94);
            posThree = calculateAverageRGB(bm, 0, 0, 0, 0);
        }else if(auto == "blueSecondary"){
//            posOne = calculateAverageRGB(bm, 804, 69, 869, 115);
            posTwo = calculateAverageRGB(bm, 360, 82, 486, 116);
            posThree = calculateAverageRGB(bm, 76, 81, 187, 125);
        }

        saveImage();
        if(auto == "redPrimary"){
            if(posOne[3] < redThreshold){
                return 0;
            }else if(posTwo[3] < redThreshold){
                return 1;
            }else{
                return 2;
            }
        }else if(auto == "bluePrimary"){
            if(posOne[1] < redThreshold){
                return 0;
            }else if(posTwo[1] < redThreshold){
                return 1;
            }else{
                return 2;
            }
        }else if(auto == "redSecondary"){
            if(posOne[3] < redThreshold){
                return 0;
            }else if(posTwo[3] < redThreshold){
                return 1;
            }else{
                return 2;
            }
        } else{
            if (posThree[1] < redThreshold) {
                return 2;
            } else if (posTwo[1] < redThreshold) {
                return 1;
            } else {
                return 0;
            }
        }
    }


    public void saveImage(){
        try{
            File outF = AppUtil.getInstance().getSettingsFile("Boxes.png");
            FileOutputStream out = new FileOutputStream(outF);

            Bitmap bm;

            // Link to image visualizer: https://yangcha.github.io/iview/iview.html

            VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();

            bm = vuforia.convertFrameToBitmap(closeableFrame);
            drawRectangle(bm, 113, 76, 136, 90);
             drawRectangle(bm, 512, 69, 600, 120);

            bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void saveImage(int id){
        try{
            File outF = AppUtil.getInstance().getSettingsFile("Picture" +id +".png");
            FileOutputStream out = new FileOutputStream(outF);

            Bitmap bm;

            // Link to image visualizer: https://yangcha.github.io/iview/iview.html

            VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();

            bm = vuforia.convertFrameToBitmap(closeableFrame);


            bm.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e){
            e.printStackTrace();
        }
    }

    public void drawRectangle(Bitmap bm, int left, int top, int right, int bottom){
        for(int x = left; x <= right; x++){
            for(int y = top; y <= bottom; y++){
                if(x == left || x == right || y == top || y == bottom){
                    bm.setPixel(x, y, Color.BLACK);
                }
            }
        }
    }

    public double[] calculateAverageRGB(Bitmap bm, int left, int top, int right, int bottom){
        long numTotalPixels = 0;
        double[] colorValues = {0, 0, 0, 0};
        for(int x = left; x <= right; x++){
            for(int y = top; y <= bottom; y++){
                int pixel = bm.getPixel(x, y);
                colorValues[0] += Color.alpha(pixel);
                colorValues[1] += Color.red(pixel);
                colorValues[2] += Color.green(pixel);
                colorValues[3] += Color.blue(pixel);
                numTotalPixels++;
            }
        }

        for(int i = 0; i < colorValues.length; i++){
            colorValues[i] = colorValues[i] / numTotalPixels;
        }

        return colorValues;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.setFrameQueueCapacity(1);

        vuforia.enableConvertFrameToBitmap();

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public String argbToText(double[] arr){
        return String.format("A: %.2f, R: %.2f, G: %.2f, B: %.2f", arr[0], arr[1], arr[2], arr[3]);
    }
}
