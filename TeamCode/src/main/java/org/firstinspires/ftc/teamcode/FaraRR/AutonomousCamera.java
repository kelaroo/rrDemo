package org.firstinspires.ftc.teamcode.FaraRR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(group = "Demo")
public class AutonomousCamera extends LinearOpMode {

    HardwareConfig hw;
    OpenCvCamera webcam;
    ElapsedTime time = new ElapsedTime();

    static public int H_MIN = 9;    // 9
    static public int H_MAX = 45;   //65
    static public int S_MIN = 60;   //60
    static public int S_MAX = 256;  //256
    static public int V_MIN = 95;   //95
    static public int V_MAX = 256;  //256

    final static private int ERODE_SIZE = 3;
    final static private int DILATE_SIZE = 8;

    RingsDetectionPipeline.RingsNumber ringsNumber;

    @Override
    public void runOpMode() throws InterruptedException {
        initCamera();
        hw = new HardwareConfig(hardwareMap);

        /// men deci lasa asta sa se updateze pe telemetry ca dureaza mult initializarea si ia frameul trecut (de la ultimu program)
        while(!isStarted()) {
            ringsNumber = RingsDetectionPipeline.ringsNumber;
            if(ringsNumber == RingsDetectionPipeline.RingsNumber.FOUR) {
                telemetry.addData("detection", "FOUR");
            } else if(ringsNumber == RingsDetectionPipeline.RingsNumber.ONE) {
                telemetry.addData("detection", "ONE");
            } else {
                telemetry.addData("detection", "NONE");
            }
            telemetry.addData("nrPixels", RingsDetectionPipeline.nrPixels);
            telemetry.update();
        }

        //waitForStart();
        if(ringsNumber == RingsDetectionPipeline.RingsNumber.FOUR) {
            telemetry.addData("finalDetection", "FOUR");
        } else if(ringsNumber == RingsDetectionPipeline.RingsNumber.ONE) {
            telemetry.addData("finalDetection", "ONE");
        } else {
            telemetry.addData("finalDetection", "NONE");
        }
        telemetry.update();

        if(ringsNumber == RingsDetectionPipeline.RingsNumber.FOUR) {
            // Mergi Zona A (sau care o fi)
            moveAndDropWobble();
            time.reset();
            while(time.seconds() < 2)
                ;
            // mergi la wobble 2
            getWobble();
        } else if(ringsNumber == RingsDetectionPipeline.RingsNumber.ONE) {
            // Mergi Zona B (sau care o fi)
            moveAndDropWobble();
            time.reset();
            while(time.seconds() < 2)
                ;
            // mergi la wobble 2
            getWobble();
        } else {
            // Mergi Zona C (sau care o fi)
            moveAndDropWobble();
            time.reset();
            while(time.seconds() < 2)
                ;
            // mergi la wobble 2
            getWobble();
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new AutonomousCamera.RingsDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    private void moveAndDropWobble() { // da peste cap si lasa wobble
        hw.brat.setTargetPosition(450);
        hw.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        time.reset();
        hw.brat.setPower(0.2);
        while(hw.brat.isBusy() && time.seconds() < 2)
            ;
        hw.brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.brat.setPower(0);

        time.reset();
        while(time.milliseconds() < 500)
            ;
        hw.apucat.setPosition(0.5);

        /*hw.brat.setTargetPosition(20);
        hw.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        time.reset();
        hw.brat.setPower(0.5);
        while(hw.brat.isBusy() && time.seconds() < 2)
            ;
        hw.brat.setPower(0);
        hw.brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }

    private void getWobble() { // pur si simplu apuca wobble (cred ca trebuie sa mergi in el)
        hw.apucat.setPosition(0.2);
        /*time.reset();
        while(time.milliseconds() < 500)
            ;
        hw.brat.setTargetPosition(20);
        hw.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        time.reset();
        hw.brat.setPower(0.5);
        while(hw.brat.isBusy() && time.seconds() < 2)
            ;
        hw.brat.setPower(0);
        hw.brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }


    public static class RingsDetectionPipeline extends OpenCvPipeline {

        enum RingsNumber {
            FOUR,
            ONE,
            NONE
        };

        static RingsNumber ringsNumber;
        static int nrPixels = -1;

        @Override
        public Mat processFrame(Mat input) {
            Rect cropRect = new Rect(120, 70, 100, 100);
            Imgproc.rectangle(input, cropRect, new Scalar(0, 255, 0));
            Mat filtered = input.clone();
            Imgproc.cvtColor(filtered, filtered, Imgproc.COLOR_RGB2HSV);

            Core.inRange(filtered, new Scalar(H_MIN, S_MIN, V_MIN), new Scalar(H_MAX, S_MAX, V_MAX), filtered);

            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(ERODE_SIZE, ERODE_SIZE));
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(DILATE_SIZE, DILATE_SIZE));
            Imgproc.erode(filtered, filtered, erodeElement);
            Imgproc.dilate(filtered, filtered, dilateElement);

            erodeElement = null;
            dilateElement = null;

            nrPixels = Core.countNonZero(filtered);
            if(nrPixels >= 3000) {
                ringsNumber = RingsNumber.FOUR;
            } else if(nrPixels >= 800) {
                ringsNumber = RingsNumber.ONE;
            } else {
                ringsNumber = RingsNumber.NONE;
            }

            return filtered;
        }

        public RingsNumber getNumberOfRings() {return ringsNumber;}
        public int getNrPixels() {return nrPixels;}
    }
}
