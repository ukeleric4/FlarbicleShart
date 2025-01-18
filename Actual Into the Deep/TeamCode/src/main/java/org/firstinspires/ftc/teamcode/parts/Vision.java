package org.firstinspires.ftc.teamcode.parts;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class Vision {
    List<ColorBlobLocatorProcessor.Blob> blobs;
    ColorBlobLocatorProcessor colorLocatorBlue;
    ColorBlobLocatorProcessor colorLocatorYellow;
    ColorBlobLocatorProcessor colorLocatorRed;
    ColorBlobLocatorProcessor.BlobFilter areaFilter;
    VisionPortal portal;
    ColorBlobLocatorProcessor.Blob block;
    RotatedRect boxFit;

    double angle1 = 0;
    double angle2 = 0;

    int centerX = 160;
    int centerY = 120;

    double multipler = 0.025;

    Telemetry telemetry;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        // get claw orientation servo
        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .build();
        colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .build();
        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .build();
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorYellow)
                .addProcessor(colorLocatorRed)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,50, 20000);
    }

    public void updateVision() {
        blobs = colorLocatorBlue.getBlobs();
        if (!blobs.isEmpty()) {
            block = blobs.get(0);

            boxFit = block.getBoxFit();
            angle1 = boxFit.angle;
            if (boxFit.size.width < boxFit.size.height) {
                angle2 = angle1 - 90;
            }
        }
    }

    public ColorBlobLocatorProcessor.Blob getFirstBlock() {
        blobs = colorLocatorBlue.getBlobs();
        if (!blobs.isEmpty()) {
            block = blobs.get(0);
            if (block.getContourArea() < 1000) {
                return block;
            }
            return null;
        }
        return null;
    }

    public double getXMovement() {
        ColorBlobLocatorProcessor.Blob blob = getFirstBlock();
        if (blob != null) {
            double x = blob.getBoxFit().center.x;
            if (x < centerX) {
                return -(multipler * (centerX - x));
            } else {
                return multipler * (x - centerX);
            }
        }
        return 0.01;
    }

    public double getYMovement() {
        ColorBlobLocatorProcessor.Blob blob = getFirstBlock();
        if (blob != null) {
            double y = blob.getBoxFit().center.y;
            if (y < centerY) {
                return multipler * (centerY - y);
            } else {
                return -(multipler * (y - centerY));
            }
        }
        return 0.01;
    }

    public double getYPixels() {
        ColorBlobLocatorProcessor.Blob blob = getFirstBlock();
        if (blob != null) {
            double y = blob.getBoxFit().center.y;
            if (y < centerY) {
                return (centerY - y);
            } else {
                return -(y - centerY);
            }
        }
        return 0.01;
    }

    public double getOrientation() {
        if (block != null) {
            angle1 = boxFit.angle;
            if (boxFit.size.width < boxFit.size.height) {
                angle2 = angle1 - 90;
                if (angle2 > -90 && angle2 < -60) {
                    return 0;
                }
                if (angle2 > -60 && angle2 < -30) {
                    return 0.55;
                }
                if (angle2 > -30 && angle2 < 0) {
                    return 0.3;
                }

            } else if (angle1 != 0 && angle1 != 90){
                if (angle1 > 0 && angle1 < 30){
                    return 0.3;
                }
                if (angle1 >= 30 && angle1 <= 60){
                    return 0.15;
                }
                if (angle1 > 60 && angle1 < 90){
                    return 0;
                }
            }
            return 0.15;
        }
        return 0.01;
    }
}
