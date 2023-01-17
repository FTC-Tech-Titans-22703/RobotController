package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360;
    public static double WHEEL_RADIUS_SIDE = 1.77065; // in
    public static double WHEEL_RADIUS_MIDDLE = 1.98; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.5; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 24.5 / 24.1048 - 0.02; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 24.5 / 23.0932; // Multiplier in the Y direction

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInchesSide(double ticks) {
        return WHEEL_RADIUS_SIDE * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double encoderTicksToInchesMiddle(double ticks) {
        return WHEEL_RADIUS_MIDDLE * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInchesSide(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInchesSide(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInchesMiddle(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInchesSide(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInchesSide(rightEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInchesMiddle(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
