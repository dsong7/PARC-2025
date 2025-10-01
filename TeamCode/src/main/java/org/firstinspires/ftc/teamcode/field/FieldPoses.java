package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.HashMap;
import java.util.Map;

/**
 * All DECODE™ field constants & landmark poses live here.
 * !! TODO: Replace placeholder numbers with official values from the CAD / Competition Manual TU02.
 *
 * Coordinate frame: FTC Field Coordinate System (X right, Y up from field origin; headings in radians CCW).
 */
public final class FieldPoses {

    private FieldPoses() {}

    // ---- AprilTag config (YOU MUST FILL THESE) ----
    // Set to the DECODE official tag size (meters). See manual TU02 "Vision Targets / AprilTags".
    public static final double APRILTAG_SIZE_METERS = 0.165; // TODO: Replace with official DECODE tag size!

    // Field-fixed poses (inches) for each tag center and orientation (heading radians).
    // Use the FTC “Uncover the Field”/manual resources for exact positions.
    // Example placeholders:
    public static final class TagPose {
        public final Pose2d pose; // field pose of the tag center (x,y,heading)
        public TagPose(Pose2d p){ this.pose = p; }
    }

    public static final Map<Integer, TagPose> TAG_POSES = new HashMap<>();
    static {
        // TODO: Replace with real IDs/poses from DECODE field map
        TAG_POSES.put(1, new TagPose(new Pose2d(  12.0, 141.0, Math.toRadians(180)))); // Example “Audience wall” tag
        TAG_POSES.put(2, new TagPose(new Pose2d(  36.0, 141.0, Math.toRadians(180))));
        TAG_POSES.put(3, new TagPose(new Pose2d(  60.0, 141.0, Math.toRadians(180))));
        TAG_POSES.put(4, new TagPose(new Pose2d(  84.0, 141.0, Math.toRadians(180))));
        // Add the rest per manual/CAD...
    }

    // ---- Start poses (set exactly to your legal starting squares per DECODE manual) ----
    public static final Pose2d RED_LEFT_START  = new Pose2d(12.0, 12.0, Math.toRadians(90));   // TODO: real start
    public static final Pose2d RED_RIGHT_START = new Pose2d(36.0, 12.0, Math.toRadians(90));   // TODO
    public static final Pose2d BLUE_LEFT_START = new Pose2d(12.0, 12.0, Math.toRadians(-90));  // TODO
    public static final Pose2d BLUE_RIGHT_START= new Pose2d(36.0, 12.0, Math.toRadians(-90));  // TODO

    // ---- Scoring landmarks (examples—rename to DECODE objects like GOAL, OBELISK, RAMP) ----
    public static final Vector2d RED_GOAL    = new Vector2d( 96.0, 96.0);  // TODO: real
    public static final Vector2d BLUE_GOAL   = new Vector2d( 96.0, 36.0);  // TODO: real
    public static final Vector2d RAMP_ENTRY  = new Vector2d(120.0, 18.0);  // TODO: real park zone
    public static final Vector2d CYCLE_PICK  = new Vector2d( 24.0, 72.0);  // TODO: where you intake game pieces
}
