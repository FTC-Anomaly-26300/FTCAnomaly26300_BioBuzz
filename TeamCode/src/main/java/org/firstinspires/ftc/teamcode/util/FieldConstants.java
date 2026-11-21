package org.firstinspires.ftc.teamcode.util;

// Positions on the field, all in inches and degrees, in Pedro Pathing's field coordinates.
//
// (0, 0) is the corner at tile A1, on the audience side by the red alliance.
// (144, 144) is the corner at tile F6. +X points from the red alliance wall (X = 0) to the
// blue alliance wall (X = 144). +Y points from the audience wall (Y = 0) to the back wall.
// Heading 0 points along +X, and turning left (counterclockwise) makes it go up, the same as
// the Pinpoint. So 90 is +Y, 180 is -X, and 270 is -Y.
//
// The field is the same when turned 180 degrees around its center, so every red position
// matches a blue one at (144 - x, 144 - y).
public class FieldConstants {

    // ===== Robot size =====

    // The robot's outside size in inches, bumper to bumper, measured on the robot
    public static final double ROBOT_LENGTH = 17.0; // front to back
    public static final double ROBOT_WIDTH = 17.5;  // left to right
    public static final double HALF_LENGTH = ROBOT_LENGTH / 2.0;
    public static final double HALF_WIDTH = ROBOT_WIDTH / 2.0;

    // ===== HIVE CELLs =====

    // Each alliance has its own HIVE on a frame in the center of the field, at (72, 72).
    // A HIVE is a seesaw with two CELLs, one on the audience side and one on the back side.
    // You score by launching into the CELL facing up, and a TIP flips it so the other CELL faces up.
    // At the start, red's audience CELL and blue's back CELL face up (Event Field Setup Guide).
    // Each CELL's opening faces outward, toward the wall on its side, so the shooter aims at
    // whichever of our CELLs opens toward the half of the field the robot is on.
    // Never aim at the other alliance's HIVE, launching at it is a foul (G417).
    //
    // AprilTags on the bottom face of each CELL, facing down at the tiles (manual 9.9).
    // On SDK 12 a camera sees them as one cluster per CELL, and the cluster's position points at
    // the center of that CELL's opening. If its roll is between -90 and 90, that CELL is facing up
    // (FTC Docs AprilTag Clusters tech tip).
    //   red audience CELL, cluster "RED AUDIENCE"    34, 35, 36, 37
    //   red back CELL, cluster "RED SCORING"         30, 31, 32, 33
    //   blue audience CELL                           38, 39, 40, 41
    //   blue back CELL                               42, 43, 44, 45

    // How far each CELL center is from the middle of the field along Y. Measured from the
    // manual's tile figure (Figure 9-4), where each HIVE's two CELLs are about 26 to 27 in apart
    // center to center. The figure also shows each pair shifted 2 to 3 in toward one wall,
    // probably because the seesaw is tilted, so this uses the average.
    // Height isn't needed. The speed table is tuned by shooting into the real CELL, so the
    // height is already part of those numbers.
    private static final double CELL_Y_OFFSET = 13.2; // CELLs at Y = 58.8 and 85.2

    // How far each HIVE is from the middle of the field along X, red on the red side.
    // Measured from the same figure, the HIVE centers are at X = 59.0 and 84.7, so each is
    // about 12.75 in from the middle. Red's HIVE is on tile C and blue's on tile D.
    private static final double CELL_X_OFFSET = 12.75;

    public static final double RED_AUDIENCE_CELL_X = 72.0 - CELL_X_OFFSET;  // up at the start
    public static final double RED_AUDIENCE_CELL_Y = 72.0 - CELL_Y_OFFSET;
    public static final double RED_BACK_CELL_X = 72.0 - CELL_X_OFFSET;
    public static final double RED_BACK_CELL_Y = 72.0 + CELL_Y_OFFSET;
    public static final double BLUE_AUDIENCE_CELL_X = 72.0 + CELL_X_OFFSET;
    public static final double BLUE_AUDIENCE_CELL_Y = 72.0 - CELL_Y_OFFSET;
    public static final double BLUE_BACK_CELL_X = 72.0 + CELL_X_OFFSET;     // up at the start
    public static final double BLUE_BACK_CELL_Y = 72.0 + CELL_Y_OFFSET;

    // ===== LOADING ZONES =====

    // About 23 in wide and 11 in deep, against each alliance's own wall (manual 9.3), on tile A5
    // for red and F2 for blue (Event Field Setup Guide). These are the centers.
    // Ending auto at least partly inside it is PARK, worth 5 points.
    public static final double RED_LOADING_ZONE_X = 5.5;
    public static final double RED_LOADING_ZONE_Y = 108.0;
    public static final double BLUE_LOADING_ZONE_X = 144.0 - 5.5;
    public static final double BLUE_LOADING_ZONE_Y = 144.0 - 108.0;

    // ===== FLOWERS =====

    // One FLOWER on each wall, where two perimeter wall pieces join (Setup Guide step 10).
    // The manual's hint (10.3.1) is that the CELL pointing at a FLOWER starts tilted down.
    // Red's back CELL starts down, so the back wall FLOWER is on the red half, and blue's
    // audience CELL starts down, so the audience wall FLOWER is on the blue half.
    // TODO the exact spots are only in the setup guide's figures, so check on the real field.
    public static final double RED_WALL_FLOWER_X = 0.0;
    public static final double RED_WALL_FLOWER_Y = 48.0;       // where tiles A2 and A3 meet
    public static final double BACK_WALL_FLOWER_X = 48.0;      // where tiles B6 and C6 meet
    public static final double BACK_WALL_FLOWER_Y = 144.0;
    public static final double AUDIENCE_WALL_FLOWER_X = 96.0;  // where tiles D1 and E1 meet
    public static final double AUDIENCE_WALL_FLOWER_Y = 0.0;
    public static final double BLUE_WALL_FLOWER_X = 144.0;
    public static final double BLUE_WALL_FLOWER_Y = 96.0;      // where tiles F4 and F5 meet

    // ===== Drivers =====

    // The field heading that points straight away from each alliance's drivers.
    // Red drivers stand behind X = 0 facing +X, and blue drivers stand behind X = 144 facing -X.
    public static final double BLUE_DRIVER_FORWARD_HEADING = 180.0;
    public static final double RED_DRIVER_FORWARD_HEADING = 0.0;

    // ===== Relocalize corner =====

    // Drive into this corner and press BACK to reset the position and heading completely.
    // It's the corner on the drive team's left, with the robot's back flat against your
    // alliance wall and its left side flat against the side wall. It's the corner without the
    // GARDEN, and clear of the LOADING ZONE and FLOWERS. If auto didn't run, start TeleOp here.
    public static final double RED_CORNER_X = HALF_LENGTH;
    public static final double RED_CORNER_Y = 144.0 - HALF_WIDTH;
    public static final double RED_CORNER_HEADING = 0.0;
    public static final double BLUE_CORNER_X = 144.0 - HALF_LENGTH;
    public static final double BLUE_CORNER_Y = HALF_WIDTH;
    public static final double BLUE_CORNER_HEADING = 180.0;

    // ===== Starting spots for the autos =====

    // Robots have to start touching the field wall, fully on their own side, not in the
    // LOADING ZONE, and not in a FLOWER (G304). Each spot has the robot's back flat against
    // the wall, facing into the field, so driving forward moves it off the wall for LEAVE.
    // Park starts against your alliance wall, next to the LOADING ZONE, out of the way of a
    // partner robot. Front starts against the audience wall (red) or back wall (blue), right in
    // front of the CELL that's up at the start, so it can shoot right away. G304 allows any
    // wall as long as the robot is fully on its own half.
    // Every list has one entry per spot, in the same order. Each auto picks its spot by its
    // position in these lists (Blue Park is 0, Red Front is 3), so don't reorder them.
    public static final String[] START_NAMES = {
            "Blue Park (next to the LOADING ZONE)",
            "Blue Front (in front of the CELL)",
            "Red Park (next to the LOADING ZONE)",
            "Red Front (in front of the CELL)"
    };

    public static final boolean[] START_IS_RED = {false, false, true, true};

    public static final double[] START_X = {
            144.0 - HALF_LENGTH,  // Blue Park, against the blue wall
            BLUE_BACK_CELL_X,     // Blue Front, lined up with blue's back CELL
            HALF_LENGTH,          // Red Park, against the red wall
            RED_AUDIENCE_CELL_X   // Red Front, lined up with red's audience CELL
    };

    public static final double[] START_Y = {
            60.0,                 // Blue Park, middle of tile F3, next to the F2 LOADING ZONE
            144.0 - HALF_LENGTH,  // Blue Front, against the back wall
            84.0,                 // Red Park, middle of tile A4, next to the A5 LOADING ZONE
            HALF_LENGTH           // Red Front, against the audience wall
    };

    public static final double[] START_HEADING = {180.0, 270.0, 0.0, 90.0};
}
