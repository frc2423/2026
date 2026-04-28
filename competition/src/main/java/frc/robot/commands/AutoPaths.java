package frc.robot.commands;

import java.util.ArrayList;

import javax.lang.model.element.Element;

import frc.robot.NTHelper;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.FloppingUtil;

public class AutoPaths {
    private static final Path trenchToCenterPath = new Path("Trench-to-Center");
    private static final Path centerToTrenchPath = new Path("Center-to-Trench");
    private static final Path trenchToLeftoversPath = new Path("Trench-to-Leftovers");
    private static final Path leftoversToTrenchPath = new Path("Leftovers-to-Trench");
    private static final Path centerToBumpPath = new Path("Center-to-Bump");
    private static final Path bumpToLeftoversPath = new Path("Bump-to-Leftovers");
    private static final Path centerToLeftoversPath = new Path("Center-to-Leftovers");
    private static final Path bumpToOutpostPath = new Path("Bump-to-Outpost");
    private static final Path bumpToDepotPath = new Path("Bump-to-Depot");
    private static final Path trenchToOutpostPath = new Path("Trench-to-Outpost");
    private static final Path outpostToDepotPath = new Path("Outpost-to-Depot");
    private static final Path depotToDepotPath = new Path("Depot-to-Outpost");
    private static final Path shootToTrenchPath = new Path("Shoot-to-Trench");
    private static final Path trenchToCollectPath = new Path("Trench-to-Collect");

    public static Path getTrenchToCenterPath() {
        return trenchToCenterPath.copy();
    }

    public static Path getCenterToTrenchPath() {
        return centerToTrenchPath.copy();
    }

    public static Path getTrenchToLeftoversPath() {
        return trenchToLeftoversPath.copy();
    }

    public static Path getLeftoversToTrenchPath() {
        return leftoversToTrenchPath.copy();
    }

    public static Path getCenterToBumpPath() {
        return centerToBumpPath.copy();
    }

    public static Path getBumpToLeftoversPath() {
        return bumpToLeftoversPath.copy();
    }

    public static Path getCenterToLeftoversPath() {
        return centerToLeftoversPath.copy();
    }

    public static Path getBumpToOutpostPath() {
        return bumpToOutpostPath.copy();
    }

    public static Path getBumpToDepotPath() {
        return bumpToDepotPath.copy();
    }

    public static Path getTrenchToOutpostPath() {
        return trenchToOutpostPath.copy();
    }

    public static Path getOutpostToDepotPath() {
        return outpostToDepotPath.copy();
    }

    public static Path getDepotToDepotPath() {
        return depotToDepotPath.copy();
    }

    public static Path getShootToTrenchPath() {
        return shootToTrenchPath.copy();
    }

    public static Path getTrenchToCollectPath() {
        return trenchToCollectPath.copy();
    }

    public static double[] getDoubleArrayFromPath(Path path) {
        // if (PoseTransformUtils.isRedAlliance()) {
        //     path.flip();
        // }
        // if (!NTHelper.getString("OutpostOrDepotSide", "outpost").equals("oupost")) {
        //     FloppingUtil.flopPath(path);
        // }
        ArrayList<Double> poses = new ArrayList<Double>();
        for (int i = 0; i < path.getPathElements().size(); i++) {
            if (path.getPathElements().get(i).getClass() == Waypoint.class) {
                poses.add(((Waypoint) path.getPathElements().get(i)).translationTarget().translation().getX());
                poses.add(((Waypoint) path.getPathElements().get(i)).translationTarget().translation().getY());
                poses.add(((Waypoint) path.getPathElements().get(i)).rotationTarget().rotation().getDegrees());
            } else if (path.getPathElements().get(i).getClass() == TranslationTarget.class) {
                poses.add(((TranslationTarget) path.getPathElements().get(i)).translation().getX());
                poses.add(((TranslationTarget) path.getPathElements().get(i)).translation().getY());
                poses.add(0.0);
            }
        }
        double[] posesArray = new double[poses.size()];
        for (int j = 0; j < poses.size(); j++) {
            posesArray[j] = poses.get(j);
        }
        return posesArray;
    }
}
