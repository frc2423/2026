package frc.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

public class AutoPaths {
    private static final Map<String, Path> pathCache = new HashMap<>();

    public static Path getPath(String pathFileName) {
        return getPath("autos", pathFileName);
    }

    public static Path getPath(String autosDir, String pathFileName) {
        return loadPath(autosDir, pathFileName).copy();
    }

    public static void loadAllPaths() {
        loadAllPaths("autos");
    }

    // Loads every path in <autosDir>/paths into the cache so later getPath calls don't read files
    public static void loadAllPaths(String autosDir) {
        File pathsDir = new File(new File(Filesystem.getDeployDirectory(), autosDir), "paths");
        File[] pathFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));
        if (pathFiles == null) {
            DriverStation.reportError("Auto paths directory not found: " + pathsDir, false);
            return;
        }
        for (File pathFile : pathFiles) {
            String pathFileName = pathFile.getName().substring(0, pathFile.getName().length() - ".json".length());
            try {
                loadPath(autosDir, pathFileName);
            } catch (RuntimeException e) {
                DriverStation.reportError("Failed to load auto path " + pathFile + ": " + e, e.getStackTrace());
            }
        }
    }

    private static Path loadPath(String autosDir, String pathFileName) {
        String key = autosDir + "/" + pathFileName;
        return pathCache.computeIfAbsent(key,
                k -> new Path(new File(Filesystem.getDeployDirectory(), autosDir), pathFileName));
    }

    public static double[] getDoubleArrayFromPath(Path path) {
        // if (PoseTransformUtils.isRedAlliance()) {
        // path.flip();
        // }
        // if (!NTHelper.getString("OutpostOrDepotSide", "outpost").equals("oupost")) {
        // FloppingUtil.flopPath(path);
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
