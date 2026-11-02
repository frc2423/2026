package frc.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.lang.model.element.Element;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.NTHelper;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.FloppingUtil;

public class AutoPaths {
    private static final Map<String, Path> pathCache = new HashMap<>();

    public static Path getPath(String pathFileName) {
        return getPath("autos", pathFileName);
    }

    public static Path getPath(String autosDir, String pathFileName) {
        String key = autosDir + "/" + pathFileName;
        Path path = pathCache.computeIfAbsent(key,
                k -> new Path(new File(Filesystem.getDeployDirectory(), autosDir), pathFileName));
        return path.copy();
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
