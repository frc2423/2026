package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.HeatmapConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



public class PieceHeatmap {

  private final double cellSize;

  private final int nx;
  private final int ny;

  private final double[][] heat;

  public PieceHeatmap() {

    this.cellSize = Constants.HeatmapConstants.CELL_SIZE_METERS;

    this.nx = (int)Math.ceil(Constants.HeatmapConstants.FIELD_LENGTH_METERS / cellSize);
    this.ny = (int)Math.ceil(Constants.HeatmapConstants.FIELD_WIDTH_METERS / cellSize);

    this.heat = new double[nx][ny];
  }

  public void decay() {
    for (int ix = 0; ix < nx; ix++) {
      for (int iy = 0; iy < ny; iy++) {
        heat[ix][iy] *= Constants.HeatmapConstants.DECAY_FACTOR;
      }
    }
  }

  public void addObservation(Translation2d fieldPoint, double weight) {
    int ix = (int)(fieldPoint.getX() / cellSize);
    int iy = (int)(fieldPoint.getY() / cellSize);
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) return;
    heat[ix][iy] += weight;
  }

  public Translation2d bestCellCenter(double minHeat) {
    int bestX = -1, bestY = -1;
    double best = minHeat;

    for (int ix = 0; ix < nx; ix++) {
      for (int iy = 0; iy < ny; iy++) {
        if (heat[ix][iy] > best) {
          best = heat[ix][iy];
          bestX = ix;
          bestY = iy;
        }
      }
    }
    if (bestX < 0) return null;

    return new Translation2d(
        (bestX + 0.5) * cellSize,
        (bestY + 0.5) * cellSize
    );
  }


  
}
