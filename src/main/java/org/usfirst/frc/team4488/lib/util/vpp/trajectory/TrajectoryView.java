package org.usfirst.frc.team4488.lib.util.vpp.trajectory;

import org.usfirst.frc.team4488.lib.util.app.math.State;

public interface TrajectoryView<S extends State<S>> {
  public TrajectorySamplePoint<S> sample(final double interpolant);

  public double first_interpolant();

  public double last_interpolant();

  public Trajectory<S> trajectory();
}
