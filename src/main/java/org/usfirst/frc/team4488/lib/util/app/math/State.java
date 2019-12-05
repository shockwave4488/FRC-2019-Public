package org.usfirst.frc.team4488.lib.util.app.math;

import org.usfirst.frc.team4488.lib.util.app.CSVWritable;
import org.usfirst.frc.team4488.lib.util.app.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
