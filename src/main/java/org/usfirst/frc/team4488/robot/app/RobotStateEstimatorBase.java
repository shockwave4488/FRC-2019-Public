package org.usfirst.frc.team4488.robot.app;

import java.util.Map;
import org.usfirst.frc.team4488.lib.util.app.InterpolatingDouble;
import org.usfirst.frc.team4488.lib.util.app.InterpolatingTreeMap;
import org.usfirst.frc.team4488.lib.util.app.math.RigidTransform2d;
import org.usfirst.frc.team4488.lib.util.app.math.Twist2d;

public abstract class RobotStateEstimatorBase {

  protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> field_to_vehicle_;
  protected Twist2d vehicle_velocity_predicted_;
  protected Twist2d vehicle_velocity_measured_;
  protected double distanceDriven;

  public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
    return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
    return field_to_vehicle_.lastEntry();
  }

  public synchronized RigidTransform2d getPredictedFieldToVehicle(double lookahead_time) {
    return getLatestFieldToVehicle()
        .getValue()
        .transformBy(RigidTransform2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
  }

  public synchronized void addFieldToVehicleObservation(
      double timestamp, RigidTransform2d observation) {
    field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
  }

  /** Resets the field to robot transform (robot's position on the field) */
  public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle) {
    field_to_vehicle_ = new InterpolatingTreeMap<>(100);
    field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    vehicle_velocity_predicted_ = Twist2d.identity();
    vehicle_velocity_measured_ = Twist2d.identity();
    distanceDriven = 0.0;
  }

  public double getDistanceDriven() {
    return distanceDriven;
  }

  public synchronized Twist2d getPredictedVelocity() {
    return vehicle_velocity_predicted_;
  }

  public synchronized Twist2d getMeasuredVelocity() {
    return vehicle_velocity_measured_;
  }

  public synchronized void resetDistanceDriven() {
    distanceDriven = 0.0;
  }

  public abstract void start();

  public abstract void addObservations(
      double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity);

  public abstract Twist2d generateOdometryFromSensors();

  public abstract Twist2d generatePredictedVelocity();
}
