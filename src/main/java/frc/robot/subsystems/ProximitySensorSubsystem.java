// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ProximitySensorSubsystem extends SubsystemBase {
  private static final double PRINT_PERIOD = 0.5; // Update every 500 ms
  private final CANBus kCANBus = new CANBus("rio");

  /**
   * We recommend reading the Tuning CANrange devblog in our API documentation
   * to see how to best utilize a CANrange for your system.
   * https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/tuning-canrange.html
   */
  private final CANrange canRange = new CANrange(1, kCANBus);

  private double currentTime = Timer.getFPGATimestamp();


  /** Creates a new ProximitySensorSubsystem. */
  public ProximitySensorSubsystem() {
    
    canRange.getConfigurator().apply(Configs.ProximitySensorSubsystem.proximitySensorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Timer.getFPGATimestamp() - currentTime > PRINT_PERIOD) {
      currentTime += PRINT_PERIOD;

      /**
       * getDistance() and getSignalStrength() automatically call refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.
       */
      var distance = canRange.getDistance();
      var signalStrength = canRange.getSignalStrength();
      System.out.println("Distance is " + distance.toString() + " with a signal strength of " + signalStrength + " and " + distance.getTimestamp().getLatency() + " seconds of latency");

      /**
       * Get the isDetected StatusSignalValue without refreshing
       */
      var isDetected = canRange.getIsDetected(false);
      /* This time wait for the signal to reduce latency */
      isDetected.waitForUpdate(PRINT_PERIOD); // Wait up to our period
      /**
       * This uses the explicit getValue and getUnits functions to print, even though it's not
       * necessary for the ostream print
       */
      System.out.println(
        "Is Detected is " +
        isDetected.getValue() + " " +
        isDetected.getUnits() + " with " +
        isDetected.getTimestamp().getLatency() + " seconds of latency"
      );
      /**
       * Notice when running this example that the second print's latency is always shorter than the first print's latency.
       * This is because we explicitly wait for the signal using the waitForUpdate() method instead of using the refresh()
       * method, which only gets the last cached value (similar to how Phoenix v5 works).
       * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
       * CAN bus measurements.
       * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
       * timestamps when it receives the frame. This can be further used for latency compensation.
       */
      System.out.println();
    }
  }
}
