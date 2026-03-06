// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static BaseStatusSignal[] registeredSignals = new BaseStatusSignal[0];

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(BaseStatusSignal... signals) {
    BaseStatusSignal[] newSignals = new BaseStatusSignal[registeredSignals.length + signals.length];
    System.arraycopy(registeredSignals, 0, newSignals, 0, registeredSignals.length);
    System.arraycopy(signals, 0, newSignals, registeredSignals.length, signals.length);
    registeredSignals = newSignals;
  }

  /** Refresh all registered signals once per main robot loop. */
  public static void refreshAll() {
    if (registeredSignals.length > 0) {
      BaseStatusSignal.refreshAll(registeredSignals);
    }
  }
}
