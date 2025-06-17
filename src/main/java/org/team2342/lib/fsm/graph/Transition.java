package org.team2342.lib.fsm.graph;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;

public class Transition<E extends Enum<E>> {

  @Getter private final E startState, endState;
  private final Command command;

  public Transition(E startState, E endState, Command command) {
    this.startState = startState;
    this.endState = endState;
    this.command = command;
  }

  public String toString() {
    return command.getName() + ": " + startState.name() + " -> " + endState.name();
  }

  public void execute() {
    command.schedule();
  }

  public void cancel() {
    command.cancel();
  }

  public boolean isFinished() {
    return command.isFinished();
  }

  public boolean hasStarted() {
    return command.isScheduled() || command.isFinished();
  }
}
