package org.team2342.lib.fsm.graph;

import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;

public class Transition<E extends Enum<E>> {

  @Getter private final E startState, endState;
  private final Command command;

  /**
   * Creates a new transition between two states
   *
   * @param startState The start state of the transition
   * @param endState The end state of the transition
   * @param command The command to run to transition
   */
  public Transition(E startState, E endState, Command command) {
    this.startState = startState;
    this.endState = endState;
    this.command = command;
  }

  /** Returns a string representation of the transition */
  public String toString() {
    return command.getName() + ": " + startState.name() + " -> " + endState.name();
  }

  /** Run the transition command */
  public void execute() {
    command.schedule();
  }

  /** Cancel the transition command */
  public void cancel() {
    command.cancel();
  }

  /**
   * Whether the transition has finished or not.
   *
   * @return Whether the transition command has finished
   */
  public boolean isFinished() {
    return command.isFinished();
  }

  /**
   * Whether the transition has been started or not.
   *
   * @return Whether the transition command has started
   */
  public boolean hasStarted() {
    return command.isScheduled() || command.isFinished();
  }
}
