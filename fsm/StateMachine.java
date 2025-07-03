// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.lib.fsm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.HashMap;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.fsm.graph.Transition;
import org.team2342.lib.fsm.graph.TransitionGraph;

public class StateMachine<E extends Enum<E>> {
  private final TransitionGraph<E> transitions;
  @Getter private E currentState;
  private final HashMap<E, Command> stateCommands;

  private final Supplier<E> determiner;
  private final E undeterminedState;

  @Getter private final String name;
  private final Class<E> enumType;

  @Getter private boolean enabled;

  private Command currentCommand;
  @Getter private Transition<E> currentTransition;

  @Getter private E targetState;
  @Getter private boolean transitioning;

  /**
   * Create a new StateMachine
   *
   * @param name The name of the state machine (for logging purposes)
   * @param undeterminedState The undetermined state of the system
   * @param determiner A supplier to determine a state from the undetermined state
   * @param enumType The enum to derive states from
   */
  public StateMachine(String name, E undeterminedState, Supplier<E> determiner, Class<E> enumType) {
    this.name = name;
    this.undeterminedState = undeterminedState;
    this.determiner = determiner;

    currentState = undeterminedState;
    stateCommands = new HashMap<>();

    this.enumType = enumType;

    transitions = new TransitionGraph<>(enumType);

    enabled = false;
  }

  /**
   * Enable the state machine. Commands and transitions will only run when the machine is enabled.
   *
   * <p>Note that running enable will cause the state machine to use the determiner to find its
   * state
   */
  public void enable() {
    if (isEnabled()) return;

    currentState = determiner.get();
    targetState = currentState;
    enabled = true;
  }

  /**
   * Disable the state machine. Commands and transitions will only run when the machine is enabled.
   *
   * <p>Note that running disable will cause the state to become undetermined
   */
  public void disable() {
    if (!isEnabled()) return;

    currentCommand.cancel();
    currentCommand = null;

    currentTransition.cancel();
    currentTransition = null;

    transitioning = false;

    currentState = undeterminedState;
    targetState = null;

    enabled = false;
  }

  /**
   * Add a transition to the machine
   *
   * @param start The start state
   * @param end The end state
   * @param transitionCommand The command to run during the transition
   */
  public void addTransition(E start, E end, Command transitionCommand) {
    transitions.addTransition(new Transition<>(start, end, transitionCommand));
  }

  /**
   * Add a transition to the machine
   *
   * @param start The start state
   * @param end The end state
   */
  public void addTransition(E start, E end) {
    addTransition(start, end, Commands.none());
  }

  /**
   * Remove a transition from the machine
   *
   * @param start The start state
   * @param end The end state
   */
  public void removeTransition(E start, E end) {
    transitions.removeTransition(start, end);
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start The start state
   * @param end The end state
   * @param transitionCommand The command to run during the transition
   */
  public void addDualTransition(E start, E end, Command transitionCommand) {
    transitions.addTransition(new Transition<>(start, end, transitionCommand));
    transitions.addTransition(new Transition<>(end, start, transitionCommand));
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start The start state
   * @param end The end state
   */
  public void addDualTransition(E start, E end) {
    addDualTransition(start, end, Commands.none());
  }

  /**
   * Add a transition from every state to the given state
   *
   * @param state The state to transition to
   * @param transitionCommand The command to run during the transition
   */
  public void addOmniTransition(E state, Command transitionCommand) {
    for (E s : enumType.getEnumConstants()) {
      if (s != state) {
        addTransition(s, state, transitionCommand);
      }
    }
  }

  /**
   * Add a transition from every state to the given state
   *
   * @param state The state to transition to
   */
  public void addOmniTransition(E state) {
    addOmniTransition(state, Commands.none());
  }

  /**
   * Register a state command to run when the machine is at the given state
   *
   * @param state The state to run the command at
   * @param stateCommand The command to run
   */
  public void addStateCommand(E state, Command stateCommand) {
    stateCommands.put(state, stateCommand);
  }

  /**
   * Request the machine to transition to a target state
   *
   * @param state The target state
   */
  public void requestTransition(E state) {
    targetState = state;
  }

  /**
   * Request the machine to transition to a target state
   *
   * @param state The target state
   * @return A command to request the transition, that will run until the target is reached
   */
  public Command requestTransitionCommand(E state) {
    return new FunctionalCommand(
        () -> requestTransition(state),
        () -> {},
        (interrupted) -> {},
        () -> getCurrentState() == state);
  }

  /**
   * Wait for a certain state
   *
   * @param state The state to wait for
   * @return A command that will wait until the state is reached
   */
  public Command waitForState(E state) {
    return new WaitUntilCommand(() -> getCurrentState() == state);
  }

  private void cancelStateCommand() {
    if (stateCommands.containsKey(getCurrentState())) {
      Command prevCommand = stateCommands.get(getCurrentState());
      if (prevCommand != null && prevCommand.isScheduled()) prevCommand.cancel();
      if (currentCommand != null && currentCommand.isScheduled()) currentCommand.cancel();
      currentCommand = null;
    }
  }

  private void setState(E state) {
    currentState = state;
    currentCommand = stateCommands.get(getCurrentState());
    if (currentCommand != null) currentCommand.schedule();
  }

  /**
   * DANGER: Do not use unless you understand the consquences
   *
   * <p>Forcibly sets the FSM's state to the current state
   */
  public void forceState(E state) {
    setState(state);
  }

  /** Run the machine */
  public void periodic() {
    if (isEnabled()) {
      // Update transitions
      if (isTransitioning() && currentTransition.isFinished()) {
        setState(currentTransition.getEndState());
        currentTransition = null;
        transitioning = false;
      }

      // Check to see if we need to transition to a new state
      if (targetState != currentState && !isTransitioning()) {
        Transition<E> transition = transitions.getNextTransition(currentState, targetState);
        if (transition != currentTransition) {
          transitioning = true;
          currentTransition = transition;
          cancelStateCommand();
          currentTransition.execute();
        }
      }
    }

    // Logging statements
    Logger.recordOutput(name + "/FSM/Enabled", isEnabled());
    Logger.recordOutput(name + "/FSM/TargetState", targetState.name());
    Logger.recordOutput(
        name + "/FSM/DesiredState",
        isTransitioning() ? getCurrentTransition().getEndState().name() : targetState.name());
    Logger.recordOutput(name + "/FSM/CurrentState", getCurrentState().toString());
    Logger.recordOutput(name + "/FSM/Transitioning", isTransitioning());
  }

  /**
   * Returns a representation of the states and transitions in DOT format for visualization of the
   * state machine
   *
   * <p>Use Graphviz to view the output (websites exist if you don't want install it)
   */
  public String dot() {
    return transitions.dot();
  }
}
