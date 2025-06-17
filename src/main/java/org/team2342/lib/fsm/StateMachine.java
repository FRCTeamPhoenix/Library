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

  public void enable() {
    if (isEnabled()) return;

    currentState = determiner.get();
    targetState = currentState;
    enabled = true;
  }

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

  public void addTransition(E start, E end, Command transitionCommand) {
    transitions.addEdge(new Transition<>(start, end, transitionCommand));
  }

  public void addTransition(E start, E end) {
    addTransition(start, end, Commands.none());
  }

  public void addDualTransition(E start, E end, Command transitionCommand) {
    transitions.addEdge(new Transition<>(start, end, transitionCommand));
    transitions.addEdge(new Transition<>(end, start, transitionCommand));
  }

  public void addDualTransition(E start, E end) {
    addDualTransition(start, end, Commands.none());
  }

  public void addOmniTransition(E state, Command transitionCommand) {
    for (E s : enumType.getEnumConstants()) {
      if (s != state) {
        addTransition(s, state, transitionCommand);
      }
    }
  }

  public void addOmniTransition(E state) {
    addOmniTransition(state, Commands.none());
  }

  public void addStateCommand(E state, Command stateCommand) {
    stateCommands.put(state, stateCommand);
  }

  public void requestTransition(E state) {
    targetState = state;
  }

  public Command requestTransitionCommand(E state) {
    return new FunctionalCommand(
        () -> requestTransition(state),
        () -> {},
        (interrupted) -> {},
        () -> getCurrentState() == state);
  }

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
        Transition<E> transition = transitions.getNextEdge(currentState, targetState);
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
