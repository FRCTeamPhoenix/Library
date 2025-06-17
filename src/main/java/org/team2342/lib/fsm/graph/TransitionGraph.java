package org.team2342.lib.fsm.graph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

public class TransitionGraph<E extends Enum<E>> {

  private final Transition<E>[][] adjMap;
  private final Class<E> enumType;

  @SuppressWarnings("unchecked") // lol type "safety"
  public TransitionGraph(Class<E> enumType) {
    int c = enumType.getEnumConstants().length;
    this.enumType = enumType;

    adjMap = (Transition<E>[][]) new Transition[c][c];
  }

  public void addEdge(Transition<E> transition) {
    adjMap[transition.getStartState().ordinal()][transition.getEndState().ordinal()] = transition;
  }

  public Transition<E> getNextEdge(E start, E target) {
    if (adjMap[start.ordinal()][target.ordinal()] != null) {
      return adjMap[start.ordinal()][target.ordinal()];
    }

    return bfs(start, target);
  }

  // "Based" on 6328's implementation of BFS
  public Transition<E> bfs(E start, E target) {
    // Map to track the parent of each visited node
    Map<E, E> parents = new HashMap<>();
    Queue<E> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      E current = queue.poll();
      // Check if we've reached the goal
      if (current.equals(target)) {
        break;
      }
      // Process valid neighbors
      for (Transition<E> edge : adjMap[current.ordinal()]) {
        if (edge == null) continue;
        E neighbor = edge.getEndState();
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    if (!parents.containsKey(start)) {
      return null; // Goal not reachable
    }

    // Trace back the path from goal to start
    E nextState = target;
    while (!nextState.equals(start)) {
      E parent = parents.get(nextState);
      if (parent == null) {
        return null; // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return adjMap[start.ordinal()][nextState.ordinal()];
      }
      nextState = parent;
    }
    return adjMap[start.ordinal()][nextState.ordinal()];
  }

  /**
   * Returns a representation of the states and transitions in DOT format for visualization of the
   * state machine
   *
   * <p>Use Graphviz to view the output (websites exist if you don't want install it)
   */
  public String dot() {
    StringBuilder dot = new StringBuilder();
    dot.append("digraph FSM {\n");
    dot.append("  rankdir=LR;\n");
    dot.append("  node [shape = circle];\n");
    E[] states = enumType.getEnumConstants();
    Set<E> usedStates = new HashSet<>();
    for (int i = 0; i < adjMap.length; i++) {
      for (int j = 0; j < adjMap[i].length; j++) {
        Transition<E> edge = adjMap[i][j];
        if (edge != null) {
          usedStates.add(states[i]);
          usedStates.add(states[j]);
        }
      }
    }
    for (E state : usedStates) {
      String rawName = state.name();
      String[] parts = rawName.split("_");
      for (int k = 0; k < parts.length; k++) {
        if (parts[k].length() > 0) {
          parts[k] = parts[k].substring(0, 1).toUpperCase() + parts[k].substring(1).toLowerCase();
        }
      }
      String label = String.join(" ", parts);
      dot.append(String.format("  %s [label=\"%s\"];\n", rawName, label));
    }
    for (int i = 0; i < adjMap.length; i++) {
      for (int j = 0; j < adjMap[i].length; j++) {
        Transition<E> edge = adjMap[i][j];
        if (edge != null) {
          String from = states[i].name();
          String to = states[j].name();
          dot.append(String.format("  %s -> %s;\n", from, to));
        }
      }
    }
    dot.append("}");
    return dot.toString();
  }
}
