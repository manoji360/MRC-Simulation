# Design

## Context and scope

This repository is intentionally a startup-phase planning repository. It captures the project direction for a Unity-first Mars rover simulation and defines a lightweight, reviewable structure before production implementation begins.

The goal of this checkpoint is to create a clear foundation for future work. The team is not yet building a simulator runtime, and the repo should not imply that the system is more mature than it is.

## Planned architecture

The early architecture is intentionally simple and conceptual. The project is expected to evolve toward a Unity-based simulation environment that supports rover behavior, camera or interaction loops, and visual validation of the prototype.

```text
+---------------------------+
| Repository / team docs    |
| README + requirements +   |
| design + risk register    |
+-------------+-------------+
              |
              v
+---------------------------+
| Planning and milestone    |
| coordination              |
+-------------+-------------+
              |
              v
+---------------------------+
| Unity-first prototype     |
| scene + core gameplay     |
| scripts                   |
+-------------+-------------+
              |
              v
+---------------------------+
| Future simulation and     |
| validation work           |
+---------------------------+
```

## Key design choices

### Unity-first direction

The project is currently planned around Unity rather than Gazebo. This decision is intentional and reflects the design goal of a visual, interactive prototype that is easier to iterate on during the startup phase.

### Documentation as the initial deliverable

The repository currently functions as a planning and documentation hub. The living docs are the primary product of this checkpoint because the actual simulator has not yet started.

### Minimal structure

The repo intentionally keeps a lightweight structure so the team can add implementation work later without needing to untangle an overly complex start state.

## Data and interface expectations

This checkpoint does not define a production API or a finalized simulation contract. Instead, the repo establishes a clear review pipeline and planning foundation for future work.

Potential future interfaces may include:

- Unity scene objects and scripts
- rover state values and commands
- user interaction or camera controls
- gameplay loop and prototype telemetry

These are not yet final, and no product-level API should be claimed in this phase.

## Deployment and runtime target

We plans to use Unity as the primary simulation environment. By Week 10, the goal is to develop a minimal working rover prototype that can run locally on a designated computer for demonstration.
The prototype will focus on launching a Unity simulation scene and demonstrating basic rover functionality. Cloud hosting and public web deployment are not planned for this milestone.
We also plans to test the simulation locally before presenting it. The specific demonstration computer, infrastructure owner, and distribution method have not yet been finalized.
Production Target: Local Unity simulation demonstration on a designated computer.
Target Date: Week 10.
Infrastructure Owner: To be assigned.
Mentor Approval: Pending discussion and confirmation.
Approval Follow-up: The Project Manager will discuss the proposed production target with the mentor by November 15, 2026.

## Decisions intentionally deferred

- Final simulation gameplay loop
- Rover control scheme and algorithms
- Visual asset and world design
- Production deployment strategy
- Full testing strategy for runtime behavior

## Open technical questions

- What is the minimum interactive rover prototype that demonstrates the required behavior?
- Which Unity scene or level will be used to validate the prototype?
- Which rover interactions and telemetry are essential in the first milestone?
- What team and tooling choices are required before a production implementation begins?
