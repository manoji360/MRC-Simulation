# Inherited-Codebase Audit

## 1. Project Background

The Mars Rover Simulation team inherited a Unity project developed by the previous year's team. Our preferred approach is to repair and reuse this existing project rather than rebuild the simulation from scratch.

However, the inherited Unity project is currently unusable, and the team must investigate the technical issues preventing it from running.

We have identified three possible development approaches:

1. **Repair the existing Unity project (preferred):** Investigate and resolve the issues preventing the inherited project from running.

2. **Create a new Unity project:** If repairing the inherited project is not feasible within our timeline, rebuild using Unity.

3. **Switch to Gazebo:** If Unity presents technical limitations that cannot be resolved, consider Gazebo as an alternative.

The team will prioritize evaluating the inherited codebase before committing to a replacement implementation.

## 2. Current Codebase Status

We have not yet attempted to open, build, or execute the project in our current development environment. 

The inherited project has been reported as unusable, but the specific technical issues have not yet been independently verified.

Our next step is to open the project in Unity, identify any compatibility issues or missing dependencies, and determine whether the existing implementation can be repaired and reused.

Current Status: Not yet tested.

Next Action: Attempt to open and run the inherited Unity project and document any errors encountered.

## 3. Tests and CI

We has not yet evaluated the existing tests or Continuous Integration (CI) configuration of the inherited Unity project.

We have not confirmed whether the previous project includes automated tests or whether its original build and testing procedures still work.

Our current MRC-Simulation repository has a GitHub Actions CI workflow that successfully runs repository-level checks. However, these checks do not currently validate the inherited Unity simulation.

Inherited Project Test Status: Not yet verified.

Current Repository CI Status: Passing repository-level checks.

Next Action: Inspect the inherited Unity project's test configuration, identify any existing automated tests, and determine whether the project can be built and tested in our current development environment.

## 4. Dependencies

The inherited Unity project's exact Unity Editor version, installed packages, and external dependencies have not yet been verified.

We plans to inspect the project's configuration files to identify the original Unity version and required packages. We will also investigate potential compatibility issues between the inherited project and our current development environment.

Current Status: Dependencies not yet verified.

Next Actions:

1. **Check ProjectSettings to identify the original Unity Editor version.

2. **Review Packages to identify required Unity packages.

3. **Check whether any packages, plugins, or assets are missing or outdated.

4. **Determine whether the inherited project can run using its original Unity version or requires updates.

## 5. Technical Debt and Risks

The inherited Unity project has not yet been fully evaluated, so we cannot confirm the extent of its technical debt.

The main known concern is that the previous Unity project has been reported as unusable. The underlying cause has not yet been independently verified.

Potential technical risks include:

1. **Unity Version Compatibility: The inherited project may depend on an older Unity Editor version that is incompatible with the team's current development environment.

2. **Outdated or Missing Dependencies: Some packages, plugins, or assets may be outdated, unavailable, or incompatible with newer Unity versions.

3. **Legacy Code Issues: Existing scripts or project configurations may require modification before the simulation can run.

4. **Legacy Code Issues: Existing scripts or project configurations may require modification before the simulation can run.

5. **Project Timeline: Repairing the inherited project could take longer than expected and delay implementation.

Risk Mitigation

We will first investigate whether the inherited Unity project can be repaired. If repairing it is not feasible within the project timeline, the team will create a new Unity project.

If Unity cannot meet the project's technical requirements, Gazebo will be considered as an alternative.

Current Status: Technical debt has not yet been fully assessed.

Next Action: Inspect the inherited project, document confirmed issues, and estimate the effort required to repair it.
