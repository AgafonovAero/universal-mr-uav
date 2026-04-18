\# Project instructions



Project: Universal multirotor UAV model for MATLAB/Simulink



\## Current stage

Stage-1 only.

Build a code-centric kernel-first repository.

Do NOT build GUI.

Do NOT build SLX models in this task.

Do NOT build MLAPP apps in this task.

Do NOT create binary-only artifacts as the main source of truth.



\## Main engineering idea

The source of truth must be MATLAB code and text files.

Simulink will be added later as a thin orchestration shell.



\## Domain language

Use Russian flight-dynamics terminology in docs:

\- объект управления

\- математическая модель движения

\- связанная система координат

\- земная система координат

\- ВМГ

\- система автоматического управления

\- контур угловой стабилизации

\- контур стабилизации высоты

\- траекторное управление

\- верификация

\- валидация

\- идентификация

\- летные испытания



Use concise English identifiers in code.



\## Coordinate conventions

\- Earth frame in core: NED

\- Body frame: X forward, Y right, Z down

\- Internal attitude representation: quaternion

\- Euler angles are allowed only for display, debugging, and selected interfaces



\## Units

Use SI only.

Angles in code must be radians.

If degrees are used anywhere, conversion must be explicit.



\## Architectural rules

1\. Keep the implementation text-first.

2\. No .slx, .mlapp, .sldd, .prj creation in this task.

3\. Use MATLAB package folders under /src/+uav/.

4\. Separate code into:

&#x20;  - core

&#x20;  - vmg

&#x20;  - env

&#x20;  - ctrl

&#x20;  - sim

5\. Every nontrivial function must include:

&#x20;  - H1 line

&#x20;  - description

&#x20;  - inputs

&#x20;  - outputs

&#x20;  - units

&#x20;  - assumptions

6\. No hardcoded aircraft values inside generic functions.

7\. Put example parameters into dedicated preset files.

8\. Keep the first implementation minimal and runnable.

9\. Do not fake test execution results.

10\. If MATLAB execution is not available, explicitly say so and provide exact local commands to run.



\## Modeling rules for this task

\- Baseline plant: rigid-body 6DOF

\- Attitude: quaternion-based

\- Propulsion model: simple rotor model T = kT \* omega^2, Q = kQ \* omega^2

\- Mixer: quad-X only for now

\- Control: minimal body-rate PID scaffold only

\- Environment: gravity only for now

\- One basic hover demo

\- Two basic unit tests



\## Done criteria

A task is done only if:

\- requested files are created

\- repository structure is coherent

\- code is readable

\- tests are provided

\- local verification commands are provided

\- assumptions and limitations are written down

