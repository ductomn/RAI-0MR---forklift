# Camera navigated forklift

An application for automatic localisation of forklift and palate with AruCo markers, followed by path-planning and navigation (control) of forklift. Project as part of assignment for courses - Mobile Robotics (0MR) and Artificial Inteligence (RAI).

Videos [here](https://drive.google.com/drive/folders/1I0BrmYF28AGaCJv-hU61jXQ6gOcWJSau?usp=drive_link&fbclid=IwY2xjawSKYypleHRuA2FlbQIxMQBzcnRjBmFwcF9pZAEwAAEeotVoscn4jFc70l5WkeKUeoDFRALwGTEwweNuVPn_y9JHAYtO-aWhbo_8BCA_aem_JSdnNN2rNKQAMNQsLFqX0w)

<img src="img/forklift_front.jpg" width="300">

---

## Structure of the project

### forklift from this link

<https://www.printables.com/model/1058749-3d-printed-rc-forklift-diy>

### drive

- interface for controling the forklift via websocket

### perception

- camera access - OAK-D Lite
- AruCo detection
  - with cv2 function
  - sorted based on ID

- localization (position and angle orientation)
  TODO: add position and angle calculation (formula)

  TODO: conversion to mm based on known size of marker

#### Localization mathematics

##### Orientation angle of the marker

The orientation angle $\theta$ of a marker is calculated using the first two corners $P_0(x_0, y_0)$ and $P_1(x_1, y_1)$ of the detected marker. The `atan2` function is used to handle the correct quadrant mapping in radians. Because the direction (polarity) of Y-axis of the image coordinate system (Y-axis increases downward) and the standard coordinate system (Y-axis increases upward) are opposite, negative sign is added to $(y_1 - y_0)$ to convert to standard coordinate system.

$$
\theta = atan2(-(y_1 - y_0), x_1 - x_0)
$$

##### Center position of the marker

Because the cv2 function for AruCo marker detection returns coordinates of the four corners in correct order, the center position $(X_c, Y_c)$ of a single marker is found simply by taking the arithmetic mean of its four corner coordinates: $P_0, P_1, P_2, P_3$.

$$
X_c = \frac{x_0 + x_1 + x_2 + x_3}{4}
$$

$$
Y_c = \frac{y_0 + y_1 + y_2 + y_3}{4}
$$

##### Pixel to millimeter scale conversion

Since the real-world size of used AruCo marker is known, the ratio of number of pixels to mm can be calculated. The ratio is calculated from the length of the marker edge. Since AruCo markers are square-shaped, hence the length of each edge should be the same, an average length of the edge $L$ is calculated like so:

$$
L_{px} = \frac{d(P_0, P_1) + d(P_1, P_2) + d(P_2, P_3) + d(P_3, P_0)}{4},
$$

where the formula for the length of specific edge is: $d(A, B) = \sqrt{(x_A - x_B)^2 + (y_A - y_B)^2}$.

If multiple markers are detected, the function averages this pixel size across all $n$ markers to find a global average $\bar{L}_{px}$. It then calculates the conversion ratio ($Ratio_{px/mm}$) using the known physical marker size in mm ($L_{mm}$):

$$
Ratio_{px/mm} = \frac{\bar{L}_{px}}{L_{mm}}
$$

Finally, the function transforms the marker's center pixel coordinates into mm. In `get_position_simple_mm`, the Y-axis is also inverted relative to the camera's frame height ($H$) so that the coordinate frame aligns with standard Cartesian mapping (where "up" is positive).

$$
X_{mm} = \frac{X_c}{Ratio_{px/mm}}
$$

$$
Y_{mm} = \frac{H - Y_c}{Ratio_{px/mm}}
$$

The size of state-space is converted to mm in the sane way.

#### Kinematic Motion Model

This module simulates the kinematic movement of the forklift using the standard **Kinematic Bicycle Model**. The path calculation switches automatically between straight-line integration and precise arc integration along a circle based on the steering angle.

##### Geometric Concept

The vehicle steers around an **Instantaneous Center of Rotation (ICR)**. The turning radius $R$ is determined by the wheelbase $L$ and the steering angle $\phi$.

##### Mathematical Equations

###### 1. Angular Velocity & Heading Update

The change in heading angle ($\theta$) is driven by linear velocity ($v$), the vehicle wheelbase ($L$), and the steering wheel angle ($\phi$):

$$\Delta\theta = \frac{v}{L} \cdot \tan(\phi)$$

$$\theta_{new} = \theta + \Delta\theta \cdot dt$$

Note: $\theta_{new}$ is automatically saturated / wrapped to the interval $[-\pi, \pi]$ to prevent numerical overflow.

###### 2. Position Updates

- Case A: Linear Motion (Straight line when $|\phi| < 10^{-5}$)
  When the steering angle is practically zero, the coordinates update using simple linear trigonometry to avoid dividing by zero:

$$x_{new} = x + v \cdot \cos(\theta) \cdot dt$$
$$y_{new} = y + v \cdot \sin(\theta) \cdot dt$$

- Case B: Circular Motion (Turning path when $|\phi| \geq 10^{-5}$)
  When turning, the vehicle moves along an arc. The exact turning radius $R$ is given by:

$$R = \frac{L}{\tan(\phi)}$$

The new position is computed by finding the delta change across the circular arc:

$$x_{new} = x + R \cdot \left(\sin(\theta_{new}) - \sin(\theta)\right)$$
$$y_{new} = y - R \cdot \left(\cos(\theta_{new}) - \cos(\theta)\right)$$

### Hybrid A\*

An advanced motion-planning framework tailored for autonomous forklifts. It combines discrete graph search with vehicle kinematics to generate smooth, physically drivable trajectories within a bounded state space.

#### Architecture Overview

The system is split into three core components:

1. **Motion Model (`ForkSim`)**: Simulates a bicycle steering geometry with rear-wheel/back-wheel characteristics.
2. **Path Engine (`MainPathPlaning`)**: Core orchestrator handling search loops, priority queues, path reconstruction, and error thresholding.
3. **Hybrid A\* Search Core (`AstarHybrid`)**: Implements grid-discretized search tracking non-holonomic constraints.

---

#### Hybrid A\* Search Logic

The algorithm uses a customized formulation designed to respect vehicle maneuvering constraints:

##### Primitive Action Space

The path planner relies on an explicit matrix of discrete operational primitives allowing multi-directional steering sweeps combined with fast-forwarding and low-speed reverse shunts:

- Forward Sweeps ($\approx 1.5v \to 2v$) combined with aggressive cornering angles ($\pm 30^\circ$, $\pm 22.5^\circ$, or straight $0^\circ$)
- Shifting/Reversing Maneuvers ($-v$) matched with sweeping exit angles ($\pm 36^\circ$ or straight $0^\circ$)

##### Cost Formulation ($f(n) = g(n) + h(n)$)

Nodes are sorted within a Min-Heap priority queue (`heapq`) based on a dual-weight optimization metric:

- **Cost-From-Start ($g$):** Accumulates physical distance mapped ($ds$) combined with heading fluctuations ($ts$) plus a strict penalty index from the parent node.
- **Heuristic Cost-To-Goal ($h$):** Computes remaining Euclidean separation ($d$) and weights orientation error ($t$) by a factor of 10 to guarantee a smooth vehicle landing layout.

---

#### Core Execution Pipelines

##### 1. Perception & Coordinate Mapping

- **ArUco Telemetry:** The system expects a minimum of two active markers inside the frame workspace:
  - `corners[0]`: The forklift's real-time position/pose marker.
  - `corners[1]`: The designated target/pallet location marker.
- **Pixel-to-Millimeter Normalization:** Raw image pixel dimensions are dynamically scaled using the known physical marker footprint size ($45\text{ mm}$).
- **State Inversion:** Vertical pixel coordinates are mathematically inverted ($y_{px} = \text{height} - y_{mm}$) to translate the top-left camera origin into a standard ground-plane coordinate map.

##### 2. Error Evaluation & Closed-Loop Feedback

At every time step ($dt$), the runtime engine samples the distance deviation between the current position ($x, y, \theta$) and the expected step coordinates inside the planned array trajectory:

$$\text{Error}_{\text{Pos}} = \sqrt{(x_{real} - x_{plan})^2 + (y_{real} - y_{plan})^2}$$

$$\text{Error}_{\text{Theta}} = |(\theta_{real} - \theta_{plan} + \pi) \pmod{2\pi} - \pi|$$

- **Dynamic Re-planning:** If $\text{Error}_{\text{Pos}} \ge 2\epsilon$ or $\text{Error}_{\text{Theta}} \ge \frac{\epsilon}{5}$, the controller identifies an unacceptable drift, instantly halts the drivetrain, clears out old nodes, and commands a localized state re-plan.

##### 3. Kinematic Translation & Actuation Scaling

When tracking within acceptable error boundaries, raw floating-point actions (`v`, `steer`) are converted to match the hardware's scaling constraints before transmission over the client socket:

- **Steering Command Scaling:**
  $$\text{Payload}_{\text{Steer}} = \lfloor \text{deg}(\phi) \cdot 1.12 \rfloor + 100$$
- **Throttle Velocity Scaling:**
  $$\text{Payload}_{\text{Throttle}} = \lfloor \frac{v}{0.617} \rfloor$$

##### 4. Sequence Termination (Automated Cargo Handling)

Upon successful arrival within the goal tolerance zone ($\text{Distance} < 2\epsilon$), the software stops the trajectory loop and initiates a hardcoded manipulation sequence via `pickUpSeq()`:

##### 1. Unique State-Key Quantization

To prevent exponential node explosion while searching in continuous coordinates ($x, y, \theta$), the environment quantizes the continuous environment space dynamically:

- Position coordinates ($x, y$) are rounded to the nearest $10\text{ mm}$ interval.
- Absolute heading ($\theta$) is rounded to $0.1\text{ rad}$.
  This maps continuous vehicle physics into discrete 3D spatial grids safely checked inside the `closed` lookup cache.

##### 2. Runtime Safety Guardrails

- **Search Expansion Limit:** If open set expansions surpass $100,000$ iterations, the planner truncates calculations and returns the best-known local branch path up to that moment.
- **Dynamic Re-planning Triggers:** The `error()` monitor continuously checks the vehicle's real position relative to the planned path index. If the cross-track error or orientation drift steps outside safe tolerance bounds ($\epsilon$), a path recalculation sequence triggers immediately.

### UI Operator Conclusion & Telemetry Visualization

The graphical user interface (GUI) serves as the primary evaluation and telemetry monitor for the path-planning engine. It bridges real-time computer vision outputs with the inner state of the Hybrid A\* algorithm, providing immediate visual feedback on the system's performance.

#### Real-Time Path Rendering

When path visualization is toggled active, the UI dynamically projects the planned trajectory array directly onto the camera's live video stream:

- **Waypoints:** Individual node states $(x, y)$ calculated by the planner are drawn as distinct **red circles**.
- **Heading Vectors:** The absolute orientation ($\theta$) at each waypoint is represented by a **blue directional arrow**, confirming the calculated steering alignment.
- **Trajectory Links:** Solid **green lines** connect consecutive waypoints, tracing the exact continuous geometric arc generated by the kinematic motion model.

#### Status Notifications & System Guards

- **Active Status:** A green `"Path Planning Active"` indicator confirms that the closed-loop tracking thread is running safely within bounds.
- **Closed-Loop Feedback:** If the physical forklift drifts and exceeds the cross-track or heading tolerance thresholds ($\text{Error}_{\text{Pos}} \ge 2\epsilon$), the user can visually track the instantaneous path clearing and localized re-planning sequence.
- **Control Buttons:** The UI provides quick-action toggle slots (`Go`, `Override`, and `Show Path`) allowing an operator to seamlessly safely halt execution.

---

TODO:

- path planning algorithms ( Dijkstra, RRT)
  TODO: explanation of each algorithms
- mby explain threading?
- how the program runs (sequence and iteration - each step of program)
