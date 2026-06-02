hihihi

# Camera navigated forklift

An application for automatic localisation of forklift and palate with AruCo markers, followed by path-planning and navigation (control) of forklift. Project as part of assignment for courses - Mobile Robotics (0MR) and Artificial Inteligence (RAI). 

Videos [here](https://drive.google.com/drive/folders/1I0BrmYF28AGaCJv-hU61jXQ6gOcWJSau?usp=drive_link&fbclid=IwY2xjawSKYypleHRuA2FlbQIxMQBzcnRjBmFwcF9pZAEwAAEeotVoscn4jFc70l5WkeKUeoDFRALwGTEwweNuVPn_y9JHAYtO-aWhbo_8BCA_aem_JSdnNN2rNKQAMNQsLFqX0w)

<img src="img/forklift_front.jpg" width="300">

---


## Structure of the project

### forklift from this link ...

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



### pathPlanning
- forklift model
    TODO: formula

- path planning algorithms (A*, Dijkstra, RRT)
    TODO: explanation of each algorithms

### ui
- live camera feed with displayed path and control buttons
    TODO: explain buttons

---



TODO:  
- mby explain threading?
- how the program runs (sequence and iteration - each step of program)


