# Obstacle Avoidance with Gaussian Potential Field Method

## Algorithm

The main idea behind this method is to

1. Receive distance data from the range sensor(s).
2. Consider only the objects that are within the threshold range.
3. Enlarge the obstacles with regard to the vehicle’s width.
4. Construct a Gaussian (repulsive) potential field from them.
5. Calculate the attractive field from the yaw angle information from an inertial measurement unit (IMU).
6. The total field is made of these two fields. Choose the angle with the minimum total field value.

Work on the algorithm can be found [here](petcat-obstacle-avoidance-).

The algorithm has been tested on a husky, the node for which can be found [here](random_explorer.cpp).

## Modules

### ODG-PF.h

```cpp
#include "ODG-PF.h"
```

#### Global Constants

```cpp
const float Gamma    = 0.59
const float LC       = 1
const float MAX_DIST = 3.5
```

#### Functions

``` cpp
float get_goal_angle(float x, float y, float goal_x, float goal_y)
void goal_field(vector<float> &field, float goal_angle)
float index_to_angle(int i)
float angle_to_index(float a)
```

#### Description

1. **get_goal_angle()**

```cpp
float get_goal_angle( float x,
                      float y,
                      float goal_x,
                      float goal_y )
```

Returns the goal angle in radians given the ego and goal positions

*Parameters*:

* **x** - Ego x coordinate
* **y** - Ego y coordinate
* **goal_x** - Goal’s x coordinate
* **goal_y** - Goal’s y coordinate

2. **goal_field()**

```cpp
void goal_field( vector<float> &field,
                 float goal_angle )
```

Adds the attractive goal field to the potential field

*Parameters*:

* **field** - The potential field
* **goal_angle** - Goal angle in radians

3. **index_to_angle()**

```cpp
float index_to_angle(int i)
```

Returns the angle in radians for index **i**

4. **angle_to_index()**

```cpp
float angle_to_index(float a)
```

Returns the index value for angle **a**(radians)

#### Classes

1. **obstacle**

**Private Data Members**

```cpp
float d
float phi
float theta
float A
```

**Public Member Functions**

```cpp
obstacle(float d_in, float phi_in,float theta_in)
void  increase_width(float w)
void  compute_field(vector<float>& field)
float get_theta()
float get_dist()
float get_phi()
```

**Detailed Description**

1. **obstacle()**

```cpp
obstacle( float d_in,
          float phi_in,
          float theta_in )
```

Instantiates the obstacle object.

*Parameters*:

* **d_in** - Mean distance
* **phi_in** - Angular width
* **theta_in** - Mean angle of the obstacle

2. **increase_width()**

```cpp
void increase_width(float w)
```

Increases the width of all obstacles as per the bot’s width.

*Parameters*:

* **w** - Width of the bot

3. **compute_field()**

```cpp
void  compute_field(vector<float>& field)
```

Computes the Gaussian potential field due to this obstacle.

*Parameters*:

* **field** - Potential field to which this obstacles contribution is added

4. **get_theta()**

```cpp
float get_theta()
```

Returns the angular position of the obstacle.

5. **get_dist()**

```cpp
float get_dist()
```

Returns the distance of the obstacle from the bot.

5. **get_phi()**

```cpp
float get_phi()
```

Returns the angular width of the obstacle.

### GaussianPF_PathPlanning.cpp

```cpp
#include "GaussianPF_PathPlanning.cpp"
```

This module uses the OBD-GF header file for implementing the algorithm.

#### Methods

```cpp
vector<obstacle> get_obstacles(vector<float> polar_dat)
vector<obstacle> process_obs(vector<obstacle> &obs)
int              get_best_header(vector<float> potential)
float            get_header_rad(vector<float> polardat, float goal_angle, bool showPlot)
```

#### Description

1. **get_obstacles()**

```cpp
vector<obstacle> get_obstacles(vector<float> polar_dat)
```

Returns an array of obstacle instances for the given lidar scan data.

*Parameters:*

* **polar_dat**  - Lidar scan data (array of distances)

2. **process_obs()**

```cpp
vector<obstacle> process_obs(vector<obstacle> &obs)
```

Augments the obstacle array by increasing the angular width of each obstacle as per the bot's width.

*Parameters*:
* **obs** - Array of obstacles obtained from get_obstacles()

3. **get_best_header()**

```cpp
int get_best_header(vector<float> potential)
```

Returns the angle with minimum potential given the potential field.

*Parameters:*

* **potential** - Potential field array : value of potential at the index angle

4. **get_header_rad()**

```cpp
get_header_rad( vector<float> polardat,
                float goal_angle,
                bool showPlot)
```

Returns the angle of minimum potential in radians given the lidar scan data.

*Parameters* :

* **polardat** - Lidar scan data
* **goal_angle** - The desired heading angle towards the goal
* **show_plot** - Determines whether potential field plots are shown
