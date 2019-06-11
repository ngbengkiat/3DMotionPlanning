## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes a-star search in a grid space. The search is only in NSEW direction.
As the goal is (10,10) away, this resulted in a staircase like diagonal path as shown below:

Y
_____________________________________________________________________
10|     |     |     |     |     |     |     |     |     | 9,10|10,10|
9 |     |     |     |     |     |     |     |     | 8,9 | 9,9 |
8 |     |     |     |     |     |     |     | 7,8 | 8,8 | 
7 |     |     |     |     |     |     | 6,7 | 7,7 |
6 |     |     |     |     |     | 5,6 | 6,6 |
5 |     |     |     |     | 4,5 | 5,5 |
4 |     |     |     | 3,4 | 4,4 |
3 |     |     | 2,3 | 3,3 |
2 |     | 1,2 | 2,2 |
1 | 0,1 | 1,1 |
0 | 0,0 | 

As there is no smoothing of the path, this resulted in the quad moving grid by grid in a step like manner.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

    read the first line. replace comma with space and split the string.
    Convert second a fourth items to float into lat0 and lon0

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

    grid_start = (int(self.local_position[0])-north_offset, int(self.local_position[1])-east_offset)

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

        goal_local = global_to_local(goal_gps, self.global_home)
        grid_goal = (int(goal_local[0])-north_offset, int(goal_local[1])-east_offset)
        
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.
    
    Add 4 more directions to Action list
    I have modified A* so that even if next node is in visited but the cost is higher, next_node is put into queue. See below:
    
    #check if next path is lower cost 
                elif branch[next_node][0]>branch_cost:
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))  
                    
#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

    This is like path pruning in previous lesson. For every neighbouring 3 points, do a collinearity check by calculating their determinant. If the value is low, remove the middle way point.
    

### Execute the flight
#### 1. Does it work?
It works!

But sometimes it crashes into building as speed buildup is too high for long straight.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

I have tried Voronoi edges but I'm keep encountering error when executing self.send_waypoints()
I have included the files. planning_utils2.py and motion_planning2.py

Any help will be appreciated as I couldn't figure out the error message.
