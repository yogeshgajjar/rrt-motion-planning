# motion_planning
Motion Planning using Rapidly Exploring Random Tree 

## Concept 

Among numereous motion planning algorithms, RRT uses iteratively build tree with clever use of the random sampling that is likely (though not guaranteed) to build one such path from start to goal points. Unlike PRM for multi-query planning algorithm, RRT's were introduced as a single-query planning algorithm which covers the entire space between the start node and the goal node. 

## Construction of Trees

- We first randomly sample a point in the search space. Let's call it `P_sample`
- We then compute the direction vector between the closest node `N_close` in our search tree and `P_sample`. 
- Now we create a new node that is a fixed distance δ away from `N_close` along our direction vector, which we’ll call `N_new`
- If `N_new` is not in collision with an obstacle, we add it to our search tree.
- After every new addition, we check if that point is within δ of the goal position. If so, we connect to the goal and draw our path.

Note that the path we build is quite convoluted. RRT will only build a valid path, not necessarily the shortest path! The algorithm provides no proof of optimality, but it is relatively fast to compute compared to other path planning algorithms.

Pseudo Code for RRT 
```
RapidlyExploringRandomTree(q_s, n)

V = {q_s};
E = {};

for i = 1 to n dp
    q_rand = Sample();
    q_nearest = Nearest(V, E, q_rand);
    q_new = Steer(q_nearest, q_rand);
    
    if ObstacleFree(q_nearest, q_new) then
        V = V + {q_new};
        E = E + {(q_nearest, q_new)};
    end
end
return V,E;

```
## Testing 

Run unit tests for the RRT with the following commands: 

```
cd code/rrt 
# Run all tests for RRT 
nose2 test.test_rrt
# Run a single test class 
nose2 test.test_rrt.TestRRTInit
#Run a single test case 
nose2 test.test_rrt.TestRRTInit.test_rrt_init_basic
```

## Visualization 

Follow the steps for visualization, 
```
# Run the PlanarRRT 
nose2 test.test_planar_rrt
```
![RRT Tree](/home/yogesh/Git/motion_planning/Figure_1.png)
Format: ![Alt Text](url)
