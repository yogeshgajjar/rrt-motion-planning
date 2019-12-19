# motion_planning
Motion Planning using Rapidly Exploring Random Tree 

## Concept 

Among numereous motion planning algorithms, RRT uses iteratively build tree with clever use of the random sampling that is likely (though not guaranteed) to build one such path from start to goal points. 

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
