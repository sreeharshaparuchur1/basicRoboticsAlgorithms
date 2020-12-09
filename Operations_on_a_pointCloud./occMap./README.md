We generate a simple occupancy map without a bayesian update rule.

A threshold (k) to indicate our confidence in marking a grid as occupied or not. If a grid is marked as 1 (white, occupied), it means that the LiDAR has reported more than 'k' instances of that 3D point in the world.
The higher the value of 'k' the more "robust" our occupancy map is to noise. The value of K shouldn't be too high as to mark a grid as "empty" when in reality it isn't.

We vary this threshold as well as the number of frames concatenated and see how the occupancy map takes on a different structures. 
