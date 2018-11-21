# Multi-target tracking for Drosophila melanogaster (Bee-O-Sensor project)

The program takes as an input a sequence of images, recorded during the run of an experiment with Drosophila melanogaster. 
Each image is first processed in order to locate each target on in. Afterwards, to reconstruct target trajectories, we use
Kalman filter for temporal update of positions and velocities, and minimum distance assignent for data association between consecutive frames.
