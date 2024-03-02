# ArtificialLife Project
**Code are listed under different branches**

**Step 1: Basic Simulation with Mujoco**

*Objective:* 
Introduce a basic body structure into the simulation.

*Tasks:*
1. Create a simulated body in mujoco.
2. Implement movement capabilities.

**Step 2: Genotype to Phenotype**

*Objective:* 
Implement the directed graph genotype-to-phenotype conversion for creating 3D creatures, as described in Karl Sims's paper "Evolving 3D Morphology and Behavior by Competition."

*Tasks:*

Develop the genotype-to-phenotype mapping for 3D creature creation.

**Step 3: Fitness Function and Iterative Simulations.**

*Objective:* 
This step procedurally generate random bodies and evaluate the bodies in simulation according to a fitness measure of your choosing.

*Tasks:*
1. Modify the fitness function to favor the distance traveled.
2. Using the best feature with highest fittness score as a baseline to do subsequent evolutions. 

**Step 4: Evolutionary Mutations and Stability.**

*Objective:* 
Implement mutations in the physical attributes of the simulated bodies to explore the impact on fitness, focusing on mutations to body and wing sizes and masses.

*Tasks:*
1. Introduce mutations to the body and wing sizes and masses.
2. Evaluate the impact of these mutations on the fitness scores across generations.
3. Address simulation stability issues arising from extreme mutations.

