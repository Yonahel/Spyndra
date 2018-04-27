# Machine Learning 
-----
This folder contains machine learning algorighms for Spyndra's walking pattern generation.

Remember to source ros package before running any of the algorithms provided in this folder.

```
source /opt/ros/indigo/setup.bash
source ~/catwin_ws/devel/setup.bash
``` 
## Random search and Hill Climber

In random, we implemented random search and hill climber algorithms. These two algorithms can generate Spyndra's best walking pattern once converged.
### Random Search

To test this code, directly run 

```
cd random/
python random_search.py --g 10 --n 20 --lr 0.25
```

where `--g` specifies number of gates for each walking pattern, `--n` denotes number of patterns for each episode and `--lr` is percentage of patterns kept for next episode.

### Hill Climber

To test this code, directly run

```
cd random/
python hill_climber.py --g 10 --n 20 --lr 0.25
```

where the arguments are the same as random search. But this time, instead of randomly generate other `n*(1-lr)` patterns, we add arbitrary noises to the kept patterns to generate new ones.


## Deep Q Network (DQN & Dueling DQN)

DQN is able to generate a more robust model for Spyndra, this algorithm produces not one fixed pattern like Random Search or Hill Climber do, but a model that takes action based on its observation.

For algorithm detail, please refer to this [website](https://leonardoaraujosantos.gitbooks.io/artificial-inteligence/content/deep_q_learning.html).

For detail about Dueling DQN, please refer this [paper](https://arxiv.org/abs/1511.06581).

To test this code, directly run

```
cd deepQNetwork/
python main.py
```

Note that this code is unstable due to Gazebo's inner bug for simulation.

## Asynchronous Advantage Actor-Critic (A3C / A2C)

A3C is a more powerful model among deep reinforcement learning. Besides, unlike DQN's discrete output, A3C's output is continuous, making Spyndra's movement more reasonable.

For algorighm detail, please refer to this [website](https://medium.com/emergent-future/simple-reinforcement-learning-with-tensorflow-part-8-asynchronous-actor-critic-agents-a3c-c88f72a5e9f2).

To test this code, directly run

```
cd a3C/
python A3C_continuous_action.py
```

Note that this code is unstable due to Gazebo's inner bug for simulation. Also, this A3C code currently only suply single process, so it is basically A2C.

## Acknowledgement

Thanks to [MorvanZhou](https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow) for deep reinforcement learning code reference.
