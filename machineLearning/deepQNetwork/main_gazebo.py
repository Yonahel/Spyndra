import time
from dqn import DeepQNetwork
from util_gazebo import SpyndraEnv


def run(episode, timestep):
    for ep in range(episode):
        # initial observation
        print "Start render and reset..."
	env._render()
	observation = env._reset()
	step = 0
        print "Reset complete, Start episode %i" % ep
        time.sleep(2)
	for t in range(timestep):
            # fresh env
            #env._render()
            # RL choose action based on observation
            action = RL.choose_action(observation)

            # RL take action and get next observation and reward
            observation_, reward, done = env._step(action, observation)
	    #print "step finished"
            RL.store_transition(observation, action, reward, observation_)
            print reward, action, done#, observation_
            if (step > 200) and (step % 5 == 0):
                RL.learn()

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                break
            step += 1

    # end of game
    print('game over')

n_actions  = 24
n_features = 35
EPISODE = 100
TIMESTEP = 1000

if __name__ == "__main__":

    env = SpyndraEnv()
    print "Start model initialization..."
    RL = DeepQNetwork(n_actions, n_features,
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=2000,
                      # output_graph=True
                      )
    print "Model initialization complete"
    env._render()
    run(EPISODE, TIMESTEP)

    #RL.plot_cost()
