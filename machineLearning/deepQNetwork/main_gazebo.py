import time
from dqn import DeepQNetwork
from util_gazebo import SpyndraEnv


def run(episode, timestep):
    for ep in range(episode):
        # initial observation
        observation = env._reset()
        step = 0
        print "Reset complete, Start episode %i" % ep
	for t in range(timestep):
            # DQN choose action based on observation
            print "Episode %i, time step %i" % (ep, t)
	    action = RL.choose_action(observation)
            
            # DQN take action and get next observation and reward
            observation_, reward, done = env._step(action, observation)
	        #print observation_[:18], observation_[18:36], observation_[36:54], observation_[54:72], observation_[72:]
            
            # Store transition into its memory
            RL.store_transition(observation, action, reward, observation_)
  
            # log print
            
            if (step > 200) and (step % 5 == 0):
		RL.learn()

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                break
            step += 1
            #time.sleep(2)

    # end of game
    print('game over')


n_actions  = 24
# one state consists of 18 dimesions
# 0-7:   motor signal
# 8:     action
# 9-14:  imu data
# 15-17: position
# n_features = [s_t, s_t-1, s_t-2, s_t-3, s_t-4]
n_features = 90 
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
