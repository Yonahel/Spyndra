import multiprocessing
import threading
import tensorflow as tf
import numpy as np
import random
import gym
import os
import shutil
import matplotlib.pyplot as plt
from spyndra_env import SpyndraEnv


N_WORKERS = 1# multiprocessing.cpu_count()
MAX_GLOBAL_EP = 2500
GLOBAL_RUNNING_R = []
GLOBAL_EP = 0
MAX_STEP = 20

n_previous_state = 0
N_S = 8 * (1+n_previous_state)
N_A = 8
A_BOUND = [-5, 5]#[env.action_space.low, env.action_space.high]

minFemur, maxFemur = 350, 630
minTibia, maxTibia = 650, 890

class RandomSearch():
    def __init__(self, As=8, n_gates=10, n_patterns=2, evolve_rate=0.5):
        self.As = As #action space
        self.n_gates = n_gates #number of gates for each pattern
        self.n_patterns = n_patterns #number of patterns for each episode
        self.evolve_rate = evolve_rate #percent of patterns kept after one episode

        self.femur_patterns = np.random.randint(low=minFemur, high=maxFemur, size=(n_patterns, As/2, n_gates))
        self.tibia_patterns = np.random.randint(low=minTibia, high=maxTibia, size=(n_patterns, As/2, n_gates))
        self.patterns = np.concatenate((self.femur_patterns, self.tibia_patterns), axis=1)

    def clip(self, v, minv, maxv):
        return max(min(v, maxv), minv)

    def update(self, reward):
        '''
        1. Select top `evolve_rate` percent of patterns, keep them. 
        2. Add gaussian noise to thosed kept patterns
        3. Randomly generate new patterns to totaly n different patterns
        '''
        # step 1, 2
        n_keep = int(self.n_patterns * self.evolve_rate)
        top_idx = sorted(range(len(reward)), key=lambda k: reward[k], reverse=True)[ :n_keep]
        self.patterns = self.patterns[top_idx]
        for i in range(len(self.patterns)):
            for k in range(self.n_gates):
                # Femur
                for j in range(self.As/2):
                    gate = self.patterns[i, j, k]
                    noise = np.random.normal(loc=0, scale=min(abs(gate-minFemur), abs(gate-maxFemur))/2)
                    self.patterns[i, j, k] = self.clip(gate + noise, 350, 630)
                # Tibia
                for j in range(self.As/2, self.As):
                    gate = self.patterns[i, j, k]
                    noise = np.random.normal(loc=0, scale=min(abs(gate-minTibia), abs(gate-maxTibia))/2)
                    self.patterns[i, j, k] = self.clip(gate + noise, 650, 890)
        
        # step 3
        n_to_generate = self.n_patterns - n_keep
        self.femur_patterns = np.random.randint(low=minFemur, high=maxFemur, size=(n_to_generate, self.As/2, self.n_gates))
        self.tibia_patterns = np.random.randint(low=minTibia, high=maxTibia, size=(n_to_generate, self.As/2, self.n_gates))
        self.new_patterns = np.concatenate((self.femur_patterns, self.tibia_patterns), axis=1)
        print(self.patterns.shape, self.new_patterns.shape)
	self.patterns = np.concatenate((self.patterns, self.new_patterns), axis=0)

    def choose_action(self, idx_pattern, step):
        return self.patterns[idx_pattern, :, step]


class Worker(object):
    def __init__(self, name):
        self.env = SpyndraEnv(N_S, N_A)
        self.name = name
        self.agent = RandomSearch(N_A)

    def work(self):
        global GLOBAL_RUNNING_R, GLOBAL_EP
        
        while GLOBAL_EP < MAX_GLOBAL_EP:
            rewards = []
            for idx_pattern in range(self.agent.n_patterns):
                
                s = self.env._reset()
                ep_r, dist_traveled = 0, -999.
                for step in range(MAX_STEP):
                
                    a = self.agent.choose_action(idx_pattern, step % self.agent.n_gates)

                    s_, r, done, info = self.env._step(a, s)
                        #done = True if ep_t == MAX_EP_STEP - 1 else False
                    if self.name == 'W_0':
                        print("Ep %4i, %2i th agent, step %4i" % (GLOBAL_EP, idx_pattern, step))
                        print("distance to goal=", info, "reward=", r)
                        print("position before action=", list(s[:8]))
                        print("action=",list(a))
                        print("position after  action=", list(s_[:8]))
                    
                    dist_traveled = 10. - info
                    print("dist_traveled", dist_traveled)
                    
                    if done:
                        print("We get our best model!!!")
                        np.save('best_pattern.npy', self.agent.patterns[idx_pattern])
                        break
                    
                    s = s_
                rewards.append(dist_traveled)
            
            self.agent.update(rewards)
            GLOBAL_EP += 1
            print("Best pattern traveled for", max(rewards))
            with open('ep_reward.txt', 'a') as f:
                f.write('ep=%i, distence traveled=%f\n' % (GLOBAL_EP, max(rewards)))


if __name__ == "__main__":
    SESS = tf.Session()

#    with tf.device("/cpu:0"):
#        OPT_A = tf.train.AdamOptimizer(LR_A, name='RMSPropA')
#        OPT_C = tf.train.AdamOptimizer(LR_C, name='RMSPropC')
#        GLOBAL_AC = ACNet(GLOBAL_NET_SCOPE)  # we only need its params
#        workers = []
        # Create worker
#        for i in range(N_WORKERS):
#            i_name = 'W_%i' % i   # worker name
#            workers.append(Worker(i_name))
    worker = Worker('W_0')
    worker.work()
#    COORD = tf.train.Coordinator() # multiprocess coordinator 
#    SESS.run(tf.global_variables_initializer())

#    if OUTPUT_GRAPH:
#        if os.path.exists(LOG_DIR):
#            shutil.rmtree(LOG_DIR)
#        tf.summary.FileWriter(LOG_DIR, SESS.graph)

#    worker_threads = []
#    for worker in workers:
#        job = lambda: worker.work()
#        t = threading.Thread(target=job)
#        t.start()
#        worker_threads.append(t)

#    for w in workers:
#	w.join()
#    COORD.join(worker_threads) # synchronize all the threads, fast ones wait for slow ones until all threads are done

#    plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
#    plt.xlabel('step')
#    plt.ylabel('Total moving reward')
#    plt.show()


