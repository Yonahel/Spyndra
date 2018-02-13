import multiprocessing
import threading
import tensorflow as tf
import numpy as np
import os
import shutil
#import matplotlib.pyplot as plt
from util_gazebo import SpyndraEnv


OUTPUT_GRAPH = True
LOG_DIR = './log'
N_WORKERS = 1#multiprocessing.cpu_count()
MAX_GLOBAL_EP = 1000

GLOBAL_NET_SCOPE = 'Global_Net'
UPDATE_GLOBAL_ITER = 10 
GAMMA = 0.9
ENTROPY_BETA = 0.001
LR_A = 0.001    # learning rate for actor
LR_C = 0.001    # learning rate for critic
GLOBAL_RUNNING_R = []
GLOBAL_EP = 0
NUM_STATE  = 38
NUM_ACTION = 24


class ACNet(object):
	def __init__(self, nS, nA, scope, globalAC=None):
		self.nS, self.nA = nS, nA
		if scope == GLOBAL_NET_SCOPE:   # get global network
			with tf.variable_scope(scope):
				self.state = tf.placeholder(tf.float32, [None, self.nS], 'State')
				self.actor_params, self.critic_params = self._build_net(scope)[-2:]
		else:   # local net, calculate losses
			with tf.variable_scope(scope):
				self.state = tf.placeholder(tf.float32, [None, self.nS], 'State')
				self.action = tf.placeholder(tf.int32, [None, ], 'Action')
				self.target_value = tf.placeholder(tf.float32, [None, 1], 'Target_value')

				self.policy, self.value, self.actor_params, self.critic_params = self._build_net(scope)

				advantage = tf.subtract(self.target_value, self.value, name='Advantage')
				with tf.name_scope('critic_loss'):
					self.critic_loss = tf.reduce_mean(tf.square(advantage))

				with tf.name_scope('actor_loss'):
					log_prob = tf.reduce_sum(tf.log(self.policy) * tf.one_hot(self.action, self.nA, dtype=tf.float32), axis=1, keep_dims=True)
					exp_v = log_prob * tf.stop_gradient(advantage)
					entropy = -tf.reduce_sum(self.policy * tf.log(self.policy + 1e-5),
											 axis=1, keep_dims=True)  # encourage exploration
					self.exp_v = ENTROPY_BETA * entropy + exp_v
					self.actor_loss = tf.reduce_mean(-self.exp_v)

				with tf.name_scope('local_grad'):
					self.actor_grads = tf.gradients(self.actor_loss, self.actor_params)
					#self.actor_grads = [tf.clip_by_average_norm(g, .1) for g in self.actor_grads]
					self.critic_grads = tf.gradients(self.critic_loss, self.critic_params)
					#self.critic_grads = [tf.clip_by_average_norm(g, .1) for g in self.critic_grads]

			with tf.name_scope('sync'):
				with tf.name_scope('pull'):
					self.pull_actor_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.actor_params, globalAC.actor_params)]
					self.pull_critic_params_op = [l_p.assign(g_p) for l_p, g_p in zip(self.critic_params, globalAC.critic_params)]
				with tf.name_scope('push'):
					self.update_actor_op = OPT_A.apply_gradients(zip(self.actor_grads, globalAC.actor_params))
					self.update_critic_op = OPT_C.apply_gradients(zip(self.critic_grads, globalAC.critic_params))

	def _build_net(self, scope):
		w_init = tf.random_normal_initializer(0., .1)
		with tf.variable_scope('actor'):
			l_a_1 = tf.layers.dense(self.state, 20, tf.nn.relu6, kernel_initializer=w_init, name='layer_actor_1')
			policy = tf.layers.dense(l_a_1, self.nA, tf.nn.softmax, kernel_initializer=w_init, name='actor_policy')
		with tf.variable_scope('critic'):
			l_c_1 = tf.layers.dense(self.state, 10, tf.nn.relu6, kernel_initializer=w_init, name='layer_critic_1')
			value = tf.layers.dense(l_c_1, 1, kernel_initializer=w_init, name='critic_value')  # state value
		actor_params  = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/actor')
		critic_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=scope + '/critic')
		return policy, value, actor_params, critic_params

	def update_global(self, feed_dict):  # run by a local
		SESS.run([self.update_actor_op, self.update_critic_op], feed_dict)  # local grads applies to global net

	def pull_global(self):  # run by a local
		SESS.run([self.pull_actor_params_op, self.pull_critic_params_op])

	def choose_action(self, s):  # run by a local
		prob_weights = SESS.run(self.policy, feed_dict={self.state: s[np.newaxis, :]})
		action = np.random.choice(range(prob_weights.shape[1]),
								  p=prob_weights.ravel())  # select action w.r.t the actions prob
		return action


class Worker(object):
	def __init__(self, nS, nA, name, globalAC):
		self.env = SpyndraEnv()
		self.name = name
		self.AC = ACNet(nS, nA, name, globalAC)

	def work(self):
		global GLOBAL_RUNNING_R, GLOBAL_EP
		total_step = 1
		buffer_s, buffer_a, buffer_r = [], [], []
		while not COORD.should_stop() and GLOBAL_EP < MAX_GLOBAL_EP:
			s = self.env._reset()
			ep_r = 0
			# theta for actor, theta_v for critic
			while True:
				# if self.name == 'W_0':
				#     self.env._render()
				
				# Synchronize thread-specific parameters theta' = theta, theta'_v = theta_v
				self.AC.pull_global()
				# Perform action a_t according to policy(a_t | s_t, theta')
				a = self.AC.choose_action(s)
				# Receive reward r_t and new state s_t+1
				s_, r, done = self.env._step(a, s)
				if done: r = -5
				ep_r += r
				buffer_s.append(s)
				buffer_a.append(a)
				buffer_r.append(r)

				if total_step % UPDATE_GLOBAL_ITER == 0 or done:   # update global and assign to local net
					if done: 
						# terminal s_t
						v_s_ = 0   
					else:
						# for non-terminal s_t, bootstrap from last state to get V(s_t, theta'_v)
						v_s_ = SESS.run(self.AC.value, {self.AC.state: s_[np.newaxis, :]})[0, 0]
					buffer_v_target = []
					for r in buffer_r[::-1]:    # reverse buffer r
						v_s_ = r + GAMMA * v_s_
						buffer_v_target.append(v_s_)
					buffer_v_target.reverse()

					buffer_s, buffer_a, buffer_v_target = np.vstack(buffer_s), np.array(buffer_a), np.vstack(buffer_v_target)
					feed_dict = {
						self.AC.state: buffer_s,
						self.AC.action: buffer_a,
						self.AC.target_value: buffer_v_target,
					}
					# This below function do 3 things:
					# 1. Accumulate gradients w.r.t. theta'  : dtheta <-- dtheta + gradient(log(policy) * (advantage))
					# 2. Accumulate gradients w.r.t. theta'_v: dtheta_v <-- dtheta_v + gradient(advantage^2)
					# 3. Perform asynchronous update of theta using dtheta and theta_v using dtheta_v
					self.AC.update_global(feed_dict)

					buffer_s, buffer_a, buffer_r = [], [], []
					
				s = s_
				total_step += 1
				if done:
					if len(GLOBAL_RUNNING_R) == 0:  # record running episode reward
						GLOBAL_RUNNING_R.append(ep_r)
					else:
						GLOBAL_RUNNING_R.append(0.99 * GLOBAL_RUNNING_R[-1] + 0.01 * ep_r)
					print(
						self.name,
						"Ep:", GLOBAL_EP,
						"| Ep_r: %i" % GLOBAL_RUNNING_R[-1],
						  )
					GLOBAL_EP += 1
					break

if __name__ == "__main__":
	SESS = tf.Session()

	with tf.device("/cpu:0"):
		OPT_A = tf.train.RMSPropOptimizer(LR_A, name='RMSPropA')
		OPT_C = tf.train.RMSPropOptimizer(LR_C, name='RMSPropC')
		GLOBAL_AC = ACNet(NUM_STATE, NUM_ACTION, GLOBAL_NET_SCOPE)  # we only need its params
		workers = []
		# Create worker
		for i in range(N_WORKERS):
			i_name = 'W_%i' % i   # worker name
			workers.append(Worker(NUM_STATE, NUM_ACTION, i_name, GLOBAL_AC))


	COORD = tf.train.Coordinator()
	SESS.run(tf.global_variables_initializer())
		
	if OUTPUT_GRAPH:
		if os.path.exists(LOG_DIR):
			shutil.rmtree(LOG_DIR)
		tf.summary.FileWriter(LOG_DIR, SESS.graph)

	worker_threads = []
	for worker in workers:
		job = lambda: worker.work()
		t = threading.Thread(target=job)
		t.start()
		worker_threads.append(t)
	COORD.join(worker_threads)

	plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
	plt.xlabel('step')
	plt.ylabel('Total moving reward')
	plt.show()
