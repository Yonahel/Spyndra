import numpy as np
import tensorflow as tf

np.random.seed(1)
tf.set_random_seed(1)


# Deep Q Network off-policy
class DeepQNetwork:
	def __init__(
			self,
			n_actions,
			n_features,
			learning_rate=0.01,
			reward_decay=0.9,
			e_greedy=0.9,
			replace_target_iter=300,
			memory_size=500,
			batch_size=32,
			e_greedy_increment=None,
			output_graph=False,
	):
		self.n_actions = n_actions
		#self.n_features = n_features
		self.n_features = n_features
		self.lr = learning_rate
		self.gamma = reward_decay
		self.epsilon_max = e_greedy
		self.replace_target_iter = replace_target_iter
		self.memory_size = memory_size
		self.batch_size = batch_size
		self.epsilon_increment = e_greedy_increment
		self.epsilon = 0 if e_greedy_increment is not None else self.epsilon_max

		# total learning step
		self.learn_step_counter = 0

		# state representation: position(8), speed(8), load(8) of each motor, action_t-1~action_t-5(5), 
		#                       IMU_acceleration_xyz(3), IMU_angularVelocity_rpy(3)
		# initialize zero memory [s, a, r, s_]
		self.memory = np.zeros((self.memory_size, n_features * 2 + 2))

		# consist of [target_net, evaluate_net]
		self._build_net()
		t_params = tf.get_collection('target_net_params')
		e_params = tf.get_collection('eval_net_params')
		self.replace_target_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

		self.sess = tf.Session()

		if output_graph:
			# $ tensorboard --logdir=logs
			# tf.train.SummaryWriter soon be deprecated, use following
			writer = tf.summary.FileWriter("", self.sess.graph)
			writer.close()

		self.sess.run(tf.global_variables_initializer())
		self.cost_his = []

	def _build_net(self):
		# ------------------ build evaluate_net ------------------
		self.s = tf.placeholder(tf.float32, [None, self.n_features], name='s')  # input
		self.q_target = tf.placeholder(tf.float32, [None, self.n_actions], name='Q_target')  # for calculating loss
		with tf.variable_scope('eval_net'):
			# c_names(collections_names) are the collections to store variables
			c_names, n_l1, w_initializer, b_initializer = \
				['eval_net_params', tf.GraphKeys.GLOBAL_VARIABLES], 10, \
				tf.random_normal_initializer(0., 0.3), tf.constant_initializer(0.1)  # config of layers

			# first layer. collections is used later when assign to target net
			with tf.variable_scope('l1'):
				w1 = tf.get_variable('w1', [self.n_features, n_l1], initializer=w_initializer, collections=c_names)
				b1 = tf.get_variable('b1', [1, n_l1], initializer=b_initializer, collections=c_names)
				l1 = tf.nn.relu(tf.matmul(self.s, w1) + b1)

			# second layer. collections is used later when assign to target net
			with tf.variable_scope('l2'):
				w2 = tf.get_variable('w2', [n_l1, self.n_actions], initializer=w_initializer, collections=c_names)
				b2 = tf.get_variable('b2', [1, self.n_actions], initializer=b_initializer, collections=c_names)
				self.q_eval = tf.matmul(l1, w2) + b2

		with tf.variable_scope('loss'):
			self.loss = tf.reduce_mean(tf.squared_difference(self.q_target, self.q_eval))
		with tf.variable_scope('train'):
			self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

		# ------------------ build target_net ------------------
		self.s_ = tf.placeholder(tf.float32, [None, self.n_features], name='s_')    # input
		with tf.variable_scope('target_net'):
			# c_names(collections_names) are the collections to store variables
			c_names = ['target_net_params', tf.GraphKeys.GLOBAL_VARIABLES]

			# first layer (35x10). collections is used later when assign to target net 
			with tf.variable_scope('l1'):
				w1 = tf.get_variable('w1', [self.n_features, n_l1], initializer=w_initializer, collections=c_names)
				b1 = tf.get_variable('b1', [1, n_l1], initializer=b_initializer, collections=c_names)
				l1 = tf.nn.relu(tf.matmul(self.s_, w1) + b1)

			# second layer(10x3). collections is used later when assign to target net
			with tf.variable_scope('l2'):
				w2 = tf.get_variable('w2', [n_l1, self.n_actions], initializer=w_initializer, collections=c_names)
				b2 = tf.get_variable('b2', [1, self.n_actions], initializer=b_initializer, collections=c_names)
				self.q_next = tf.matmul(l1, w2) + b2

	def store_transition(self, s, a, r, s_):
		# Store the action(transition) into memory
		if not hasattr(self, 'memory_counter'):
			self.memory_counter = 0

		transition = np.hstack((s, [a, r], s_))

		# replace the old memory with new memory
		index = self.memory_counter % self.memory_size
		self.memory[index, :] = transition

		self.memory_counter += 1

	def choose_action(self, observation):
		# For Spyndra, we treat actions as specifying a change in joint anble by a fixed value.
		# This allow us to collapse the action space down to 3^8 actions(8 dimension vector) - with each joint having
		# the option of either 0. decreasing, 1.maintain, 2.increasing the angle.
		# However, to break it further down, we consider one joint at a time
		
		# to have batch dimension when feed into tf placeholder
		observation = observation[np.newaxis, :]

		if np.random.uniform() < self.epsilon:
			# forward feed the observation and get q value for every actions
			actions_value = self.sess.run(self.q_eval, feed_dict={self.s: observation})
			action = np.argmax(actions_value)
		else:
			action = np.random.randint(0, self.n_actions) 
		return action

	def learn(self):
		# check to replace target parameters
		if self.learn_step_counter % self.replace_target_iter == 0:
			self.sess.run(self.replace_target_op)
			print('\ntarget_params_replaced\n')

		# sample batch memory from all memory (Experience Replay)
		if self.memory_counter > self.memory_size:
			sample_index = np.random.choice(self.memory_size, size=self.batch_size)
		else:
			sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
		batch_memory = self.memory[sample_index, :]

		q_next, q_eval = self.sess.run(
			[self.q_next, self.q_eval],
			feed_dict={
				self.s_: batch_memory[:, -self.n_features:],  # fixed params  (Exploit old NN)
				self.s : batch_memory[:, :self.n_features],   # newest params (Explore new NN)
			})

		# change q_target w.r.t q_eval's action
		q_target = q_eval.copy() # 32(batch)x4(n_action)
		batch_index = np.arange(self.batch_size, dtype=np.int32)
		eval_act_index = batch_memory[:, self.n_features].astype(int) # each 32 actions
		reward = batch_memory[:, self.n_features + 1]
		
		# we backpropagate this error, q_target, w.r.t. the corresponding action to network
		# leave other actions as error = 0 because we didn't use it.
		# To be more specific, loss = (q_target - q_eval)^2 and that q_target = q_eval except the values at eval_act_index
		q_target[batch_index, eval_act_index] = reward + self.gamma * np.max(q_next, axis=1)

		# train eval network
		_, self.cost = self.sess.run([self._train_op, self.loss],
									  feed_dict={self.s: batch_memory[:, :self.n_features],
												 self.q_target: q_target})
		self.cost_his.append(self.cost)

		# increasing epsilon
		self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
		self.learn_step_counter += 1

	def plot_cost(self):
		# plot the cost for each step
		import matplotlib.pyplot as plt
		plt.plot(np.arange(len(self.cost_his)), self.cost_his)
		plt.ylabel('Cost')
		plt.xlabel('training steps')
		plt.show()



