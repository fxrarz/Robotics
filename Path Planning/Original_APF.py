import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation

class Vector2d():
    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length,
                                                                             self.direction)


class APF():
    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def attractive(self):
        att = (self.goal - self.current_pos) * self.k_att
        return att

    def repulsion(self):
        rep = Vector2d(0, 0) 
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr): 
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (
                        1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)  # 方向由障碍物指向机器人
        return rep

    def path_plan(self):
        # frames = []
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
            if self.is_plot:
                plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
                im = plt.pause(self.delta_t)
                # frames.append(im)
        # ani = animation.ArtistAnimation(fig, frames, interval=50, blit=True, repeat_delay=1000)
        # ani.save('dynamic_images.mp4')
        # print(frames)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True


if __name__ == '__main__':
    k_att, k_rep = 1.0, 100.0
    rr = 1 # circle radius
    step_size, max_iters, goal_threashold = 0.2, 150, 0.2 
    step_size_ = 2


    start, goal = (0, 0), (15, 15)
    is_plot = True
    if is_plot:
        fig = plt.figure(figsize=(7, 7))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')

    # obs = [[1, 4], [2, 4], [3, 3], [6, 1], [6, 7], [10, 6], [11, 12], [12, 13]]
    obs = [[7,11],[8,12],[9,11],[10,10],[11,9],[12,8],[11,7]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')
    # plt.show()
	
    # path plan
    if is_plot:
        apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    else:
        apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)

    apf.path_plan()
    # # print(apf.path)
    # if apf.is_path_plan_success:
    #     path = apf.path
    #     path_ = []
    #     i = int(step_size_ / step_size)
    #     while (i < len(path)):
    #         path_.append(path[i])
    #         i += int(step_size_ / step_size)

        # if path_[-1] != path[-1]:  # 添加最后一个点
            # path_.append(path[-1])
        # print('planed path points:{}'.format(path_))
        # print('path plan success')
        # frames = []
        # if is_plot:
        #     for x, y in path_:
        #         frames.append(subplot.plot(x, y, '.b', markersize=8))

        # print(len(frames))

        # ani = animation.ArtistAnimation(fig, frames, interval=50, blit=True, repeat_delay=1000)
        # ani.save('dynamic_images.mp4')
        # plt.show()


            # px, py = [K[0] for K in path_], [K[1] for K in path_]
            # subplot.plot(px, py, '^k')
            # plt.show()
    # else:
    #     print('path plan failed')
