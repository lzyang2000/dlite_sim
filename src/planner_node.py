#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from heapq import heappush, heappop
from nav_msgs.msg import Path
import pdb


class planner():
    def __init__(self):
        self.map = None
        self.pos = [0, 0]
        self.grid_pos = [0, 0]
        self.map_init = False
        self.path_msg = None
        self.vel = np.array([[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1], [1, 1, 1], [
            1, -1, 1], [-1, 1, 1], [-1, -1, 1]])  # x y cost
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_sub = rospy.Subscriber("/odom", Odometry, self.update_pos)
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.update_goal)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)

    def update_pos(self, msg):
        if self.map is None:
            return [0, 0]
        self.pos = [msg.pose.pose.position.y, msg.pose.pose.position.x]
        self.update_grid()

        # grid_x = (transform.getOrigin().x() - (int)map.info.origin.position.x) / map.info.resolution;
        # grid_y = (transform.getOrigin().y() - (int)map.info.origin.position.y) / map.info.resolution;
        # currentPoint = map.data[grid_y * map.info.width + grid_x];

    def update_grid(self):
        grid_x = int(
            (self.pos[1] - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int(
            (self.pos[0] - self.map.info.origin.position.y) / self.map.info.resolution)
        self.grid_pos = [grid_y, grid_x]
        self.start = self.grid_pos

    def h(self, s):
        return 1

    def calculate_key(self, s):  # 1
        return (min(self.g[s[0]][s[1]], self.rhs[s[0]][s[1]]) + self.h(s), min(self.g[s[0]][s[1]], self.rhs[s[0]][s[1]]))

    def c(self, s1, s2):
        if self.map.data[s2[0] * self.map.info.width + s2[1]] == 100:
            return np.inf
        velo = list(
            filter(lambda i: i[1] == (s1[0] - s2[0]) and i[0] == (s1[1] - s2[1]), self.vel))
        return velo[0][2]

    def update_goal(self, msg):
        # TODO: x is column, y is row
        goal_x = int(
            (msg.pose.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
        goal_y = int(
            (msg.pose.position.y - self.map.info.origin.position.y) / self.map.info.resolution)
        self.start = [self.grid_pos[0], self.grid_pos[1]]
        self.goal = [goal_y, goal_x]
        self.U = []  # 2
        self.g = np.full(
            (self.map.info.height, self.map.info.width), np.inf)  # 3
        self.rhs = np.full(
            (self.map.info.height, self.map.info.width), np.inf)  # 3
        self.rhs[self.goal[0]][self.goal[1]] = 0  # 4
        self.U.append((self.calculate_key(self.goal), self.goal))  # 5
        self.U.sort(key=lambda x: x[0])

        self.main()

    def update_vertex(self, u):
        if not (u[0] == self.goal[0] and u[1] == self.goal[1]):
            self.rhs[u[0]][u[1]] = min([self.g[sp[0]][sp[1]] + self.c(u, sp)
                                        for sp in self.around(u)])  # 6
        for i in range(len(self.U)):  # 7
            _, s = self.U[i]
            if u[0] == s[0] and u[1] == s[1]:
                self.U.pop(i)
                break
        if self.g[u[0]][u[1]] != self.rhs[u[0]][u[1]]:
            self.U.append((self.calculate_key(u), u))  # 08
            self.U.sort(key=lambda x: x[0])

    def around(self, s):
        neighbors = []
        for motion in self.vel:
            neighbor = [s[0] + motion[1], s[1] + motion[0]]
            if neighbor[0] >= 0 and neighbor[1] >= 0 and neighbor[0] < self.map.info.height and neighbor[1] < self.map.info.width:
                neighbors.append(neighbor)
        return neighbors

    def compute_shortest_path(self):
        self.U.sort(key=lambda x: x[0])
        while len(self.U) > 0 and (self.U[0][0][0] < self.calculate_key(self.start)[0] or (self.U[0][0][0] == self.calculate_key(self.start)[0] and self.U[0][0][1] < self.calculate_key(self.start)[1])) or self.rhs[self.start[0]][self.start[1]] != self.g[self.start[0]][self.start[1]]:  # 9
            _, u = self.U.pop(0)  # 10
            if (self.g[u[0], u[1]] > self.rhs[u[0], u[1]]):  # 11
                self.g[u[0], u[1]] = self.rhs[u[0], u[1]]  # 12
                for s in self.around(u):  # 13
                    self.update_vertex(s)
            else:  # 14
                self.g[u[0], u[1]] = np.inf  # 15
                for s in self.around(u) + [u]:  # 16
                    self.update_vertex(s)

    def main(self):
        path = []
        path.append(self.start)
        self.compute_shortest_path()
        while (not np.isclose(self.goal[0], self.start[0], atol=1)) and (not np.isclose(self.goal[1], self.start[1], atol=1)):  # path reach goal
            if self.g[self.start[0]][self.start[1]] == np.inf:
                rospy.logerr("Failed to find path!")
                return
            self.start = min(self.around(self.start),
                             key=lambda sp:
                             self.g[sp[0]][sp[1]] + self.c(self.start, sp)
                             )  # select next step
            path.append(self.start)
        # publish path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = p[
                1] * self.map.info.resolution + self.map.info.origin.position.x
            pose.pose.position.y = p[
                0] * self.map.info.resolution + self.map.info.origin.position.y
            self.path_msg.poses.append(pose)
        # self.path_pub.publish(self.path_msg)
        return

    def update_map(self, msg):
        self.map = msg
        if not self.map_init:  # run only once
            self.map_init = True
            self.last_map = np.array(self.map.data[:])  # copy map history
        # compare map history and current map data for values that changed to
        # 100
        changed_index = np.where(np.array(self.map.data) != self.last_map)
        obstacle_list = np.where(np.array(self.map.data) == 100)
        # find common elements in two arrays
        changed_vertices = np.intersect1d(changed_index, obstacle_list)
        changed_vertices = np.array(
            [changed_vertices // self.map.info.width, changed_vertices % self.map.info.width]).T

        if len(changed_vertices) != 0:  # 21
            for v in changed_vertices:
                if self.start[0] == v[0] and self.start[1] == v[1]:
                    continue
                self.rhs[v[0]][v[1]] = np.inf  # 22
                self.g[v[0]][v[1]] = np.inf
                self.update_vertex(v)  # 23
            self.main()
        self.last_map = np.array(self.map.data[:])  # update map history

    def publish_path(self):
        if self.path_msg != None:
            self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    rospy.init_node('planner_node')
    plan = planner()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        plan.publish_path()
        rate.sleep()
    # rospy.spin()
