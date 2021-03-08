import numpy as np
from heapq import *
import cv2


class Node(object):

    def __init__(self, start, goal, position, clearance, radius):
        self.start = start
        self.goal = goal
        self.position = position
        self.clearance = clearance
        self.radius = radius
        self.cost = 0
        self.y = 0
        self.x = 0
        self.numRows = 300
        self.numCols = 400

    def __eq__(self, other):
        return self.position == other.position

    def IsValid(self, currRow, currCol):
        sum_rc = self.clearance + self.radius
        return (currRow >= (1 + sum_rc) and currRow <= (300 - sum_rc) and currCol >= (1 + sum_rc) and currCol <= (
                    400 - sum_rc))

    def obstacle(self, row, col):
        sum_rc = self.clearance + self.radius

        # check circle
        dist1 = ((row - 90) * (row - 90) + (col - 70) * (col - 70)) - ((35 + sum_rc) * (35 + sum_rc))

        # check ellipse
        dist2 = ((((row - 145) * (row - 145)) / ((30 + sum_rc) * (30 + sum_rc))) + (
                    ((col - 246) * (col - 246)) / ((60 + sum_rc) * (60 + sum_rc)))) - 1



        # # Check rod

        # first = (col - 48) - (row - 108)
        # second = (col - 170) - (row - 194)
        # third = ((col - 36) - (row - 125)
        # fourth = (col - 158) - (row - 210)
        # dist9 = 1
        # dist10 = 1
        # if (first >= 0 and second <= 0 and third >= 0 and fourth <= 0):
        #     dist9 = 0
        #     dist10 = 0

        # # Check C shaped Polygon

        # first = (col - 200)
        # second = (col - 230)
        # third = (row - 270)
        # fourth =(row - 280)
        # dist9 = 1
        # dist10 = 1
        # if (first >= 0 and second <= 0 and third >= 0 and fourth <= 0):
        #     dist9 = 0
        #     dist10 = 0
        #
        # first = (col - 200)
        # second = (col - 210)
        # third = (row - 240)
        # fourth = (row - 270)
        # dist11 = 1
        # dist12 = 1
        # if (first >= 0 and second <= 0 and third >= 0 and fourth <= 0):
        #     dist11 = 0
        #     dist12 = 0
        #
        # first= (col - 200)
        # second = (col - 230)
        # third = (row - 230)
        # fourth = (row - 240)
        # dist13 = 1
        # dist14 = 1
        # if (first_ >= 0 and second_ <= 0 and third_ >= 0 and fourth_ <= 0):
        #     dist13 = 0
        #     dist14 = 0



        if (dist1 <= 0 or dist2 <= 0):
            return True
        return False

    def ActionU(self, currRow, currCol):
        if (self.IsValid(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
            self.x = 0
            self.y = 1
            self.cost = 1
            return True
        else:
            return False

    def ActionUR(self, currRow, currCol):
        if (self.IsValid(currRow + 1, currCol + 1) and self.obstacle(currRow + 1, currCol + 1) == False):
            self.x = 1
            self.y = 1
            self.cost = np.sqrt(2)
            return True
        else:
            return False

    def ActionR(self, currRow, currCol):
        if (self.IsValid(currRow + 1, currCol) and self.obstacle(currRow + 1, currCol) == False):
            self.x = 1
            self.y = 0
            self.cost = 1
            return True
        else:
            return False

    def ActionDR(self, currRow, currCol):
        if (self.IsValid(currRow + 1, currCol - 1) and self.obstacle(currRow + 1, currCol - 1) == False):
            self.x = 1
            self.y = -1
            self.cost = np.sqrt(2)
            return True
        else:
            return False

    def ActionD(self, currRow, currCol):
        if (self.IsValid(currRow, currCol - 1) and self.obstacle(currRow, currCol - 1) == False):
            self.x = 0
            self.y = -1
            self.cost = 1
            return True
        else:
            return False

    def ActionDL(self, currRow, currCol):
        if (self.IsValid(currRow - 1, currCol - 1) and self.obstacle(currRow - 1, currCol - 1) == False):
            self.x = -1
            self.y = -1
            self.cost = np.sqrt(2)
            return True
        else:
            return False

    def ActionL(self, currRow, currCol):
        if (self.IsValid(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
            self.x = -1
            self.y = 0
            self.cost = 1
            return True
        return False

    def ActionUL(self, currRow, currCol):
        if (self.IsValid(currRow - 1, currCol + 1) and self.obstacle(currRow - 1, currCol + 1) == False):
            self.x = -1
            self.y = 1
            self.cost = np.sqrt(2)
            return True
        else:
            return False

    def checkGoal(self, currRow, currCol):
        if (currRow == self.goal[0] and currCol == self.goal[1]):
            print("Goal Reached")
            return True
        else:
            return False

    def Dij(self):
        costMap = {}
        visited_nodes = {}
        path = {}

        for row in np.arange(1, self.numRows + 1, 1):
            for col in np.arange(1, self.numCols + 1, 1):
                costMap[(row, col)] = float('inf')
                visited_nodes[(row, col)] = False
                path[(row, col)] = -1

        explored = []
        queue = []

        heappush(queue, (0, self.start))
        costMap[self.start] = 0

        while (len(queue)) > 0:
            heapify(queue)
            _, currNode = heappop(queue)
            visited_nodes[currNode] = True
            explored.append(currNode)

            if (self.checkGoal(currNode[0], currNode[1]) == True):
                break

            if (self.ActionU(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionUR(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionR(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionDR(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionD(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionDL(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionL(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

            if (self.ActionUL(currNode[0], currNode[1]) and visited_nodes[
                (currNode[0] + self.x, currNode[1] + self.y)] == False and (
                    costMap[(currNode[0] + self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
                costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
                path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
                heappush(queue, (
                costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

        check = []
        goalx = self.goal[0]
        goaly = self.goal[1]

        for a in np.arange(goalx - 1, goalx + 1, 1):
            for b in np.arange(goaly - 1, goaly + 1, 1):
                check.append(costMap[a, b])

        ans = float('inf')
        for c in range(len(check)):
            if (check[c] != ans):
                print("Path Exists")

            NoPath = 1
            break
        if (NoPath == 0):
            print("Path does not exist")
            return (explored, [], costMap[goalx, goaly])

        print(costMap[goalx, goaly], "answer")
        result = (goalx, goaly)

        backstates = []
        node = result
        while (path[node] != -1):
            backstates.append(node)
            node = path[node]
        backstates.append(self.start)
        backstates = list(reversed(backstates))

        print(backstates)
        return (explored, backstates, costMap[goalx, goaly])

    def animate(self, explored, backstates, path):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
        count = 0
        for state in explored:
            image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 150, 0)
            if (count % 75 == 0):
                out.write(image)
            count = count + 1
            cv2.imshow('explored', image)
        count = 0
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                if (image[int(self.numRows - row), int(col - 1), 0] == 0 and image[
                    int(self.numRows - row), int(col - 1), 1] == 0 and image[
                    int(self.numRows - row), int(col - 1), 2] == 0):
                    if (self.IsValid(row, col) and self.obstacle(row, col) == False):
                        image[int(self.numRows - row), int(col - 1)] = (154, 250, 0)
                        if (count % 75 == 0):
                            out.write(image)
                        count = count + 1

        if (len(backstates) > 0):
            for state in backstates:
                image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)

        cv2.waitKey(0)
        cv2.destroyAllWindows()