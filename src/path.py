import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import heapq

from forklift_sim import ForkSim


class Node:
    def __init__(self, cost, state, action, parent):
        self.state = state  # [x, y, theta]
        self.action = action  # [v, fi]
        self.cost = cost  # [costFromStart, costToGoal, fullCost]
        self.parent = parent  # last node state from witch it was created

    def __lt__(self, other):
        return self.cost[2] < other.cost[2]  # compare cost


class AstarHybrid:
    def __init__(self, dt, avalibeActions, goal, stateSpace):
        self.dt = dt  # time period for 1 movement in s
        self.goal = goal  # [x, y, fi]
        self.avalibeActions = avalibeActions  # [[v, fi] [action2]....]
        self.forkSimClass = ForkSim()  # simulation model for new states
        self.stateSpace = stateSpace  # [maxX, maxY]

    def lookAround(self, state):
        avalibeStates = []
        for action in self.avalibeActions:
            avalibeStates.append(self.forkSimClass.move(state, action, self.dt))
        return avalibeStates

    def cost(self, oldState, sCostOld, newState):
        x, y, theta = newState
        gx, gy, gtheta = self.goal

        # calculate goal cost
        d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        t = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)
        gCost = d + t * 0.5

        # calculate cost from start
        oldX, oldY, oldTheta = oldState
        ds = np.sqrt((x - oldX) ** 2 + (y - oldY) ** 2)
        ts = np.abs((theta - oldTheta + np.pi) % (2 * np.pi) - np.pi)
        sCost = ds + ts * 0.5 + sCostOld

        # full cost
        fCost = sCost + gCost

        return [sCost, gCost, fCost]

    def checkBoundaries(self, state):
        x, y, _ = state
        bx, by = self.stateSpace

        return not (0 <= x <= bx and 0 <= y <= by)

    def checkGoal(self, tol, node):
        x, y, theta = node.state
        gx, gy, gtheta = self.goal

        # calculate error
        d = np.sqrt((x - gx) ** 2 + (y - gy) ** 2)
        t = np.abs((theta - gtheta + np.pi) % (2 * np.pi) - np.pi)
        error = d + t * 0.5
        return error < tol

    def reconstructPath(self, goalNode):
        path = []
        action = []
        node = goalNode
        while node is not None:
            path.append(node.state)
            action.append(node.action)
            node = node.parent

        # Flip
        path.reverse()
        action.reverse()

        return path, action


def visualize_path(path, start, goal, stateSpace):
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_facecolor("#1a1a2e")
    fig.patch.set_facecolor("#0f0f23")

    ax.set_xlim(-0.5, stateSpace[0] + 0.5)
    ax.set_ylim(-0.5, stateSpace[1] + 0.5)
    ax.grid(color="#2a2a4a", linewidth=0.5, linestyle="--")
    ax.set_aspect("equal")
    for spine in ax.spines.values():
        spine.set_edgecolor("#3a3a5c")
    ax.tick_params(colors="#888")

    if path:
        xs = [s[0] for s in path]
        ys = [s[1] for s in path]

        # Path trail with fading colour
        for i in range(len(xs) - 1):
            alpha = 0.3 + 0.7 * (i / len(xs))
            ax.plot(
                [xs[i], xs[i + 1]],
                [ys[i], ys[i + 1]],
                color="#00d4ff",
                alpha=alpha,
                linewidth=1.5,
            )

        # Forklift heading arrows every ~10 steps
        step = max(1, len(path) // 10)
        for i in range(0, len(path), step):
            x, y, theta = path[i]
            alpha = 0.3 + 0.7 * (i / len(path))
            ax.annotate(
                "",
                xy=(x + 0.4 * np.cos(theta), y + 0.4 * np.sin(theta)),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color="#00d4ff", lw=1.5, alpha=alpha),
            )

    # Start
    sx, sy, stheta = start
    ax.scatter(sx, sy, s=200, color="#00ff88", zorder=10)
    ax.annotate(
        "",
        xy=(sx + 0.6 * np.cos(stheta), sy + 0.6 * np.sin(stheta)),
        xytext=(sx, sy),
        arrowprops=dict(arrowstyle="->", color="#00ff88", lw=2.5),
    )
    ax.text(sx + 0.3, sy - 0.7, "START", color="#00ff88", fontsize=9, fontweight="bold")

    # Goal
    gx, gy, gtheta = goal
    ax.scatter(gx, gy, s=200, color="#ff6b6b", zorder=10, marker="*")
    ax.annotate(
        "",
        xy=(gx + 0.6 * np.cos(gtheta), gy + 0.6 * np.sin(gtheta)),
        xytext=(gx, gy),
        arrowprops=dict(arrowstyle="->", color="#ff6b6b", lw=2.5),
    )
    ax.text(gx + 0.3, gy - 0.7, "GOAL", color="#ff6b6b", fontsize=9, fontweight="bold")

    legend_elements = [
        mpatches.Patch(color="#00ff88", label="Start"),
        mpatches.Patch(color="#ff6b6b", label="Goal"),
        mpatches.Patch(color="#00d4ff", label="Path"),
    ]
    ax.legend(
        handles=legend_elements,
        loc="upper left",
        facecolor="#1a1a2e",
        edgecolor="#3a3a5c",
        labelcolor="white",
    )

    status = f"Path found — {len(path)} waypoints" if path else "No path found"
    ax.set_title(
        f"Hybrid A* Path Planning\n{status}", color="white", fontsize=13, pad=12
    )
    ax.set_xlabel("X (m)", color="#888")
    ax.set_ylabel("Y (m)", color="#888")

    plt.tight_layout()
    plt.savefig(
        "path_visualization.png",
        dpi=150,
        bbox_inches="tight",
        facecolor=fig.get_facecolor(),
    )
    plt.show()


def MainPathPlaning(dt, start, goal, stateSpace, tol):

    avalibeActions = [
        [3, np.pi / 5],
        [3, 0],
        [3, -np.pi / 5],
    ]  # this defines avalibe movements

    # define planer class
    planer = AstarHybrid(dt, avalibeActions, goal, stateSpace)

    # define starting node
    cost = planer.cost(start, 0, start)
    startNode = Node(cost, start, [0, 0], None)

    # Init parameters needed for pathPlaning
    closed = []
    open = []
    heapq.heappush(open, startNode)

    # main path calculation loop
    while open:
        # 1. select node
        selectedNode = heapq.heappop(open)
        closed.append(selectedNode)

        # 2. expand node
        newStates = planer.lookAround(selectedNode.state)

        # 3. assign state + action + parent to nodes
        for state, action in zip(newStates, avalibeActions):
            # checkBoundaries
            if planer.checkBoundaries(state):
                continue

            # calculate new cost for node
            newCost = planer.cost(selectedNode.state, selectedNode.cost[0], state)

            # save node
            newNode = Node(newCost, state, action, selectedNode)

            #  ceckGoal
            if planer.checkGoal(tol, newNode):
                return planer.reconstructPath(newNode)

            # check if visited
            if any(np.allclose(newNode.state, c.state, atol=0.1) for c in closed):
                continue
            # save
            heapq.heappush(open, newNode)


# path, actions = MainPathPlaning(0.5, [0, 0, 0], [10, 10, 0], [20, 20], 1)
# print(path)

# visualize_path(path, [0, 0, 0], [10, 10, 0], [20, 20])
