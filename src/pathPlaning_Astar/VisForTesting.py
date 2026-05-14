import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


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
