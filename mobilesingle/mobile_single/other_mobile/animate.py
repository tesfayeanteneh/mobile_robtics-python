import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import pandas as pd

def animate():
    # Load the simulation data
    df_robot_paths = pd.read_csv('robot_paths.csv')

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.grid(True)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Robot Paths on 20x20 Grid Map')

    # Plot the desired path
    waypoints = [(2, 2), (5, 5), (10, 10), (15, 15), (18, 18)]
    for i in range(len(waypoints) - 1):
        ax.plot([waypoints[i][0], waypoints[i + 1][0]], [waypoints[i][1], waypoints[i + 1][1]], 'x--', label='Desired Path' if i == 0 else "")

    # Create a small rectangle area for displaying positions
    position_box = patches.FancyBboxPatch((0, 18), 5, 2, boxstyle="round,pad=0.3", edgecolor="black", facecolor="white")
    ax.add_patch(position_box)
    position_text = ax.text(0.5, 19, "", fontsize=10)

    def update_frame(frame):
        if frame < len(df_robot_paths):
            ax.cla()
            ax.grid(True)
            ax.set_xlim(0, 20)
            ax.set_ylim(0, 20)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('Robot Paths on 20x20 Grid Map')
            for i in range(len(waypoints) - 1):
                ax.plot([waypoints[i][0], waypoints[i + 1][0]], [waypoints[i][1], waypoints[i + 1][1]], 'x--', label='Desired Path' if i == 0 else "")
            ax.plot(df_robot_paths['robot_1_x'][frame], df_robot_paths['robot_1_y'][frame], 'ro', label='Robot 1')
            ax.plot(df_robot_paths['robot_2_x'][frame], df_robot_paths['robot_2_y'][frame], 'bo', label='Robot 2')
            ax.add_patch(position_box)
            position_text.set_text(f"Robot 1: ({df_robot_paths['robot_1_x'][frame]:.2f}, {df_robot_paths['robot_1_y'][frame]:.2f})\n"
                                   f"Robot 2: ({df_robot_paths['robot_2_x'][frame]:.2f}, {df_robot_paths['robot_2_y'][frame]:.2f})")
            ax.legend()
        return ax,

    # Create animation
    anim = animation.FuncAnimation(fig, update_frame, frames=len(df_robot_paths), interval=100, blit=False, repeat=False)

    # Save the animation as an mp4 file
    anim.save('robot_paths1.mp4', writer='ffmpeg', fps=10)

if __name__ == "__main__":
    animate()
