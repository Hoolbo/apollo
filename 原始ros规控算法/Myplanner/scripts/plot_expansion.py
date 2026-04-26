import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend

# Vehicle parameters
L_f = 0.77
L_r = 0.77
W = 0.4
L_box = 0.5  # Approximate visible box length

def draw_vehicle(ax, x, y, theta, gamma, color, alpha=1.0, label=None):
    # Front body center
    x_f, y_f = x, y
    
    # Joint position
    x_j = x_f - L_f * np.cos(theta)
    y_j = y_f - L_f * np.sin(theta)
    
    # Rear body center
    theta_r = theta - gamma
    x_r = x_j - L_r * np.cos(theta_r)
    y_r = y_j - L_r * np.sin(theta_r)
    
    # Draw front box
    front_bl_x = x_f - (L_box/2)*np.cos(theta) + (W/2)*np.sin(theta)
    front_bl_y = y_f - (L_box/2)*np.sin(theta) - (W/2)*np.cos(theta)
    front_rect = patches.Rectangle((front_bl_x, front_bl_y), L_box, W, angle=np.degrees(theta),
                                   linewidth=1.5, edgecolor=color, facecolor=color, alpha=alpha*0.3)
    ax.add_patch(front_rect)
    # Highlight boundary
    front_border = patches.Rectangle((front_bl_x, front_bl_y), L_box, W, angle=np.degrees(theta),
                                   linewidth=1.5, edgecolor=color, facecolor='none', alpha=alpha)
    ax.add_patch(front_border)
    
    # Draw rear box
    rear_bl_x = x_r - (L_box/2)*np.cos(theta_r) + (W/2)*np.sin(theta_r)
    rear_bl_y = y_r - (L_box/2)*np.sin(theta_r) - (W/2)*np.cos(theta_r)
    rear_rect = patches.Rectangle((rear_bl_x, rear_bl_y), L_box, W, angle=np.degrees(theta_r),
                                  linewidth=1.5, edgecolor=color, facecolor=color, alpha=alpha*0.3)
    ax.add_patch(rear_rect)
    rear_border = patches.Rectangle((rear_bl_x, rear_bl_y), L_box, W, angle=np.degrees(theta_r),
                                  linewidth=1.5, edgecolor=color, facecolor='none', alpha=alpha)
    ax.add_patch(rear_border)
    
    # Draw linkage (back of front to front of rear is connected by joint, but let's just connect centers to joint)
    ax.plot([x_f, x_j], [y_f, y_j], color='black', linewidth=2, alpha=alpha*0.8)
    ax.plot([x_j, x_r], [y_j, y_r], color='black', linewidth=2, alpha=alpha*0.8)
    # Joint dot
    ax.scatter(x_j, y_j, color='black', s=40, zorder=5, alpha=alpha)

def simulate_step(state, step, dgamma_target, v_desire=1.0, gamma_dot_max=5.0):
    x, y, theta, gamma = state
    path_x, path_y = [x], [y]
    
    sim_step = 0.05
    steps = int(np.ceil(abs(step) / sim_step))
    actual_step = step / steps
    
    for _ in range(steps):
        # limit gamma_dot
        max_dgamma = gamma_dot_max * (abs(actual_step) / v_desire)
        diff = dgamma_target - gamma
        if abs(diff) > max_dgamma:
            dgamma_step = np.sign(diff) * max_dgamma
        else:
            dgamma_step = diff
            
        next_gamma = gamma + dgamma_step
        
        # kinematics
        d_theta = (actual_step * np.sin(gamma) + L_r * dgamma_step) / (L_f * np.cos(gamma) + L_r)
        theta = theta + d_theta
        x = x + actual_step * np.cos(theta + d_theta * 0.5)
        y = y + actual_step * np.sin(theta + d_theta * 0.5)
        gamma = next_gamma
        
        path_x.append(x)
        path_y.append(y)
        
    return (x, y, theta, gamma), path_x, path_y

def main():
    fig, ax = plt.subplots(figsize=(12, 10))
    
    base_state = (0.0, 0.0, 0.0, 0.0)
    
    # Expansion parameters
    move_step = 2.0
    move_step_backwards = -2.0
    gammas = [-0.6, 0.0, 0.6]  # Right, Straight, Left
    
    # Forward expansion
    for g in gammas:
        end_state, px, py = simulate_step(base_state, move_step, g)
        ax.plot(px, py, 'b--', linewidth=2, alpha=0.6)
        draw_vehicle(ax, *end_state, color='blue', alpha=0.7)
        
    # Backward expansion
    for g in gammas:
        end_state, px, py = simulate_step(base_state, move_step_backwards, g)
        ax.plot(px, py, 'r--', linewidth=2, alpha=0.6)
        draw_vehicle(ax, *end_state, color='red', alpha=0.7)
        
    # Draw base vehicle LAST so it stays on top
    draw_vehicle(ax, *base_state, color='black', alpha=1.0)
        
    ax.set_aspect('equal')
    ax.margins(0.25)
    
    # Style formatting for a paper
    plt.title('Articulated Hybrid A* Node Expansion Kinematics', fontsize=18, fontweight='bold', pad=20)
    plt.xlabel('X (m)', fontsize=14)
    plt.ylabel('Y (m)', fontsize=14)
    
    # Add a legend-like text box
    props = dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray')
    ax.text(0.02, 0.97, r'Forward ($\Delta s > 0$)', transform=ax.transAxes, fontsize=14, color='blue', verticalalignment='top', bbox=props)
    ax.text(0.02, 0.90, r'Reverse ($\Delta s < 0$)', transform=ax.transAxes, fontsize=14, color='red', verticalalignment='top', bbox=props)
    ax.text(0.02, 0.83, r'$\Delta \gamma \in \{-0.6, 0, 0.6\}$ rad', transform=ax.transAxes, fontsize=14, color='darkgreen', verticalalignment='top', bbox=props)
    
    plt.grid(True, linestyle=(0, (5, 5)), alpha=0.5)
    plt.tight_layout()
    
    # Save to current working directory
    output_path = 'expansion_tree.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved {output_path}")

if __name__ == "__main__":
    main()
