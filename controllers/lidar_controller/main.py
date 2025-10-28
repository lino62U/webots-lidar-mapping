import matplotlib
matplotlib.use("TkAgg")
import pandas as pd
import matplotlib.pyplot as plt
import time
import os

# --- Crear figura ---
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
fig.patch.set_facecolor("white")

for ax in axes:
    ax.set_facecolor("white")
    ax.tick_params(colors="black")
    ax.xaxis.label.set_color("black")
    ax.yaxis.label.set_color("black")
    ax.title.set_color("black")

plt.ion()  # 🔥 modo interactivo
plt.show()

while True:
    # Verificar si existen los CSV
    if not os.path.exists("trajectory.csv") or not os.path.exists("obstacles.csv"):
        print("Esperando archivos CSV...")
        time.sleep(1)
        continue

    try:
        trajectory = pd.read_csv("trajectory.csv")
        obstacles = pd.read_csv("obstacles.csv")
    except pd.errors.EmptyDataError:
        time.sleep(0.5)
        continue

    # Limpiar ejes
    for ax in axes:
        ax.clear()
        ax.set_facecolor("white")

    # --- Obstáculos ---
    if not obstacles.empty:
        axes[0].scatter(obstacles["x_world"], obstacles["z_world"], s=5, c="gray", label="Obstáculos")
    axes[0].set_title("Mapa de obstáculos")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Z (m)")
    axes[0].axis("equal")
    axes[0].legend(loc="upper right")

    # --- Trayectoria ---
    if not trajectory.empty:
        axes[1].plot(trajectory["x_world"], trajectory["z_world"], color="red", linewidth=1, label="Trayectoria")
    axes[1].set_title("Trayectoria del robot")
    axes[1].set_xlabel("X (m)")
    axes[1].set_ylabel("Z (m)")
    axes[1].axis("equal")
    axes[1].legend(loc="upper right")

    # --- Mapa combinado ---
    if not obstacles.empty:
        axes[2].scatter(obstacles["x_world"], obstacles["z_world"], s=5, c="gray", label="Obstáculos")
    if not trajectory.empty:
        axes[2].plot(trajectory["x_world"], trajectory["z_world"], color="red", linewidth=1, label="Trayectoria")
    axes[2].set_title("Mapeo 2D")
    axes[2].set_xlabel("X (m)")
    axes[2].set_ylabel("Z (m)")
    axes[2].axis("equal")
    axes[2].legend(loc="upper right")

    plt.pause(0.5)  # 🔄 refrescar cada medio segundo
