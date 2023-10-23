import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys
import os

def plot_trajectories(csv_path, pdf_path=None):
    # Read the CSV file
    df = pd.read_csv(csv_path)

    # Extract time for x-axis
    time = df['time'].to_numpy()

    # If no PDF path is provided, save it in the same directory as the CSV
    if pdf_path is None:
        pdf_path = os.path.join(os.path.dirname(csv_path), 'trajectories.pdf')

    # Create a PDF to save the plots
    with PdfPages(pdf_path) as pdf:

        # Plot positions
        fig, axs = plt.subplots(4, 1, figsize=(8, 12))
        fig.subplots_adjust(hspace=0.5)  # Adjust the space between subplots
        axs[0].plot(time, df['x'].to_numpy(), label='x')
        axs[0].plot(time, df['des_x'].to_numpy(), label='des_x')
        axs[0].set_title('x vs des_x')
        axs[0].legend()
        
        axs[1].plot(time, df['y'].to_numpy(), label='y')
        axs[1].plot(time, df['des_y'].to_numpy(), label='des_y')
        axs[1].set_title('y vs des_y')
        axs[1].legend()
        
        axs[2].plot(time, df['z'].to_numpy(), label='z')
        axs[2].plot(time, df['des_z'].to_numpy(), label='des_z')
        axs[2].set_title('z vs des_z')
        axs[2].legend()
        
        axs[3].plot(time, df['yaw'].to_numpy(), label='yaw')
        axs[3].plot(time, df['des_yaw'].to_numpy(), label='des_yaw')
        axs[3].set_title('yaw vs des_yaw')
        axs[3].legend()
        
        fig.suptitle('Positions')
        pdf.savefig(fig)

        # Plot velocities
        fig, axs = plt.subplots(4, 1, figsize=(8, 12))
        fig.subplots_adjust(hspace=0.5)
        axs[0].plot(time, df['v_x'].to_numpy(), label='v_x')
        axs[0].plot(time, df['des_vx'].to_numpy(), label='des_vx')
        axs[0].set_title('v_x vs des_vx')
        axs[0].legend()
        
        axs[1].plot(time, df['v_y'].to_numpy(), label='v_y')
        axs[1].plot(time, df['des_vy'].to_numpy(), label='des_vy')
        axs[1].set_title('v_y vs des_vy')
        axs[1].legend()
        
        axs[2].plot(time, df['v_z'].to_numpy(), label='v_z')
        axs[2].plot(time, df['des_vz'].to_numpy(), label='des_vz')
        axs[2].set_title('v_z vs des_vz')
        axs[2].legend()
        
        axs[3].plot(time, df['v_yaw'].to_numpy(), label='v_yaw')
        axs[3].plot(time, df['des_v_yaw'].to_numpy(), label='des_v_yaw')
        axs[3].set_title('v_yaw vs des_v_yaw')
        axs[3].legend()
        
        fig.suptitle('Velocities')
        pdf.savefig(fig)

        # Plot accelerations
        fig, axs = plt.subplots(4, 1, figsize=(8, 12))
        fig.subplots_adjust(hspace=0.5)
        axs[0].plot(time, df['a_x'].to_numpy(), label='a_x')
        axs[0].plot(time, df['des_ax'].to_numpy(), label='des_ax')
        axs[0].set_title('a_x vs des_ax')
        axs[0].legend()
        
        axs[1].plot(time, df['a_y'].to_numpy(), label='a_y')
        axs[1].plot(time, df['des_ay'].to_numpy(), label='des_ay')
        axs[1].set_title('a_y vs des_ay')
        axs[1].legend()
        
        axs[2].plot(time, df['a_z'].to_numpy(), label='a_z')
        axs[2].plot(time, df['des_az'].to_numpy(), label='des_az')
        axs[2].set_title('a_z vs des_az')
        axs[2].legend()
        
        axs[3].plot(time, df['a_yaw'].to_numpy(), label='a_yaw')
        axs[3].plot(time, df['des_a_yaw'].to_numpy(), label='des_a_yaw')
        axs[3].set_title('a_yaw vs des_a_yaw')
        axs[3].legend()
        
        fig.suptitle('Accelerations')
        pdf.savefig(fig)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python plot_trajectories.py <path_to_csv> [path_to_pdf]")
        sys.exit(1)

    csv_path = sys.argv[1]
    pdf_path = sys.argv[2] if len(sys.argv) > 2 else None

    plot_trajectories(csv_path, pdf_path)
