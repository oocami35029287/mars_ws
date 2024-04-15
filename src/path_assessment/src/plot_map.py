import cv2
import pandas as pd
import matplotlib.pyplot as plt

# Read the first CSV file
try:
    df_dwa = pd.read_csv('data/dwa12-h.csv')
    robot_x_dwa = df_dwa['robot_x'].tolist()
    robot_y_dwa = df_dwa['robot_y'].tolist()
    dwa_data_available = True
except FileNotFoundError:
    dwa_data_available = False

# Read the second CSV file
try:
    df_teb = pd.read_csv('data/teb12-h.csv')
    robot_x_teb = df_teb['robot_x'].tolist()
    robot_y_teb = df_teb['robot_y'].tolist()
    teb_data_available = True
except FileNotFoundError:
    teb_data_available = False

# Read the third CSV file
try:
    df_social = pd.read_csv('data/social12-h.csv')
    robot_x_social = df_social['robot_x'].tolist()
    robot_y_social = df_social['robot_y'].tolist()
    social_data_available = True
except FileNotFoundError:
    social_data_available = False

# Plot the data if available
if dwa_data_available or teb_data_available or social_data_available:
    image = cv2.imread('map/gen_world_16.jpg')
    height, width, _ = image.shape

    plt.figure()
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if dwa_data_available:
        pixel_x_dwa = [x for x in robot_x_dwa]
        pixel_y_dwa = [height - y  for y in robot_y_dwa]
        plt.plot(pixel_x_dwa, pixel_y_dwa, 'r-', label='DWA')
    if teb_data_available:
        pixel_x_teb = [x for x in robot_x_teb]
        pixel_y_teb = [height - y  for y in robot_y_teb]
        plt.plot(pixel_x_teb, pixel_y_teb, 'g-', label='TEB')
    if social_data_available:
        pixel_x_social = [x for x in robot_x_social]
        pixel_y_social = [height - y  for y in robot_y_social]
        plt.plot(pixel_x_social, pixel_y_social, 'b-', label='Social')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Paths')
    plt.grid(True)
    plt.legend()
    plt.show()
else:
    print("No data available to plot.")
