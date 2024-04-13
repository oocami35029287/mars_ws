import cv2
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv('data/assessment_data.csv')

# 提取robot_x和robot_y列的数据
robot_x = df['robot_x'].tolist()
robot_y = df['robot_y'].tolist()

# 读取底图
image = cv2.imread('map/gen_world_11.jpg')

# 获取底图的大小
height, width, _ = image.shape

# 缩放因子，用于将坐标映射到像素坐标


# 将原始坐标映射到像素坐标
pixel_x = [x for x in robot_x]
# 将y轴方向调整为图像中的y轴方向相反
pixel_y = [height - y  for y in robot_y]

# 绘制图形
plt.figure()
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.plot(pixel_x, pixel_y, 'r-')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path on Image')
plt.grid(True)
plt.show()
