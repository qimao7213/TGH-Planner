import matplotlib.pyplot as plt

def read_data(filename):
    x = []
    y = []
    with open(filename, 'r') as file:
        for line in file:
            data = line.split()
            x.append(float(data[0]))
            y.append(float(data[1]))
    return x, y

def plot_paths(file_labels):
    plt.figure(figsize=(10, 6))
    
    for label in file_labels:
        x, y = read_data(label)
        plt.plot(x, y, marker='o', linestyle='-', label=label)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Paths Plot')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# 文件名列表
labels = [
          'data/path_ctrl_init.txt',
          # 'data/path_ctrl_opt.txt',
        #   'data/path_ctrl_reall.txt', #这个reall的几何形状和path_ctrl_yaw_constr(如果没有perception)是一致的，只是在时间分布上会不同
          # 'data/path_ctrl_mid.txt',
          # 'data/path_ctrl_yaw_constr.txt',
          # 'data/path_ctrl_perception.txt',

           'data/path_geo.txt',
          'data/path_init.txt',
        #   'data/path_opt.txt',
        #   'data/path_reall.txt',
        #   'data/path_sample.txt',
          # 'data/path_yaw_constr.txt',
          ]  

# 绘制多个路径
plot_paths(labels)

