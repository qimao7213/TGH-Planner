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
           'ray_pts.txt',
           'ray_pts_1.txt',
          #  'ray_pts_scale2.txt',
        #    'ray_pts_scale3.txt',
          ]  

# 绘制多个路径
plot_paths(labels)

