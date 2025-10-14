import matplotlib.pyplot as plt

def plot_columns_from_files(file_paths, columns_info):
    """
    从多个文件中读取指定列并绘制折线图，每列绘制到单独的图片中。
    
    :param file_paths: 文件路径列表
    :param columns_info: 列信息字典，键为列索引（从 1 开始），值为列名称
    """
    for col_index, col_name in columns_info.items():
        plt.figure(figsize=(12, 8))
        
        for file_path in file_paths:
            try:
                # 读取文件内容
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                
                # 提取指定列的数据
                column_data = []
                for line in lines:
                    values = line.split()
                    if len(values) >= col_index:
                        column_data.append(float(values[col_index - 1]))
                
                # 绘制折线图
                plt.plot(column_data, label=file_path.split('/')[-1])
            
            except Exception as e:
                print(f"无法处理文件 {file_path}: {e}")
        
        # 设置图例和标签
        plt.xlabel("行号")
        plt.ylabel(col_name)
        plt.title(f"多个文件的 {col_name} 折线图")
        plt.legend()
        plt.grid()
    plt.show()

def plot_columns_1_to_5(file_paths):
    """
    绘制第 1-5 列的数据，每列绘制到单独的子图中。
    
    :param file_paths: 文件路径列表
    """
    fig, axes = plt.subplots(5, 1, figsize=(12, 20), sharex=True)
    column_names = ["Column 1", "Column 2", "Column 3", "Column 4", "Column 5"]

    for col_index in range(1, 6):
        ax = axes[col_index - 1]
        for file_path in file_paths:
            try:
                # 读取文件内容
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                
                # 提取指定列的数据
                column_data = []
                for line in lines:
                    values = line.split()
                    if len(values) >= col_index:
                        column_data.append(float(values[col_index - 1]))
                
                # 绘制折线图
                ax.plot(column_data, label=file_path.split('/')[-1])
            
            except Exception as e:
                print(f"无法处理文件 {file_path}: {e}")
        
        # 设置子图的图例和标签
        ax.set_ylabel(column_names[col_index - 1])
        ax.set_title(f"{column_names[col_index - 1]} 折线图")
        ax.legend()
        ax.grid()

    # 设置共享的 x 轴标签
    plt.xlabel("行号")
    plt.tight_layout()
    plt.show()

# 指定文件路径列表
file_paths = [  
    "EGVG-FULL.txt",   
    
    # "EGVG-HH-10.txt",   
    "EGVG-HH-7.txt",   
    # "ASTAR2.txt",                     
    # "ASTAR.txt",  
    # "FAR_processed.txt",    
    # "FAEL.txt",

    # "EGVG-NOTOPO.txt",    
    # "GVG-FULL.txt",    
    # "GVG-NOTOPO.txt",
    # "ASTAR2.txt",

    

]

# 指定列信息
columns_info = {
    6: "t_search",
    7: "t_smooth",
    8: "l_smooth",
    9: "l_raw",
}

# 调用函数绘制第 6、7、8、9 列数据
plot_columns_from_files(file_paths, columns_info)

# 调用函数绘制第 1-5 列数据
# plot_columns_1_to_5(file_paths)