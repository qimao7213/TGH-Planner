import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  # 用于处理无效值

def main():
    # 指定文件名列表（直接在代码中指定）
    file_names = [
    		#   "type1.txt", 
    		#   "type2.txt",  
    		#   "type3.txt", 
    		#   "type4.txt", 
    		#   "type4-2.txt", 
    		#   "type5.txt", 
            #   "type5-2.txt", 
            #   "type5-3.txt", 
    		  "type6-1.txt",  
              "type6-2.txt", 
              "type6-3.txt",  
              "type6-4.txt",   
              "type6-5.txt",  
              "type6-6.txt",  
            #   "type6-8.txt",                          
    		#   "type7.txt", 
    		  "type8-1.txt",
              "type8-2.txt",
              "type8-3.txt",              
              "type8-4.txt", 
              "type8-5.txt", 
            #   "type8-6.txt", 
            #   "type8-7.txt", 
            #   "type8-8.txt", 
            #   "type8-1.txt", 
    		  ]  

    # 用户自定义无效值的替代值
    invalid_value_replacement = 40  # 将无效值替换为 30（可以修改为任意值）

    # 检查文件名是否为空
    if not file_names:
        print("文件名列表为空，请添加文件名！")
        return

    # 指定要绘制的列：第一列（索引0），第二列（索引1），倒数第二列（索引7），最后一列（索引8）
    columns_to_plot = [0, 1, 2, 7, 8]
    column_titles = {
        0: "sample num",
        1: "DFS path num",
        2: "best path length",
        7: "average path length",
        8: "run time"
    }

    # 用于保存所有文件第1列数据的列表（为了绘制箱形图）
    all_first_column_data = []

    # 用于保存成功率统计的相关数据
    all_sampling_counts = []
    all_success_flags = []

    # 遍历每列并绘制数据
    for col in columns_to_plot:
        plt.figure()  # 每列一个独立图
        title = column_titles.get(col, f"unknown {col}")
        # 遍历文件并绘制每个文件的当前列
        for file_name in file_names:
            try:
                # 读取数据
                data = pd.read_csv(file_name, sep=r"\s+", header=None)
                data = data.head(150)
                # 将无效值 (-1 或小于 0) 替换为用户定义的值
                y_values = data[col].apply(lambda x: invalid_value_replacement if x < -1e-3 else x)

                # 如果是第1列（索引0），将数据存入列表中
                if col == 0:
                    all_first_column_data.append(y_values.tolist())

                # 如果是第2列（索引1），记录成功标志和采样数量
                if col == 1:
                    all_sampling_counts.extend(data.iloc[:, 0].values)  # 采样数量
                    success_flags = (data[col] >= 1).astype(int)  # 成功标志（1 表示成功，0 表示失败）
                    all_success_flags.extend(success_flags)

                # 打印每列的平均值
                avg_value = y_values.mean()
                print(f"文件 {file_name} 列 {title} 的平均值: {avg_value:.4f}")

                # 绘制有效值的曲线
                plt.plot(data.index, y_values, label=file_name)
            except FileNotFoundError:
                print(f"文件 {file_name} 未找到，跳过。")
            except KeyError:
                print(f"文件 {file_name} 中缺少列 {col}，跳过。")

        # 设置图表标题和标签
        plt.title(title)
        plt.xlabel("t")
        plt.ylabel("value")
        plt.legend()  # 显示图例，区分不同文件
        plt.grid(True)  # 添加网格线方便对比

    # 绘制第1列的箱形图
    if all_first_column_data:
        plt.figure()
        plt.boxplot(all_first_column_data, labels=file_names)  # 这里添加文件名标签
        plt.title("Boxplot of First Column (Sample Num) Across All Files")
        plt.ylabel("Sample Num")
        plt.grid(True)

    # 绘制成功率直方图
    if all_sampling_counts and all_success_flags:
        # 定义区间范围，每20个采样为一个区间
        bin_range = 20
        bins = np.arange(0, max(all_sampling_counts) + bin_range, bin_range)

        # 用于存储每个区间的成功率
        success_rates = []

        # 计算每个区间的成功率
        for i in range(len(bins) - 1):
            idx = (np.array(all_sampling_counts) >= bins[i]) & (np.array(all_sampling_counts) < bins[i + 1])
            if np.any(idx):
                success_rate = np.mean(np.array(all_success_flags)[idx])
            else:
                success_rate = 0
            success_rates.append(success_rate)

        # 绘制成功率直方图
        plt.figure(figsize=(10, 6))
        plt.bar(bins[:-1], success_rates, width=bin_range, align='edge', edgecolor='black')
        plt.title('Sampling Count vs Success Rate', fontsize=16)
        plt.xlabel('Sampling Count (per interval)', fontsize=14)
        plt.ylabel('Success Rate', fontsize=14)
        plt.xticks(bins, rotation=45)
        plt.grid(True)
        # 输出bins和success_rates到txt文件
        with open('success_rate_data.txt', 'w') as f:
            for bin_value, success_rate in zip(bins[:-1], success_rates):
                f.write(f"{bin_value}:{bin_value + bin_range}, {success_rate:.4f}\n")
    plt.show()  # 显示所有图表

if __name__ == "__main__":
    main()

