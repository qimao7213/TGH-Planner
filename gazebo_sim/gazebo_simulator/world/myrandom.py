import random
import numpy as np

# --- 在这里修改参数 ---

# 每个区域内要生成的柱子数量 (总共300根, 150*2)
num_pillars_per_area = 400

# 第一个区域的坐标范围 (底部 100x20 米)
area1_x_range = (2.0, 98.0)  # X 轴从 0 到 100
area1_y_range = (10.0, 30.0)   # Y 轴从 0 到 20

# 第二个区域的坐标范围 (顶部 100x20 米, 上下对称)
area2_x_range = (2.0, 98.0)
area2_y_range = (70.0, 90.0)  # Y 轴从 80 到 100
obstacles = []
min_distance = 2.5

# 输出文件路径
output_file = "generated_pillars.world"

# --- 脚本主逻辑 (无需修改) ---

def generate_pillar_model(model_name, x, y):
    """为单根柱子生成完整的 SDF <model> 代码块"""
    model_template = f"""    <model name='{model_name}'>
      <static>true</static>
      <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
      <link name='link'>
        <pose>0 0 2 0 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.083</iyy>
            <iyz>0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>4</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>4</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Black</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""
    return model_template

def generate_pillars_for_area(area_name, num_pillars, x_range, y_range, file):
    """为指定的区域生成所有柱子的SDF代码并写入文件"""
    for i in range(num_pillars):
        x = random.uniform(x_range[0], x_range[1])
        y = random.uniform(y_range[0], y_range[1])
        if all(np.hypot(x - ox, y - oy) >= min_distance for ox, oy in obstacles):
            obstacles.append((x, y))
            model_name = f"pillar_{area_name}_{i}"
            file.write(generate_pillar_model(model_name, x, y))
            file.write("\n")

if __name__ == "__main__":
    with open(output_file, "w") as file:
        # 写入 world 文件头部
        file.write("<world>\n")
        file.write("  <include>\n")
        file.write("    <uri>model://sun</uri>\n")
        file.write("  </include>\n")
        file.write("  <include>\n")
        file.write("    <uri>model://ground_plane</uri>\n")
        file.write("  </include>\n")
        
        # 生成柱子并写入文件
        generate_pillars_for_area("bottom_area", num_pillars_per_area, area1_x_range, area1_y_range, file)
        generate_pillars_for_area("top_area", num_pillars_per_area, area2_x_range, area2_y_range, file)
        
        # 写入 world 文件尾部
        file.write("</world>\n")