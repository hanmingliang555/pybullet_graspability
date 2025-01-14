import os
import pandas as pd
from fpdf import FPDF

# 设置文件夹路径
root_dir = "output_new_json"
pdf_filename = os.path.join(root_dir, "merged_output.pdf")

# 初始化 PDF 对象
pdf = FPDF()
pdf.set_auto_page_break(auto=True, margin=10)
pdf.set_font("Arial", size=12)

# 布局参数
image_width = 40  # 图片宽度
image_height = 30  # 图片高度
x_left = 10  # 左列起始 x 坐标
x_right = 110  # 右列起始 x 坐标
y_margin = 10  # 上下边距
line_spacing = 5  # 数据行间距
group_height = image_height + 15 + line_spacing * 3  # 每组数据占据的高度

# 遍历每个场景文件夹
for scene_folder in sorted(os.listdir(root_dir)):
    scene_path = os.path.join(root_dir, scene_folder)
    if not os.path.isdir(scene_path):
        continue

    # 遍历每个子文件夹
    for sub_folder in sorted(os.listdir(scene_path)):
        sub_folder_path = os.path.join(scene_path, sub_folder)
        if not os.path.isdir(sub_folder_path):
            continue

        # CSV 文件路径
        csv_filename = os.path.join(sub_folder_path, f"{sub_folder}.csv")
        if not os.path.exists(csv_filename):
            print(f"CSV 文件未找到: {csv_filename}")
            continue

        # 读取 CSV 文件
        df = pd.read_csv(csv_filename)

        # 初始化有效数据计数器
        valid_data_count = 0

        # 遍历 CSV 数据
        for i, row_data in df.iterrows():
            # 提取 `空间角变化(弧度)` 和 `机械臂移动距离`
            angle_change = row_data.get("空间角变化(弧度)", "N/A")
            arm_movement = row_data.get("机械臂移动距离", "N/A")
            
            # 跳过无效数据
            if pd.isna(angle_change) or pd.isna(arm_movement) or angle_change == "N/A" or arm_movement == "N/A":
                continue

            # 格式化数据为四位有效数字
            angle_change = f"{float(angle_change):.4g}"
            arm_movement = f"{float(arm_movement):.4g}"

            # 每 8 组有效数据开启新的一页
            if valid_data_count % 8 == 0:
                pdf.add_page()
                pdf.cell(0, 5, f"Scene: {scene_folder}, obj_id: {sub_folder}", ln=True)

            # 当前组在页面中的 X 和 Y 位置
            if (valid_data_count % 8) % 2 == 0:  # 左列
                x_start = x_left
            else:  # 右列
                x_start = x_right

            y_start = y_margin + ((valid_data_count % 8) // 2) * group_height + 10

            # 插入文字描述
            pdf.set_xy(x_start, y_start)
            pdf.cell(0, 5, f"index: {i}", ln=True)
            pdf.set_xy(x_start, y_start + 5)
            pdf.cell(0, 5, f"angle_change: {angle_change}", ln=True)
            pdf.set_xy(x_start, y_start + 10)
            pdf.cell(0, 5, f"move_distance: {arm_movement}", ln=True)

            # 插入第一张图片
            image_1 = os.path.join(sub_folder_path, f"{sub_folder}_{i}_1.png")
            if os.path.exists(image_1):
                pdf.image(image_1, x=x_start, y=y_start + 20, w=image_width, h=image_height)

            # 插入第二张图片
            image_2 = os.path.join(sub_folder_path, f"{sub_folder}_{i}_2.png")
            if os.path.exists(image_2):
                pdf.image(image_2, x=x_start + image_width + 10, y=y_start + 20, w=image_width, h=image_height)

            # 计数有效数据
            valid_data_count += 1

# 保存 PDF 文件
pdf.output(pdf_filename)
print(f"PDF 文件已保存到: {pdf_filename}")