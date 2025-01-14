import os
import json
import numpy as np

# 定义函数：计算向量的单位化
def unit_vector(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else np.zeros_like(v)

# 检查 endpoints 相减后的单位向量是否与 vector 相同
def check_vectors(file_path):
    with open(file_path, "r") as f:
        data = json.load(f)  # 加载 JSON 文件内容

    inconsistent_results = []
    for i, entry in enumerate(data):
        # 计算 endpoints 差向量
        endpoints_vector = np.array(entry["endpoints"][0]) - np.array(entry["endpoints"][1])
        # 将差向量单位化
        endpoints_unit = unit_vector(endpoints_vector)
        
        # 获取 vector 数据并单位化
        vector_1 = np.array(entry["vector"][0])
        vector_2 = np.array(entry["vector"][1])
        vector_1_unit = unit_vector(vector_1)
        vector_2_unit = unit_vector(vector_2)
        
        # 比较单位向量是否相同
        is_equal_1 = np.allclose(-endpoints_unit, vector_1_unit, atol=1e-6)  # 设置一个较小的误差范围
        is_equal_2 = np.allclose(endpoints_unit, vector_2_unit, atol=1e-6)  # 第二个向量应相反
        
        # 如果不一致，记录结果
        if not (is_equal_1 and is_equal_2):
            inconsistent_results.append({
                "entry_index": i,
                "endpoints_unit": endpoints_unit.tolist(),
                "vector_1_unit": vector_1_unit.tolist(),
                "vector_2_unit": vector_2_unit.tolist(),
                "is_equal_1": is_equal_1,
                "is_equal_2": is_equal_2
            })
    
    return inconsistent_results

# 文件夹路径
json_dir = "filter_obj_json/json"  # 替换为您的 JSON 文件所在目录

# 获取所有 JSON 文件
json_files = [f for f in os.listdir(json_dir) if f.endswith(".json")]

# 遍历每个 JSON 文件进行操作
for json_file in json_files:
    file_path = os.path.join(json_dir, json_file)
    try:
        # 检查当前 JSON 文件
        results = check_vectors(file_path)

        # 输出结果（仅在不一致时）
        if results:
            print(f"文件: {json_file} 存在不一致情况")
            for result in results:
                print(f"  Entry {result['entry_index']}:")
                print(f"    Endpoints Unit Vector: {result['endpoints_unit']}")
                print(f"    Vector 1 Unit: {result['vector_1_unit']}")
                print(f"    Vector 2 Unit: {result['vector_2_unit']}")
                print(f"    Is Equal to Vector 1: {result['is_equal_1']}")
                print(f"    Is Equal to Vector 2: {result['is_equal_2']}")
            print()
    except Exception as e:
        print(f"错误处理文件 {json_file}: {e}")

print("检查完成！")

