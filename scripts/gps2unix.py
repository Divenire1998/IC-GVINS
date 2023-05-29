import pandas as pd

# 读取文件a和文件b
df_a = pd.read_csv('trajectory_gps.csv')
df_b = pd.read_csv('trajectory_unix.csv')

# 把文件a的第一列替换为文件b的第一列
df_a.iloc[:, 0] = df_b.iloc[:, 0]

# 保存文件a的更新后的内容
df_a.to_csv('a_updated.csv', index=False)

