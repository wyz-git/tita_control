import subprocess

# 定义要执行的命令
command = "rm -rf log/*"

# 使用 subprocess.run 执行命令
try:
    subprocess.run(command, shell=True, check=True)
    print("Logs deleted successfully.")
except subprocess.CalledProcessError as e:
    print(f"An error occurred: {e}")
