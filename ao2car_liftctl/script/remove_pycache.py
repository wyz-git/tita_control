import os
import shutil


def remove_pycache(dir_path):
    for root, dirs, files in os.walk(dir_path):
        if "__pycache__" in dirs:
            shutil.rmtree(os.path.join(root, "__pycache__"))
            print(f"Removed: {os.path.join(root, '__pycache__')}")


if __name__ == "__main__":
    remove_pycache(".")
