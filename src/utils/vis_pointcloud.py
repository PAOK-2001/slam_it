from open3d import *
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-fp", "--FilePath", help = "Specify file path for plotting point cloud")
    parser.add_argument("-ft", "--FileType", default="ply", help = "Specify file type")
    arguments = parser.parse_args()
    
    file_path = arguments.FilePath
    file_type = arguments.FileType

    print(f"PLotting from {file_path}")
    if file_type == "ply":
        pointcloud = io.read_point_cloud(file_path)
        visualization.draw_geometries([pointcloud])

    elif file_type == "obj":
        raise notImplemented
    
    else:
        return -1