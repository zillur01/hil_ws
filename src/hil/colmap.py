import os 
import numpy as np
from scipy.spatial.transform import Rotation as R

def parse_line(line):
    tokens = line.strip().split(' ')
    img_name = tokens[-1]
    start, end = 0, 0
    for i, c in enumerate(img_name):
        if c == '_':
            start = i + 1
        if c == '.':
            end = i
    img_id = int(img_name[start:end])
    Q = [float(i) for i in tokens[1:5]]
    T = [float(i) for i in tokens[5:8]]
    w, x, y, z = Q
    r = R.from_quat([x, y, z, w])
    rot = r.as_matrix()
    trans = np.zeros((4, 4))
    trans[-1, -1] = 1
    trans[:3, :3] = rot
    trans[:3, -1] = T
    return img_id, trans

def export_camera_poses():
    dir_colmap = 'data/extracted_bags/outdoor1/colmap/sparse/txt'
    imgs_txt = os.path.join(dir_colmap, 'images.txt')

    dir_poses = 'data/extracted_bags/outdoor1/colmap/poses'
    os.makedirs(dir_poses, exist_ok=True)

    with open(imgs_txt) as file:
        for line in file:
            if 'png' in line:
                img_id, pose = parse_line(line)
                path = os.path.join(dir_poses, 'pose_{}.npy'.format(img_id))
                np.save(path, pose)

def test_pose():
    import vedo
    dir_poses = 'data/extracted_bags/outdoor1/colmap/poses'
    pose_paths = [os.path.join(dir_poses, 'pose_{}.npy'.format(i)) for i, _ in enumerate(os.listdir(dir_poses))]

    points = []
    for path in pose_paths:
        pose = np.load(path)
        R = pose[:3,:3]
        t = pose[:3,-1]
        point = R.T @ t
        points.append(point)
    
    points = np.array(points)
    print(np.mean(points, axis=0))
    points = vedo.Points(points)
    vedo.show(points, axes=1)

if __name__ == '__main__':
    # export_camera_poses()
    test_pose()
