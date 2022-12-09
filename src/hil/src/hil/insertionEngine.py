"""Adapted from https://github.com/szabolcsdombi/headless-moderngl-experiment"""
import moderngl as ModernGL
from objloader import Obj
from PIL import Image
from pyrr import Matrix44
import time
import numpy as np
import os
import cv2
import pickle
import imageio
import utm
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
import vedo

# Global shader programs
g_vertex_shader = """
#version 330

uniform mat4 Mvp;

in vec3 in_vert;
in vec3 in_norm;
in vec3 in_text;

out vec3 v_vert;
out vec3 v_norm;
out vec3 v_text;

void main() {
	v_vert = in_vert;
	v_norm = in_norm;
	v_text = in_text;
	gl_Position = Mvp * vec4(v_vert, 1.0);
}
"""

g_fragment_shader = """
#version 330

uniform sampler2D Texture;
uniform vec4 Color;
uniform vec3 Light;

in vec3 v_vert;
in vec3 v_norm;
in vec3 v_text;

out vec4 f_color;

void main() {
	float lum = dot(normalize(v_norm), normalize(v_vert - Light));
	lum = acos(lum) / 3.14159265;
	lum = clamp(lum, 0.0, 1.0);
	lum = lum * lum;
	lum = smoothstep(0.0, 1.0, lum);

	lum *= smoothstep(0.0, 80.0, v_vert.z) * 0.3 + 0.7;

	lum = lum * 0.8 + 0.2;

	vec3 color = texture(Texture, v_text.xy).rgb;
	color = color * (1.0 - Color.a) + Color.rgb * Color.a;
	f_color = vec4(color * lum, 1.0);
}
"""

class insertionEngine:
    """
    Represents a pipeline to insert a single 3D object into an image at a position in 3D world space
    """
    def __init__(self, obj_path: str, texture_path: str, is_online: bool = True) -> None:
        """
        Args: 
            obj_path:  Path to a 3D object file (.obj)
            texture_path: Path to a texture image for the input object
            is_online: True if using this class inside a ros node, False otherwise
        """
        self.obj_path = obj_path
        self.texture_path = texture_path
        self.is_online = is_online

        self.obj = Obj.open(obj_path)
        self.vertex_data = self.obj.pack()
        self.texture_image = Image.open(texture_path)    

        # Perspective Projection Matrix based on zed2 specs from: 
        # https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-
        w, h = 1280, 720
        fov = 72
        self.perspective = Matrix44.perspective_projection(fov, w/h, 0.1, 100.0)

        # Context creation
        self.ctx = ModernGL.create_standalone_context()

        # Shaders
        self.prog = self.ctx.program(
                vertex_shader=g_vertex_shader,
                fragment_shader=g_fragment_shader
        )
        self.prog['Light'].value = (-140.0, -300.0, 350.0)
        self.prog['Color'].value = (1.0, 1.0, 1.0, 0.25)

        # Texture
        self.texture = self.ctx.texture(self.texture_image.size, 3, self.texture_image.tobytes())
        self.texture.build_mipmaps()

        # Vertex Buffer and Vertex Array
        self.vbo = self.ctx.buffer(self.vertex_data)
        self.vao = self.ctx.simple_vertex_array(
            self.prog, self.vbo, * ['in_vert', 'in_text', 'in_norm'])

        # Framebuffers
        self.fbo = self.ctx.framebuffer(
            self.ctx.renderbuffer((w, h)),
            self.ctx.depth_renderbuffer((w, h)),
        )

    def render(self, base_img: np.ndarray, 
                    projection: np.ndarray,
                    world2cam: np.ndarray,
                    model_mat: np.ndarray):
        """ Returns a rendering of the object at the given camera/model poses superimposed on the base image
        Args:
            base_img: input image
        """
        base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
        base_img = Image.fromarray(base_img)

        # Understanding mvp matrix: https://jsantell.com/model-view-projection/
        mvp = projection * world2cam * model_mat
        self.prog['Mvp'].write(mvp.astype('f4').tobytes())

        # Rendering
        self.fbo.use()
        self.ctx.enable(ModernGL.DEPTH_TEST)
        self.ctx.clear(0.9, 0.9, 0.9)
        self.texture.use()
        self.vao.render()
        data = self.fbo.read(components=3, alignment=1)
        img = Image.frombytes('RGB', self.fbo.size, data, 'raw', 'RGB', 0, -1)

        # Placing rendered object pixels onto the input background image
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        mask = img.convert('L').point(fn, mode='1')
        composite = Image.composite(base_img, img, mask)
        res_img = cv2.cvtColor(np.array(composite.convert('RGB')), cv2.COLOR_RGB2BGR)
        return res_img

    def run_offline(self, extracted_bag_dir: str):
        """ Runs object insertion on the given extracted bag directory path"""
        if self.is_online:
            raise ValueError("run_offline() cannot be called when class is initialized with `is_online=True`")
        img_dir = os.path.join(extracted_bag_dir, 'img')
        gps_dir = os.path.join(extracted_bag_dir, 'raw_gps')
        output_dir = os.path.join(extracted_bag_dir, 'insertion_output')
        os.makedirs(output_dir, exist_ok=True)

        n_frames = len([i for i in os.listdir(img_dir)])
        img_paths = [os.path.join(img_dir, f'im_{i}.png') for i in range(n_frames)]
        poses = calculate_poses(gps_dir)
        # This matrix defines the global pose where the obejct is placed in meters
        scale = 0.02
        shift = [0.4, 0.6, 1.8]
        model_mat= np.array([
            [  scale,   0.0,   0.0,    shift[0]],
            [  0.0,     scale, 0.0,    shift[1]],
            [  0.0,     0.0,   scale,  shift[2]],
            [  0.0,     0.0,   0.0,    1.0]
        ]).T    

        permute = np.array([
            [0, 1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        model_mat = permute @ model_mat

        frames = []
        total_render_time = 0
        for i in tqdm(range(n_frames)):
            start_time = time.time()
            img = cv2.imread(img_paths[i])
            world2cam_gl = poses[i].T
            res_img = self.render(img, self.perspective, world2cam_gl, model_mat)
            
            total_render_time += time.time()-start_time
            cv2.imwrite(os.path.join(output_dir,'{:0>3d}.png'.format(i)), res_img)
            frames.append(res_img)

        frames = np.array(frames)
        imageio.mimsave(os.path.join(output_dir, 'video.mp4'),
                        frames[:, :, :, [2,1,0]], fps=30, macro_block_size=1)
        print(f"Average render time: {total_render_time/n_frames}")

def calculate_poses(gps_dir):
    '''
    Calculate world2cam matrix in OpenGL convention
    gps data format:
        cur_gps.latitude, 
        cur_gps.longitude,
        cur_gps.height,
        cur_gps.roll,
        cur_gps.pitch,
        cur_gps.azimuth
    coordinate definition:
        x: east
        y: north
        z: up (the opposite of gravity)
    '''
    n_frames = len([i for i in os.listdir(gps_dir)])
    gps_paths = [os.path.join(gps_dir, 'raw_gps_{}.npy'.format(i)) for i in range(n_frames)]
    gps_data = np.stack([np.load(path) for path in gps_paths])
    lat = gps_data[:, 0]
    lon = gps_data[:, 1]
    height = gps_data[:, 2]
    roll = gps_data[:, 3] # y, right-handed
    pitch = gps_data[:, 4] # x, right-handed 
    azimuth = gps_data[:, 5] # z, left-handed

    utm_data = utm.from_latlon(lat, lon)
    east, north = utm_data[0], utm_data[1]
    
    # normalize positions, center at (0, 0, 0)
    east -= east.mean()
    north -= north.mean()
    height -= height.mean()
    pos = np.stack([east, north, height]).T # position (n, 3)

    euler = np.stack([pitch, roll, azimuth]).T 
    r = R.from_euler('xyz', euler, degrees=True)
    rot = r.as_matrix() # (n, 3, 3) 
    # row-based x: right, y: forward, z: up
    # [OpenGL]  x: right, y: up,      z: backward
    trans = np.array([
        [1, 0, 0], 
        [0, 0, 1], 
        [0,-1, 0]
    ])
    r_gl = trans[None] @ rot

    # visualize_cameras(r_gl, pos, arrow_len=1)
    # quit()

    world2cam_gl = np.zeros((n_frames, 4, 4))
    world2cam_gl[:, -1, -1] = 1
    world2cam_gl[:, :3, :3] = r_gl
    t = r_gl @ -np.expand_dims(pos, -1)
    world2cam_gl[:, :3, -1] = np.squeeze(t, -1)
    return world2cam_gl

def visualize_cameras(
    R,
    pos,
    arrow_len=1,
    s=1
):  
    '''
    get vedo object representing cameras 
    R, T: world2camera transform
    x: green, y: blue, z: red
    '''
    vec_x = R[:, 0]
    vec_y = R[:, 1]
    vec_z = R[:, 2]
    x_end = pos + vec_x * arrow_len
    y_end = pos + vec_y * arrow_len
    z_end = pos + vec_z * arrow_len

    x = vedo.Arrows(pos, x_end, s=s, c='green')
    y = vedo.Arrows(pos, y_end, s=s, c='blue')
    z = vedo.Arrows(pos, z_end, s=s, c='red')
    vedo.show([x, y, z], axes=1)

class insertionEngine2:
    """
    Represents a pipeline to insert a single 3D object into an image at a position in 3D world space
    """
    def __init__(self, obj_path: str, texture_path: str, is_online: bool = True) -> None:
        """
        Args: 
            obj_path:  Path to a 3D object file (.obj)
            texture_path: Path to a texture image for the input object
            is_online: True if using this class inside a ros node, False otherwise
        """
        self.obj_path = obj_path
        self.texture_path = texture_path
        self.is_online = is_online

        self.obj = Obj.open(obj_path)
        self.vertex_data = self.obj.pack()
        self.texture_image = Image.open(texture_path)    

        # Context creation
        self.ctx = ModernGL.create_standalone_context()

        # Shaders
        self.prog = self.ctx.program(
                vertex_shader=g_vertex_shader,
                fragment_shader=g_fragment_shader
        )
        self.prog['Light'].value = (-140.0, -300.0, 350.0)
        self.prog['Color'].value = (1.0, 1.0, 1.0, 0.25)

        # Texture
        self.texture = self.ctx.texture(self.texture_image.size, 3, self.texture_image.tobytes())
        self.texture.build_mipmaps()

        # Vertex Buffer and Vertex Array
        self.vbo = self.ctx.buffer(self.vertex_data)
        self.vao = self.ctx.simple_vertex_array(
            self.prog, self.vbo, * ['in_vert', 'in_text', 'in_norm'])

        # Framebuffers
        self.fbo = self.ctx.framebuffer(
            self.ctx.renderbuffer((1008, 548)),
            self.ctx.depth_renderbuffer((1008, 548)),
        )

    def render(self, base_img: np.ndarray, 
                    projection: np.ndarray,
                    world2cam: np.ndarray,
                    model_mat: np.ndarray):
        """ Returns a rendering of the object at the given camera/model poses superimposed on the base image
        Args:
            base_img: input image
        """
        base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
        base_img = Image.fromarray(base_img)

        # Understanding mvp matrix: https://jsantell.com/model-view-projection/
        mvp = projection * world2cam * model_mat
        self.prog['Mvp'].write(mvp.astype('f4').tobytes())

        # Rendering
        self.fbo.use()
        self.ctx.enable(ModernGL.DEPTH_TEST)
        self.ctx.clear(0.9, 0.9, 0.9)
        self.texture.use()
        self.vao.render()
        data = self.fbo.read(components=3, alignment=1)
        img = Image.frombytes('RGB', self.fbo.size, data, 'raw', 'RGB', 0, -1)

        # Placing rendered object pixels onto the input background image
        thresh = 200
        fn = lambda x : 255 if x > thresh else 0
        mask = img.convert('L').point(fn, mode='1')
        composite = Image.composite(base_img, img, mask)
        res_img = cv2.cvtColor(np.array(composite.convert('RGB')), cv2.COLOR_RGB2BGR)
        return res_img

    def run_offline(self, extracted_bag_dir: str):
        """ Runs object insertion on the given extracted bag directory path"""
        if self.is_online:
            raise ValueError("run_offline() cannot be called when class is initialized with `is_online=True`")
        dir_img = os.path.join(extracted_bag_dir, 'imgs')
        img_paths = sorted([os.path.join(dir_img, file) for file in os.listdir(dir_img)])

        pose_path = os.path.join(extracted_bag_dir, 'poses.pkl')
        camera_info = None
        with open(pose_path, 'rb') as file:
            camera_info = pickle.load(file)
        w, h = camera_info['img_wh']
        K = camera_info['K'] # (3, 3)
        c2w = camera_info['c2w'] # (n, 3, 4), camera to world transform, 
        # original camera model: x: right, y: down, z: forward
        
        output_dir = os.path.join(extracted_bag_dir, 'insertion_output')
        if not os.path.exists(output_dir):
            os.mkdir(output_dir)
        
        # This matrix defines the global pose where the obejct is placed in meters
        f = K[0, 0]
        rad = 2 * np.arctan(h/(2 * f))
        deg = (rad / np.pi) * 180
        proj = Matrix44.perspective_projection(deg, w/h, 0.001, 100)
        
        scale = 0.0005
        shift = [-0.1, 0.08, -0.1]
        model_mat= np.array([
            [  scale,   0.0,   0.0,    shift[0]],
            [  0.0,     scale, 0.0,    shift[1]],
            [  0.0,     0.0,   scale,  shift[2]],
            [  0.0,     0.0,   0.0,    1.0]
        ]).T    

        permute = np.array([
            [0, 0, 1, 0],
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
        model_mat = permute @ model_mat

        frames = []
        total_render_time = 0
        n = len(img_paths)
        for i in range(n):
            start_time = time.time()

            img = cv2.imread(img_paths[i])
            cam2world_cv = c2w[i]
            cam2world_cv[-1, -1] = 1
            world2cam_cv = get_world2cam(cam2world_cv)

            world2cam_gl = cv2gl(world2cam_cv).T
            
            res_img = self.render(img, proj, world2cam_gl, model_mat)
            
            total_render_time += time.time()-start_time
            # print(f"iter {iter}: {time.time()-start_time} seconds")
            cv2.imwrite(os.path.join(output_dir,'{:0>3d}.png'.format(i)), res_img)
            frames.append(res_img)
            # cv2.imshow('res', res_img)
            # cv2.waitKey(5)

        frames = np.array(frames)
        imageio.mimsave(os.path.join(output_dir, 'video.mp4'),
                        frames[:, :, :, [2,1,0]], fps=30, macro_block_size=1)
        print(f"Average render time: {total_render_time/n}")

def get_world2cam(cam2world):
    R = cam2world[:3, :3]
    t = cam2world[:3, -1] # camera position in world coord.
    world2cam = np.zeros((4, 4))
    world2cam[-1, -1] = 1
    world2cam[:3, :3] = R.T 
    world2cam[:3, -1] = R.T @ (-t)
    return world2cam

def cv2gl(trans):
    '''
    Convert transformation matrix from opencv to opengl
    [OpenCV] x: right, y: down, z: forward
    [OpenGL] x: right, y: up,   z: backward
    '''
    convert = np.array([
        [1,  0,  0,  0], 
        [0, -1,  0,  0],
        [0,  0, -1,  0],
        [0,  0,  0,  1]
    ])
    trans_gl = convert @ trans
    return trans_gl

def main():
    import rospkg
    rospack = rospkg.RosPack()
    # extracted_bag_dir = os.path.join(rospack.get_path('hil'), 'data', 'extracted_bags', 'outdoor1')
    # model_path = os.path.join(rospack.get_path('hil'), 'data', 'models')
    extracted_bag_dir = os.path.join('../../', 'data', 'extracted_bags/outdoor1_w_raw_gps_depth')
    model_path = os.path.join('../../', 'data', 'models')

    # extracted_bag_dir = "/home/bhargav/catkin_ws/src/HIL_Research/hil/data/extracted_bags/outdoor1"
    # model_path = "/home/bhargav/catkin_ws/src/HIL_Research/hil/data/models"
    obj_path = os.path.join(model_path, 'dummy.obj')
    texture_path = os.path.join(model_path,'dummy.jpg')

    iEngine = insertionEngine(
        obj_path,
        texture_path,
        is_online=False
    )

    iEngine.run_offline(extracted_bag_dir)
    print('Completed.')

if __name__ == '__main__':
    main()