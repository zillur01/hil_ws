import torch
import torch.nn.functional as F
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import utm

from segmodels import load_sparseinst_model, load_pidnet_model, preprocess_rgb_pidnet, cmap, semseg_cmap
from matching import data_association
from detectron2.data.detection_utils import read_image


new_origin_east, new_origin_north = 394702.00, 4438802.00


def lla_to_xza(lat, lon, ang):
    xza = np.zeros(3)
    east, north, _, _ = utm.from_latlon(lat, lon)
    xza[0] = east - new_origin_east
    xza[1] = north - new_origin_north
    xza[2] = ang * np.pi / 180.
    return xza
    
def local_to_global(pos, xza):
    x, z, a = xza
    rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    gpos = rot @ pos
    gpos[0] += x
    gpos[1] += z
    return gpos

def global_to_local(pos, xza):
    x, z, a = xza
    lpos = pos[:2].copy()
    lpos[0] -= x
    lpos[1] -= z
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    lpos = rot @ lpos
    return lpos

def dist2d(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5


class Tracker:
    def __init__(self, obs, trk_id, t, mask=None):
        self.pos = obs
        self.trk_id = trk_id
        self.min_dt = 0.1      
        self.n_pts_extrap = 4
        self.frames_since_update = 0
        self.hits = 0
        self.vel = np.zeros(2)
        self.mask = mask
        self.traj = [obs]
        self.traj_global = [obs]
        self.ts = [t]

    def update(self, obs, t, mask=None):
        self.pos = obs
        self.frames_since_update = 0
        self.hits += 1
        self.mask = mask
        self.traj_global.append(obs)
        self.ts.append(t)
        if len(self.ts) >= self.n_pts_extrap:
            dt = self.ts[-1] - self.ts[-self.n_pts_extrap]
            if dt > self.min_dt:
                #self.vel = (self.traj_global[-1] - self.traj_global[-5]) / dt
                self.vel = np.polyfit(np.array(self.ts[-self.n_pts_extrap:] - self.ts[-1]), 
                                      self.traj_global[-self.n_pts_extrap:], deg=1)[0]
        #print('vel', self.vel, dist2d(self.vel, np.zeros(2)))
            
    def predict_pos(self, t):
        return self.pos + self.vel * (t - self.ts[-1])
        
        
class ModularAgent:
    """Modular autonomous longitudinal-control agent."""
    cpu_device = torch.device('cpu')
    
    def __init__(self):
        self.xzas = []
        self.ts = []
        self.speed = 0
        self.trackers = []
        self.static_xzs = []
        self.ID_count = 0
        
        # Segmentation models
        self.semseg = load_pidnet_model('PIDNet/pretrained_models/cityscapes/PIDNet_S_Cityscapes_test.pt')
        self.instseg = load_sparseinst_model('SparseInst/configs/sparse_inst_r50vd_dcn_giam_aug.yaml', 
                                             'SparseInst/sparse_inst_r50vd_dcn_giam_aug_67dc06.pth')
        
        # ROI crop
        self.i1, self.i2 = 0, 720
        self.j1, self.j2 = 0, 1280
        
        # ZED intrinsics
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.intrinsic.set_intrinsics(self.j2 - self.j1, self.i2 - self.i1, 
                                      527.5779418945312, 527.5779418945312, 
                                      616.2459716796875 - self.j1, 359.2155456542969 - self.i1)
        self.K = np.array(
            [[527.5779418945312, 0, 616.2459716796875 - self.j1],
             [0, 527.5779418945312, 359.2155456542969 - self.i1],
             [0, 0, 1]]
        )
        
        # Custom coordinate system
        self.new_origin_east, self.new_origin_north = 394702.00, 4438802.00
        
        # Tracking parameters
        self.min_hits = 3
        self.max_age = 5
        
        # Static obstacle parameters
        self.max_z_static = 8.
        self.max_x_static = 3.
        
        # Prediction parameters
        self.horizon = 10.  # lookahead time horizon (s)
        self.col_rad = 3.   # collision radius (m)
        
        # IDM parameters
        self.idm_v0 = 2.    # desired velocity (m/s)
        self.idm_s0 = 3.    # minimum spacing (m)
        self.idm_T = 0.5    # desired time headway (s)
        self.idm_a = 1.5    # acceleration (m/s^2)
        self.idm_b = 0.5    # comfortable braking deceleration (m/s^2)
        self.idm_delta = 4  # velocity ratio exponent (keep at 4)
        
        self.acc_limit = 9999  # hard limit on accel/decel
        self.out_freq = 10.    # output Hz (?)
        
        
    def get_frame_dets(self, im, dm, classes=(0, 2), conf_thr=0.5):
        """Get relevant instance detections for one frame. Also returns 3D point cloud."""
        dets = []
        preds = self.instseg(im)
        if "instances" in preds:
            instances = preds["instances"][preds["instances"].scores > conf_thr]
            instances = instances.to(self.cpu_device)

            d = o3d.geometry.Image(dm)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(d, self.intrinsic)
            pts = np.asarray(pcd.points)
            
            for i in range(len(instances)):
                pred_mask = instances.pred_masks[i].numpy()
                pred_cls = instances.pred_classes[i].numpy()
                pred_pts = pts[pred_mask.reshape(im.shape[0] * im.shape[1])]
                if len(pred_pts) > 100:  # sufficient pixel area
                    pos = np.median(pred_pts[:, [0, 2]], axis=0)
                    if pos[1] < 50 and pred_cls in classes:  # 50 meter limit
                        dets.append({'mask': pred_mask, 'pos': pos, 'cls': pred_cls})
        return dets, pts

    def update(self, im, dm, t, xza, spd=None, vis=False):
        self.xzas.append(xza)
        self.ts.append(t)
        
        # Ego speed estimation
        if spd is not None:
            self.speed = spd
        elif len(self.xzas) >= 5:  # estimate our own speed if needed
            #print(np.array(self.xzas[-5:]), np.array(self.ts[-5:] - self.ts[-1]))
            self.speed = dist2d(np.zeros(2), np.polyfit(np.array(self.ts[-5:] - self.ts[-1]), 
                                      np.array(self.xzas[-5:])[:, :2], deg=1)[0])
            
        # Dynamic obstacle detection+tracking
        # ---------------------------------------------------------------
        im = im[self.i1:self.i2, self.j1:self.j2]
        dm = dm[self.i1:self.i2, self.j1:self.j2]
        dm[np.isnan(dm)] = float('inf')  # zed depth has two types of invalid: nan = bad stereo, inf = out of range
        frame_dets, pts = self.get_frame_dets(im, dm)
        
        # Prediction for matching
        trks_pos = []
        for tmp_tracker in self.trackers:
            tmp_tracker.frames_since_update += 1
            
            # Only predict if enough hits
            if tmp_tracker.hits >= self.min_hits: 
                predicted = tmp_tracker.predict_pos(t)
            else:
                predicted = tmp_tracker.pos
            trks_pos.append(predicted)
        # print(trks_pos)
        dets_pos = [local_to_global(det['pos'], xza) for det in frame_dets]
        
        # Matching
        matched, unmatched_dets, unmatched_trks = data_association(dets_pos, trks_pos, threshold=-1.)
        
        # Observation update
        for i, trk in enumerate(self.trackers):
            if i not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == i)[0], 0] # a list of single index
                assert len(d) == 1, 'conflicting matches'

                # update statistics
                trk.update(dets_pos[d[0]], t, frame_dets[d[0]]['mask']) 
                
        # Birth
        for i in unmatched_dets: # a scalar of index of detection
            trk = Tracker(dets_pos[i], self.ID_count, t, mask=frame_dets[i]['mask'])
            self.trackers.append(trk)
            self.ID_count += 1
            
        # Death
        num_trks = len(self.trackers)
        for trk in reversed(self.trackers):
            num_trks -= 1
            # remove dead tracker if inactive for too long
            if (trk.frames_since_update >= self.max_age):
                dead = self.trackers.pop(num_trks)
                if vis and dead.hits >= min_hits:
                    plt.plot([x[0] for x in dead.traj_global], [x[1] for x in dead.traj_global], 
                             marker='.', label='id %d' % dead.trk_id)
                    
        # Turn on for instseg visualization
#         vis = im.copy() / 255.
#         for trk in self.trackers:
#             if trk.hits >= self.min_hits:
#                 trk_color = tuple(cmap[trk.trk_id % 256][:3])
#                 vis[trk.mask] = (vis[trk.mask] + trk_color)/2
#         plt.imshow(vis)
#         plt.show()


        # Static obstacle detection
        # ---------------------------------------------------------------
        with torch.no_grad():
            rgb_input = preprocess_rgb_pidnet(im)
            pred = self.semseg(rgb_input)
            pred = F.interpolate(pred, (im.shape[0], im.shape[1]), mode='bilinear', align_corners=True)
            conf, pred = torch.max(torch.softmax(pred, dim=1), dim=1)
            #pred = torch.argmax(pred, dim=1)
            #conf = conf.squeeze(0).cpu().numpy().reshape(HEIGHT * WIDTH)
            pred[0, -100:] = 0  # exclude bottom of image
            pred2 = pred.squeeze(0).cpu().numpy()
            pred = pred.squeeze(0).cpu().numpy().reshape(im.shape[0] * im.shape[1])
            
        #high_conf = conf > 0.8
        is_road = pred == 0
        is_other_ignore = (pred == 8) | (pred == 9) | (pred == 2)
        
        # Filter out points of ignore-classes and points that are obviously too far
        static_pts = pts[(~is_road) & (~is_other_ignore) & 
                         (pts[:, 2] < self.max_z_static) & (np.abs(pts[:, 0]) < self.max_x_static)]
        self.static_xzs = static_pts[:, [0, 2]]
        # print(self.static_xzs.shape, self.static_xzs[0])
        
        # Turn on for semseg visualization
        # sv_img = np.zeros_like(im).astype(np.uint8)
        # for i, color in enumerate(semseg_cmap):
        #     for j in range(3):
        #         sv_img[:,:,j][pred2==i] = semseg_cmap[i][j]
        # plt.imshow(sv_img)
        # plt.show()
        
        
    def act(self, future_traj):
        if len(self.xzas) < 5:
            return self.speed, 0
        
        # Collision distance
        pred_dist = float('inf')
        collision_detected = False
        
        # Predict up to horizon ahead
        dts = np.arange(0.2, self.horizon, 0.2)
        for dt in dts:
            # Predict ego pos along path
            total_dist = self.speed * dt
            remaining_dist = total_dist
            ego_xz = self.xzas[-1]
            for xz in future_traj:
                edge_dist = dist2d(xz, ego_xz)
                if edge_dist > remaining_dist:
                    ego_xz = ego_xz + (xz - ego_xz) * (remaining_dist / edge_dist)
                    break
                else:
                    remaining_dist -= edge_dist
                    ego_xz = xz
             
            # Dynamic obstacles
            for tracker in self.trackers:
                # Only predict if enough hits
                if tracker.hits >= self.min_hits:  
                    predicted = tracker.predict_pos(self.ts[-1] + dt)
                    pred_dist = dist2d(predicted, ego_xz)
                    
                    # Collision predicted
                    if pred_dist < self.col_rad:
                        collision_detected = True
                        break
                        
            if collision_detected:
                break
                        
            # Static obstacles
            if len(self.static_xzs) > 0:
                local_ego_xz = global_to_local(ego_xz, self.xzas[-1])
                pred_dists = (self.static_xzs[:, 0] - local_ego_xz[0])**2 + (self.static_xzs[:, 1] - local_ego_xz[1])**2
                pred_dist = np.min(pred_dists)
                if pred_dist < self.col_rad**2:
                    collision_detected = True
                    break
            
            
        # Compute IDM acceleration equation
        sa = total_dist + pred_dist
        va = dva = self.speed
        sstar = self.idm_s0 + va * self.idm_T + (va * dva) / (2 * (self.idm_a * self.idm_b)**0.5)
        accel = self.idm_a * (1 - (va / self.idm_v0)**self.idm_delta - (sstar / sa)**2)
        # print('col dist', sa, ', ego spd', va, ', desired dist', sstar)
        desired_accel = np.clip(accel, -self.acc_limit, self.acc_limit)
        desired_speed = np.clip(self.speed + accel/self.out_freq, 0, self.idm_v0 * 2)
        return desired_speed, desired_accel
    
    
if __name__ == '__main__':
    low, high = 800, 940
    imgs = [read_image("composited_rgb/im_%05d.png" % i, format="RGB") for i in range(low, high)]
    depths = [np.load('composited_depth/dm_%05d.npy' % i) for i in range(low, high)]

    root = "raw_gps"
    n = high - low
    ts = np.zeros(n)
    spds = np.zeros(n)
    xzas = np.zeros((n, 3))

    for i in range(n):
        lat, lon, h, roll, pitch, azimuth, timestamp = np.load(os.path.join(root, 'raw_gps_%d.npy' % (i + low)))
        ts[i] = timestamp
        xzas[i] = lla_to_xza(lat, lon, azimuth)
        if i >= 5:
            spds[i] = dist2d(xzas[i, :2], xzas[i - 5, :2]) / (ts[i] - ts[i-5])


    # Initialize the agent
    agent = ModularAgent()

    for i in range(n):
        # If we want to measure runtime, start at frame 2 because first takes longer due to gpu mem alloc
        if i == 2:  
            start = time.perf_counter()

        # At each step, update and act
        agent.update(imgs[i], depths[i], ts[i], xzas[i], spds[i])
        dspd, dacc = agent.act(xzas[i+1:])
        print('frame %d | desired spd: %.4f, desired acc: %.4f' % (i, dspd, dacc))

    avg_runtime = (time.perf_counter() - start)/(n - 2)
    print(avg_runtime, 's ==>', 1/avg_runtime, 'fps')