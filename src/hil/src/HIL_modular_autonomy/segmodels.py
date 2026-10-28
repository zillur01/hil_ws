import sys
import os
import numpy as np
import torch
import torch.nn.functional as F

from matplotlib import cm
from detectron2.engine.defaults import DefaultPredictor
from detectron2.config import get_cfg

sys.path.append("SparseInst")
from sparseinst import VisualizationDemo, add_sparse_inst_config

import PIDNet.tools._init_paths
import PIDNet.models.pidnet as pidnet


cmap = cm.jet(np.random.permutation(256))
semseg_cmap = [(128, 64,128),
    (244, 35,232),
    ( 70, 70, 70),
    (102,102,156),
    (190,153,153),
    (153,153,153),
    (250,170, 30),
    (220,220,  0),
    (107,142, 35),
    (152,251,152),
    ( 70,130,180),
    (220, 20, 60),
    (255,  0,  0),
    (  0,  0,142),
    (  0,  0, 70),
    (  0, 60,100),
    (  0, 80,100),
    (  0,  0,230),
    (119, 11, 32)]

RGB_MEAN = torch.tensor([0.485, 0.456, 0.406]).cuda()
RGB_STD = torch.tensor([0.229, 0.224, 0.225]).cuda()


def load_sparseinst_model(cfg_file, wts, conf_thr = 0.5):
    """Loads SparseInst model from config and weights."""
    cfg = get_cfg()
    add_sparse_inst_config(cfg)
    cfg.merge_from_file(cfg_file)
    cfg.MODEL.WEIGHTS = wts
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = conf_thr
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = conf_thr
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = conf_thr
    cfg.freeze()
    return DefaultPredictor(cfg)


def preprocess_rgb_pidnet(rgb):
    """Converts RGB to GPU tensor and performs necessary preprocessing for network input."""
    rgb = torch.FloatTensor(rgb.copy()).cuda()
    rgb /= 255.0
    rgb -= RGB_MEAN
    rgb /= RGB_STD
    rgb = rgb.permute(2, 0, 1).unsqueeze(0)
    return rgb


def load_pidnet_model(pretrained, model_name='pidnet-s'):
    """Loads PIDNet model from file."""
    model = pidnet.get_pred_model(model_name, 19)
    pretrained_dict = torch.load(pretrained, map_location='cpu')
    if 'state_dict' in pretrained_dict:
        pretrained_dict = pretrained_dict['state_dict']
    model_dict = model.state_dict()
    pretrained_dict = {k[6:]: v for k, v in pretrained_dict.items() if (k[6:] in model_dict and v.shape == model_dict[k[6:]].shape)}
    # print('Loaded PIDNet with {} parameters'.format(len(pretrained_dict)))
    model_dict.update(pretrained_dict)
    model.load_state_dict(model_dict, strict = False)
    model.eval()
    return model.cuda()