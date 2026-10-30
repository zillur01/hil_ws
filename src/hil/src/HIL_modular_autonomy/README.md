# HIL_modular_autonomy
Modular longitudinal-control agent for hardware-in-loop project.

## Setup
### Installation
- Install [PyTorch](https://pytorch.org/get-started/locally/).
- Install [detectron2](https://detectron2.readthedocs.io/en/latest/tutorials/install.html). Hopefully the command `python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'` works for that.
- Run `pip install -r requirements.txt`.

### Downloading Pretrained Weights
- Download [PIDNet weights](https://drive.google.com/uc?export=download&id=1VcF3NXLQvz2qE3LXttpxWQSdxTbATslO) and put them in `PIDNet/pretrained_models/cityscapes/`.
- Download [SparseInst weights](https://drive.google.com/uc?export=download&id=1clYPdCNrDNZLbmlAEJ7wjsrOLn1igOpT) and put them in `SparseInst/`.
