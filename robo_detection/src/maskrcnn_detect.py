from detectron2 import model_zoo
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import ColorMode, Visualizer
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import register_coco_instances
import os
import torch
import numpy as np


class MaskRcnn:
    def __init__(self, weights):
        self.weights = weights
        self.cpu_device = torch.device("cpu")
        if not os.path.exists(self.weights):
            raise FileExistsError("Not found weights!")
        self.cfg = get_cfg()

        # self.metadata = MetadataCatalog.get("__unused")
        # register_coco_instances(f"baxter_train", {}, f"baxter/train.json", f"baxter/train")
        # register_coco_instances(f"baxter_test", {}, f"baxter/test.json", f"baxter/test")
        # self.cfg.DATASETS.TEST = ("baxter_test",)
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.metadata = MetadataCatalog.get("__unused")
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = self.weights
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.1
        self.predictor = DefaultPredictor(self.cfg)

    def inference(self, img):
        assert img is not None, "No image input"
        output = self.predictor(img)
        img_vis = img[:, :, ::-1]
        assert "instances" in output
        instances = output["instances"].to(self.cpu_device)
        scores = instances.scores.tolist()
        if len(scores) > 0:
            idx = scores.index(max(scores))
            instances = instances[idx]
        mask = instances.pred_masks.numpy()
        mask = mask.squeeze()
        mask_output = np.zeros(mask.shape)
        mask_output[mask == 1] = 1
        visualizer = Visualizer(img_vis, self.metadata, scale=0.8, instance_mode=ColorMode.IMAGE)
        vis_output = visualizer.draw_instance_predictions(instances)
        rtn = vis_output.get_image()[:, :, ::-1]
        return rtn, mask_output

