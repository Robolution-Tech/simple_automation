from typing import List
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
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        # self.metadata = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0])
        # self.metadata = MetadataCatalog.get("__unused")
        # register_coco_instances(f"baxter_train", {}, f"baxter/train.json", f"baxter/train")
        # register_coco_instances(f"baxter_test", {}, f"baxter/test.json", f"baxter/test")
        # self.cfg.DATASETS.TEST = ("baxter_test",)
        self.cfg.DATALOADER.NUM_WORKERS = 2
        # self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        # self.metadata = MetadataCatalog.get("__unused")
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = self.weights
        # self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.1
        self.predictor = DefaultPredictor(self.cfg)

    # TODO: 
    def inference(self, img, desired_class, preserve_num_of_obj, accept_score):
        assert img is not None, "No image input"
        img_shape = img.shape[:2]
        print(img_shape)
        output = self.predictor(img)
        img_vis = img[:, :, ::-1]
        assert "instances" in output
        instances = output["instances"].to(self.cpu_device)
        pred_classes = instances.pred_classes.tolist()
        if isinstance(desired_class, list):
            desired_class = desired_class[0] # TODO: Currently only support 1 obejct
        if desired_class not in pred_classes:
            return None, None
        else:
            scores = instances.scores.tolist()
            desired_ind = [i for i, j in enumerate(pred_classes) if (j == desired_class and scores[i] > accept_score)]          
            desired_ind = desired_ind[:min(preserve_num_of_obj, len(desired_ind))]  
            mask_output = np.zeros(img_shape)
            for ind in desired_ind:
                current_instance = instances[ind]
                curr_mask = current_instance.pred_masks.numpy()
                curr_mask = curr_mask.squeeze()
                curr_mask_output = np.zeros(img_shape)
                curr_mask_output[curr_mask == 1] = 1
                mask_output += curr_mask_output
            
            mask_output[mask_output >= 1] = 1
            visualizer = Visualizer(img_vis, MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=0.8, instance_mode=ColorMode.IMAGE)
            # print(MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]))
            vis_output = visualizer.draw_instance_predictions(instances)
            rtn = vis_output.get_image()[:, :, ::-1]
            return rtn, mask_output
        # return rtn

