#!/usr/bin/env python3

import os
import numpy as np
import argparse
import torch
import torch.nn as nn
from torchvision import transforms
from helper_CSAILVision.models import ModelBuilder, SegmentationModule
from helper_CSAILVision.lib.utils import as_numpy
from helper_CSAILVision.config import cfg

__all__ = ['hand_segmentation', 'module_init']


def img_transform(img):
    normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225])
    img = np.float32(np.array(img)) / 255.0
    img = img.transpose((2, 0, 1))
    img = normalize(torch.from_numpy(img))
    return img


def hand_segmentation(frame, segmentation_module, save=False):
    # Convert to torch.Tensor
    frame_tensor = img_transform(frame)
    frame_tensor = torch.unsqueeze(frame_tensor, 0).cuda()

    # Get sizes
    segSize = (as_numpy(frame).shape[0], as_numpy(frame).shape[1])

    # Forward pass
    pred_tmp = segmentation_module(frame_tensor, segSize=segSize)
    _, pred = torch.max(pred_tmp, dim=1)

    # Convert to numpy.ndarray
    pred = as_numpy(pred.squeeze(0).cpu())

    if save:
        np.savetxt('numpy.txt', pred.astype(int), fmt='%i', delimiter=",")
    return pred


def module_init():

    cfg.merge_from_file(os.path.dirname(os.path.abspath(__file__)) + '/egohands-resnet50dilated-ppm_deepsup.yaml')
    cfg.MODEL.arch_encoder = cfg.MODEL.arch_encoder.lower()
    cfg.MODEL.arch_decoder = cfg.MODEL.arch_decoder.lower()
    cfg.MODEL.weights_encoder = os.path.join(os.path.dirname(os.path.abspath(__file__)) + '/encoder_epoch_30.pth')
    cfg.MODEL.weights_decoder = os.path.join(os.path.dirname(os.path.abspath(__file__)) + '/decoder_epoch_30.pth')

    # Network Builders
    net_encoder = ModelBuilder.build_encoder(arch=cfg.MODEL.arch_encoder, \
                                             fc_dim=cfg.MODEL.fc_dim, \
                                             weights=cfg.MODEL.weights_encoder)
    net_decoder = ModelBuilder.build_decoder(arch=cfg.MODEL.arch_decoder, \
                                             fc_dim=cfg.MODEL.fc_dim, \
                                             num_class=cfg.DATASET.num_class, \
                                             weights=cfg.MODEL.weights_decoder, \
                                             use_softmax=True)
    # NLLLoss
    crit = nn.NLLLoss(ignore_index=-1)
    # Instantiate segmentation module
    segmentation_module = SegmentationModule(net_encoder, net_decoder, crit).cuda()
    # Evaluation mode
    segmentation_module.eval()

    return segmentation_module
