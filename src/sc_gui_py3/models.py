import numpy as np
import torch
import torch.nn as nn
import torch.utils.data as Data
from torchvision import datasets, transforms, models
import cv2
import matplotlib.pyplot as plt
import time

def cvd_model(trainable=True):
    model = models.vgg16(pretrained=True)
    for parma in model.parameters():
        parma.requires_grad = False
    model.classifier = torch.nn.Sequential(torch.nn.Linear(25088, 4096),
                                           torch.nn.ReLU(),
                                           torch.nn.Dropout(p=0.5),
                                           torch.nn.Linear(4096, 4096),
                                           torch.nn.ReLU(),
                                           torch.nn.Dropout(p=0.5),
                                           torch.nn.Linear(4096, 2))
    for index, parma in enumerate(model.classifier.parameters()):
        if index == 6:
            parma.requires_grad = trainable
    return model

class cvd_detector(object):
    def __init__(self, model_path, img_width=224, img_height=224):
        model = cvd_model()
        model.load_state_dict(torch.load(model_path))
        if torch.cuda.is_available():
            model = model.cuda()
        model = model.eval()
        self.model = model
        self.width = img_width
        self.height = img_height
        self.names = {0: "cat", 1: "dog"}

    def detect(self, patchs):
        input = self.preprocess(patchs)
        output = self.model(input)
        pred = torch.max(output, 1)[1].cpu().numpy()
        output = []
        for p in pred:
            output.append(self.names[p])
        return output

    def preprocess(self, patchs):
        for p in range(len(patchs)):
            patchs[p] = cv2.resize(patchs[p], (self.width, self.height))
        patchs = np.array(patchs, dtype=np.float32)
        if len(patchs.shape) < 4:
            patchs = np.expand_dims(patchs, axis=0)/255.0
        else:
            patchs = patchs/255.0
        tensor = torch.Tensor(patchs).permute(0, 3, 1, 2)
        if torch.cuda.is_available():
            tensor = tensor.cuda()
        return tensor


class handy_digit_classification_model(nn.Module):
    def __init__(self):
        super(handy_digit_classification_model, self).__init__()
        self.conv1 = nn.Sequential(nn.Conv2d(in_channels=3, out_channels=64, kernel_size=5, stride=1, padding=2),
                                   nn.BatchNorm2d(64),
                                   nn.ReLU(),
                                   nn.Conv2d(64, 64, 5, 1, 2),
                                   nn.BatchNorm2d(64),
                                   nn.ReLU(),
                                   nn.MaxPool2d(2))
        self.conv2 = nn.Sequential(nn.Conv2d(in_channels=64, out_channels=128, kernel_size=5, stride=1, padding=2),
                                   nn.BatchNorm2d(128),
                                   nn.ReLU(),
                                   nn.Conv2d(128, 128, 5, 1, 2),
                                   nn.BatchNorm2d(128),
                                   nn.ReLU(),
                                   nn.MaxPool2d(2))
        self.bottleneck = nn.Sequential(nn.Conv2d(128, 16, kernel_size=1, stride=1, padding=0),
                                        nn.BatchNorm2d(16),
                                        nn.ReLU())
        self.out = nn.Sequential(nn.Dropout(0.2),
                                 nn.Linear(16*16*16, 64),
                                 nn.Linear(64, 10))
    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        x = self.bottleneck(x)
        x = x.view(x.size(0), -1)
        output = self.out(x)
        return output


class handy_digit_detector(object):
    def __init__(self, model_path, img_width=64, img_height=64):
        model = handy_digit_classification_model()
        model.load_state_dict(torch.load(model_path))
        if torch.cuda.is_available():
            model = model.cuda()
        model = model.eval()
        self.model = model
        self.width = img_width
        self.height = img_height

    def detect(self, patchs):
        input = self.preprocess(patchs)
        output = self.model(input)
        pred = torch.max(output, 1)[1].cpu().numpy()
        return pred

    def preprocess(self, patchs):
        for p in range(len(patchs)):
            patchs[p] = cv2.resize(patchs[p], (self.width, self.height))
        patchs = np.array(patchs, dtype=np.float32)
        if len(patchs.shape) < 4:
            patchs = np.expand_dims(patchs, axis=0)/255.0
        else:
            patchs = patchs/255.0
        tensor = torch.Tensor(patchs).permute(0, 3, 1, 2)
        if torch.cuda.is_available():
            tensor = tensor.cuda()
        return tensor

class printed_digit_classification_model(nn.Module):
    def __init__(self):
        super(printed_digit_classification_model, self).__init__()
        self.conv1 = nn.Sequential(nn.Conv2d(in_channels=3, out_channels=16, kernel_size=5, stride=1, padding=2),
                                   nn.BatchNorm2d(16),
                                   nn.ReLU(),
                                   nn.MaxPool2d(2),
                                   nn.Conv2d(16, 32, 5, 1, 2),
                                   nn.BatchNorm2d(32),
                                   nn.ReLU(),
                                   nn.MaxPool2d(2))
        self.bottleneck = nn.Sequential(nn.Conv2d(32, 16, kernel_size=1, stride=1, padding=0),
                                        nn.BatchNorm2d(16),
                                        nn.ReLU())
        self.out = nn.Sequential(nn.Dropout(0.2),
                                 nn.Linear(16*16*16, 64),
                                 nn.Linear(64, 10))

    def forward(self, x):
        x = self.conv1(x)
        x = self.bottleneck(x)
        x = x.view(x.size(0), -1)
        output = self.out(x)
        return output


class printed_digit_detector(object):
    def __init__(self, model_path, img_width=64, img_height=64):
        model = printed_digit_classification_model()
        model.load_state_dict(torch.load(model_path))
        if torch.cuda.is_available():
            model = model.cuda()
        model = model.eval()
        self.model = model
        self.width = img_width
        self.height = img_height

    def detect(self, patchs):
        input = self.preprocess(patchs)
        output = self.model(input)
        pred = torch.max(output, 1)[1].cpu().numpy()
        return pred

    def preprocess(self, patchs):
        for p in range(len(patchs)):
            patchs[p] = cv2.resize(patchs[p], (self.width, self.height))
        patchs = np.array(patchs, dtype=np.float32)
        if len(patchs.shape) < 4:
            patchs = np.expand_dims(patchs, axis=0)/255.0
        else:
            patchs = patchs/255.0
        tensor = torch.Tensor(patchs).permute(0, 3, 1, 2)
        if torch.cuda.is_available():
            tensor = tensor.cuda()
        return tensor
