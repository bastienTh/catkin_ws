"""
Some utilities for the training
"""
import torch
import numpy as np
import torchvision
import torchvision.transforms as transforms
import matplotlib.pyplot as plt

def load_mnist_data(kept_labels=[1, 2], batch_size=128):
    """
    Loads a subset of the mnist datase, as defined by the kept_labels.
    """

    # Some transformations will be used
    transform = transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize((0.1,), (0.3,))
        ]
    )
    
    # We load the dataset (train and test sets.)
    trainset = torchvision.datasets.MNIST(root="./data", train=True, download=True, transform=transform)
    testset = torchvision.datasets.MNIST(root="./data", train=False, download=True, transform=transform)
    
    # We restrict the datasets to two classes
    train_idx = [i for i in range(len(trainset)) if trainset.train_labels[i] in kept_labels]
    trainset.data = trainset.data[train_idx]
    trainset.targets = trainset.targets[train_idx]
    test_idx = [i for i in range(len(testset)) if testset.test_labels[i] in kept_labels]
    testset.data = testset.data[test_idx]
    testset.targets = testset.targets[test_idx]
    for i, j in enumerate(kept_labels):
        trainset.targets[trainset.targets == j] = i
        testset.targets[testset.targets == j] = i



    # We create the loaders which will yied batches for training
    train_loader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True, num_workers=2)
    test_loader = torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=True, num_workers=2)
    
    return train_loader, test_loader