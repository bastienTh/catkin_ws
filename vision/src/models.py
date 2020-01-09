"""
Simple LeNet neural network
"""
import torch.nn as nn
import torch.nn.functional as F
import torch
import numpy as np


import misc
import vis

class LeNet(nn.Module):

    def __init__(self, classes):
        """
        The initializer of the network. The layers used in the network must be
        declared here.
        """

        super(LeNet, self).__init__()
        self.classes = classes

        

        self.conv1 = nn.Conv2d(1, 6, 5)        
        self.conv2 = nn.Conv2d(6, 16, 5)

        self.dense1 = nn.Linear(256, 120)        
        self.dense2 = nn.Linear(120, 84)
        self.dense3 = nn.Linear(84, 2)

    def forward(self, x):
        """
        This method is called on a batch of data to perform a forward pass at training time.
        It links the different layers to form the complete function performed by the network.
        """

        x = F.relu(self.conv1(x))

        x = F.max_pool2d(x, 2)
 
        x = F.relu(self.conv2(x))

        x = F.max_pool2d(x, 2)   

        x = x.view(-1, 256)

        x = F.relu(self.dense1(x))        
        x = F.relu(self.dense2(x))     
        x = F.relu(self.dense3(x))
        return x

    def infer(self, x):
        """
        This method is called to perform a forward pass on a a single image at inference time.
        It must return the most likely label.
        """
        x = np.reshape(x, (1, 1, 28, 28))
        x = torch.tensor(x, dtype=torch.float)
        res = self.forward((x))#torch.from_numpy

        label = res.argmax().item()

        return label + 1


def batch_mean(batch):
    """
    This function takes 3d tensor containing a batch of data as input and computes its mean. Must return a float.
    """
    return batch.mean().item()


def batch_std(batch):
    """
    This function takes a 3d tensor containing a batch of data as input and computes its standard deviation. Must return a float.
    """

    return batch.std().item()
    

def compute_params_count(model):
    """
    This function computes the number of parameters of the input model.
    """

    ##################
    # YOUR CODE HERE # 
    ##################

    return None

if __name__ == "__main__":

    print("1) Loading data:")
    loader, _ = misc.load_mnist_data(kept_labels=[1, 2], batch_size=128)
    images, labels = iter(loader).next()
    print("Batch shape: {}".format(images.shape))
    print("Labels shape: {}".format(labels.shape))
    assert isinstance(batch_mean(images), float)
    assert isinstance(batch_std(images), float)
    print("Batch mean: {}".format(batch_mean(images)))
    print("Batch std: {}".format(batch_std(images)))
    vis.preview_loader(images, labels, "Batch sample")
    vis.show_image(images[0, 0].numpy(), "Mnist sample")
    
    print("Ok\n\n2) Checking the model:")
    model = LeNet([1, 2])
    print(model)
    modules = list(model.modules())
    # assert list(modules[1].parameters())[0].shape == torch.Size([6, 1, 5, 5])
    # assert list(modules[2].parameters())[0].shape == torch.Size([16, 6, 5, 5])
    # assert list(modules[3].parameters())[0].shape == torch.Size([120, 400])
    # assert list(modules[4].parameters())[0].shape == torch.Size([84, 120])
    # assert list(modules[5].parameters())[0].shape == torch.Size([2, 84])
    infer = model.infer(np.random.randn(28, 28))
    assert infer in [1, 2]
    output = model(images)
    _, prediction = torch.max(output.data, 1)
    vis.preview_loader(images, prediction, "Inference sample")
   
    # print("Ok\n\n3) Computing the number of parameters")
    # assert isinstance(compute_params_count(model), int)
    # print("Number of parameters of the model: {}".format(compute_params_count(model)))
    
    print("Ok\n\n4) Previewing first layer kernels")
    vis.preview_kernels(list(model.parameters())[0], "Input kernels")