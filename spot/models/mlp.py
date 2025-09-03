import torch.nn as nn


class MLP(nn.Module):
    def __init__(
        self,
        input_size,
        hidden_sizes,
        output_size,
        activation_function=nn.ReLU(),
        dropout_rate=0.0,
    ):
        super(MLP, self).__init__()

        # Create a list to hold the layers
        layers = []

        # Add input to first hidden layer
        prev_size = input_size
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(prev_size, hidden_size))
            layers.append(activation_function)  # Add activation function (default ReLU)
            if dropout_rate > 0.0:
                layers.append(nn.Dropout(dropout_rate))  # Add dropout if specified
            prev_size = hidden_size

        # Add output layer
        layers.append(nn.Linear(prev_size, output_size))

        # Combine layers into a Sequential module
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)
