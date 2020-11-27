"""Class for ICO learning node
"""
import random

class ICO():
    def __init__(self, lr:float = 0.01, weight_predic:float = random.random()):
        self.weight_reflex = 1.0
        self.weight_predic = random.random()
        self.x_reflex = 0.0
        self.x_predic = 0.0
        self.lr = lr


    def run_and_learn(self, x_reflex:float, x_predic:float) -> float:
        """Propagates through the network and updates the weight.

        Args:
            x_reflex (float): The reflex signal input
            x_predic (float): The predictive signal input

        Returns:
            output (float): The propagated value
        """
        output = self.__forward_prop()
        self.__update_weight(x_reflex)

        # Updates input signals 
        self.x_reflex = x_reflex
        self.x_predic = x_predic
        
        return output


    def reset_network(self):
        """Resets the weights and signals back to default values
        """
        self.weight_reflex = 1.0
        self.weight_predic = random.random()
        self.x_reflex = 0
        self.x_predic = 0 


    def __forward_prop(self, x_reflex:float, x_predic:float) -> float:
        """Propagates through the network with the learned weights

        Args:
            x_reflex (float): The reflex signal input
            x_predic (float): The predictive signal input

        Returns:
            y (float): The propagated value
        """
        y = self.weight_reflex * x_reflex + self.weight_predic * x_predic

        return y


    def __update_weight(self, x_reflex_new:float):
        """Updates the weights for the predictive signals.
        The network only learns when the reflex signal is on

        Args:
            x_reflex_new (float): The new reflex signal as input
        """
        self.weight_predic = self.lr * self.x_predic * (x_reflex_new - self.x_reflex)

