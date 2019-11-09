from abc import ABC, abstractmethod

import numpy as np


class CollisionObject(ABC):
    """
    Abstract class for a parametrically defined collision object.
    """
    @abstractmethod
    def in_collision(self, target):
        """
        Checks whether target point is in collision. Points at the boundary of
        the object are in collision.

        :returns: Boolean indicating target is in collision.
        """
        pass


class CollisionBox(CollisionObject):
    """
    N-dimensional box collision object.
    """
    def __init__(self, location, half_lengths):
        """
        :params location: coordinates of the center
        :params half_lengths: half-lengths of the rectangle along each axis
        """
        self.location = np.asarray(location)
        self.half_lengths = np.asarray(half_lengths)
        self.ndim = self.location.shape[0]

    def in_collision(self, target):
        # FILL in your code here
        dim = self.location.shape

        max_coordinates = self.location + self.half_lengths
        min_coordinates = self.location - self.half_lengths
        count = 0
        for i in range(self.ndim):
            if target[i] >= min_coordinates[i] and target[i] <= max_coordinates[i]:
                count += 1
        if count == self.ndim:
            return True
        else:
            return False


class CollisionSphere(CollisionObject):
    """
    N-dimensional sphere collision object.
    """
    def __init__(self, location, radius):
        """
        :params location: coordinates of the center
        :params radius: radius of the circle
        """
        self.location = np.asarray(location)
        self.radius = radius

    def in_collision(self, target):
        # FILL in your code here
        coll_eqn = 0
        target = np.asarray(target)
        if target.shape != self.location.shape:
            coll_eqn = np.linalg.norm(target.reshape(-1,1) - self.location.reshape(-1))
            if coll_eqn <= self.radius:
                return True
            else:
                return False
        else:
            coll_eqn = np.linalg.norm(target - self.location)
            if coll_eqn <= self.radius:
                return True
            else:
                return False
