import numpy as np  # type: ignore

from typing import Sized
from attr import attrs, attrib, Factory
from geometry import matrix_2_vector, position_from_matrix, vector_2_matrix


@attrs
class Tool(Sized):
    """Tool class."""

    def __len__(self):
        pass

    matrix = attrib(factory=lambda: np.eye(4), type=np.ndarray)  # type: ignore
    mass = attrib(default=0, type=float)
    cg = attrib(factory=lambda: np.zeros(3), type=np.ndarray)  # type: ignore

    @property
    def position(self):
        """
        Get the position XYZ of the frame.

        :return:
        """
        return position_from_matrix(self.matrix)

    @position.setter
    def position(self, value):
        """

        :param value: Sequence[float]
        :return:
        """
        self.matrix[:-1, -1] = value

    @property
    def vector(self):
        """
        Return the vector representation of the frame as EULER ZYX.

        :return: vectorized frame
        """
        return matrix_2_vector(self.matrix)

    @vector.setter
    def vector(self, value):
        self.matrix = vector_2_matrix(value)
