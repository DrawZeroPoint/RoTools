"""Kinematic chain module."""
import logging
from abc import abstractmethod
from typing import Sequence, Sized, Union

import attr
import numpy as np  # type: ignore

from rotools.utility.json_encoder import JSONEncoder
from rotools.robot.serial.link import MDHLink, RevoluteMDHLink, POEScrewAxis

# set logging
logger = logging.getLogger(__name__)


@attr.s
class KinematicChain(Sized):
    """
    An assembly of rigid bodies connected by joints.

    Provides constrained (or desired) motion that is the
    mathematical model for a mechanical system.
    """

    def to_json(self):
        """Encode model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    @property  # type: ignore
    @abstractmethod
    def matrix(self):
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

    @matrix.setter  # type: ignore
    @abstractmethod
    def matrix(self, value):
        """
        Set to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def links(self):
        """Get links."""
        raise NotImplementedError

    @property
    def ndof(self):
        """
        Get number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self)

    @property
    def num_parameters(self):
        """Get number of parameters of all links."""
        # noinspection PyProtectedMember
        raise NotImplementedError

    @abstractmethod
    def transforms(self, q=None):
        """
        Generate a sequence of spatial transforms.

        The sequence represents the given position of the kinematic chain.
        :param q: Optional[Sequence[float]] given position of kinematic chain
        :return: sequence of transforms
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def vector(self):
        """
        Get the vector representation of the kinematic chain.

        :return: vectorized kinematic chain
        """
        raise NotImplementedError

    @vector.setter
    def vector(self, value):
        """Set parameters of all links."""
        raise NotImplementedError


def _validate_links(value):
    """

    :param value: Union[Sequence[MDHLink] np.ndarray]
    :return: Sequence[MDHLink]
    """
    if isinstance(value, np.ndarray):
        try:
            value = value.reshape((-1, MDHLink._size))
        except ValueError as e:
            logger.error(str(e))
            raise e

        # FIXME: only assumes revolute joints
        value = [RevoluteMDHLink(*x) for x in value]
    return value


@attr.s
class MDHKinematicChain(KinematicChain):
    """Kinematic Chain of MDH links."""

    _links = attr.ib(type=Union[Sequence[MDHLink], np.ndarray])

    def __attrs_post_init__(self):
        """Post-attrs initialization."""
        self._links = _validate_links(self._links)

    @classmethod
    def from_parameters(cls, parameters):
        """Construct Kinematic Chain from parameters."""
        kc = cls(parameters)
        return kc

    @property
    def matrix(self):
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        return np.array([l.vector for l in self._links])

    @matrix.setter
    def matrix(self, value):
        """
        Set matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        for i, v in enumerate(value):
            self.links[i].vector = v

    @property
    def links(self):
        """Get links."""
        x = self._links  # type: Sequence[MDHLink]
        return x

    @links.setter
    def links(self, value):
        """Set links.

        :param value: Union[Sequence[MDHLink], np.ndarray]
        """
        self._links = _validate_links(value)

    def __len__(self):
        """Get ndof."""
        return len(self._links)

    @property
    def num_parameters(self):
        """Get number of parameters of all links."""
        # noinspection PyProtectedMember
        return len(self) * MDHLink._size

    def transforms(self, q=None):
        """Get sequence of 4x4 transforms."""
        q = np.zeros(len(self)) if q is None else q
        transforms = [link.transform(p) for link, p in zip(self._links, q)]
        return transforms

    @property
    def vector(self):
        """Get parameters of all links."""
        return self.matrix.ravel()

    # noinspection PyMethodOverriding
    @vector.setter
    def vector(self, value):
        """Set parameters of all links."""
        # noinspection PyProtectedMember
        value = np.array(value).reshape((-1, MDHLink._size))
        self.matrix = value


@attr.s
class POEKinematicChain(Sized):
    """Kinematic Chain of POE links."""

    _home_matrix = attr.ib(type=np.ndarray)
    _screw_axes = attr.ib(type=Union[Sequence[POEScrewAxis], np.ndarray])

    def __attrs_post_init__(self):
        """Post-attrs initialization."""
        pass

    @classmethod
    def from_parameters(cls, poe):
        """Construct Kinematic Chain from parameters."""
        assert len(poe) == 2, print('POE params should contain home matrix and screw axes')
        home_matrix = poe[0]
        screw_axes = poe[1]
        kc = cls(home_matrix=home_matrix, screw_axes=screw_axes)
        return kc

    @property
    def home_matrix(self):
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        return self._home_matrix

    @home_matrix.setter
    def home_matrix(self, value):
        """Set home matrix of link parameters."""
        self._home_matrix = value

    @property
    def screw_axes(self):
        """Get screw axes."""
        return self._screw_axes

    @screw_axes.setter
    def screw_axes(self, value):
        """Set screw axes.

        :param value: Union[Sequence[POEScrewAxis], np.ndarray]
        """
        self._screw_axes = value

    def __len__(self):
        """Get dof."""
        return self._screw_axes.shape[-1]
