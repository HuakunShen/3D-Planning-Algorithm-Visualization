import numpy as np
from typing import Tuple, Type


class Map:
    def __init__(self, shape: Tuple[int], dtype: Type = np.int8) -> None:
        self.dtype = dtype
        self.shape = shape
        self.map = np.zeros(self.shape, dtype=self.dtype)

    @property
    def zero_percentage(self) -> float:
        return np.count_nonzero(self.map == 0) / self.map.size

    @property
    def size(self) -> int:
        return self.map.size

    def __str__(self) -> str:
        data = [
            ("Shape", self.map.shape),
            ("dtype", self.dtype),
            ("zero_percentage", self.zero_percentage)
        ]
        return f"{self.__class__.__name__}" + "".join([f"\n\t{d[0]}: {d[1]}" for d in data])


class ProbabilityMap(Map):
    def __init__(self, shape: Tuple[int], probability: float) -> None:
        super().__init__(shape, dtype=np.int8)
        self.probability = probability
        self.reset()

    
    def reset(self) -> None:
        prob_map = np.random.rand(*self.shape)
        self.map[prob_map < self.probability] = 1


if __name__ == "__main__":
    map_ = ProbabilityMap((5, 5), 0.5)
    print(map_)
