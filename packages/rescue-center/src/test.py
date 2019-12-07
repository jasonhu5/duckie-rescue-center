# TEST CLASS

import yaml
import numpy
from simple_map import SimpleMap

map = SimpleMap("test_map.yaml")
# print(map.map_raw)

test_pos = (0.2,2.8)
map.display_raw_debug()
print map.position_on_map(test_pos, subtile=False)