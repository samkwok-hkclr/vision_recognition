import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

market_path = Path(get_package_share_directory("vision_recognition"))
sub_dirs = [
    "",

    "SensingAlgoBase",
    "SensingAlgoBase/multicamregistration",
    "SensingAlgoBase/sensingalgobase",

    "Market-vision-restocking_dep", 
    "Market-vision-restocking_dep/MarketRestockingDep", 
]

for sub_dir in sub_dirs:
    dir_path = market_path / sub_dir
    if dir_path not in sys.path:
        sys.path.append(str(dir_path))
