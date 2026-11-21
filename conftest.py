"""把仓库根目录加入导入路径，保证 tests 可直接导入 road 包。"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
