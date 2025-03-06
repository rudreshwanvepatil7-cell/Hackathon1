import os
import sys
from etils import epath

cwd = os.getcwd()
sys.path.append(cwd)

import rl.utils.train_utils as utils  # noqa: E402

ENV_NAME = "Crab"
MODEL_DIR = epath.Path(__file__).parents[1] / "models"
LOG_DIR = epath.Path(__file__).parents[1] / "logs"
RUN_NAME = "reorient-test4"


if __name__ == "__main__":
    # utils.setup()
    utils.test(ENV_NAME, MODEL_DIR / RUN_NAME / "000100270080", LOG_DIR / RUN_NAME)
