import os
import sys
from etils import epath

cwd = os.getcwd()
sys.path.append(cwd)

import rl.utils.train_utils as utils  # noqa: E402

ENV_NAME = "Crab"
MODEL_DIR = epath.Path(__file__).parents[2] / "models"
LOG_DIR = epath.Path(__file__).parents[2] / "logs"


if __name__ == "__main__":
    utils.setup()
    utils.train("reorient-test", ENV_NAME, MODEL_DIR, LOG_DIR)
