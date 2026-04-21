# Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
#
# Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
# Seoul National University, under a research agreement with ETRI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from mujoco_bridge import MujocoBridge

def main(args):
    VALID_ROBOT_LIST = ["fr3", "xls", "fr3_xls"]
    if args.robot_name not in VALID_ROBOT_LIST:
        raise ValueError(f"Invalid robot name '{args.robot_name}'. "
                         f"Must be one of {VALID_ROBOT_LIST}.")
    simulator = MujocoBridge(args.robot_name)
    simulator.run()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_name", type=str, default="fr3", help="robot name [fr3, xls, fr3_xls]")

    args = parser.parse_args()
    main(args)