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