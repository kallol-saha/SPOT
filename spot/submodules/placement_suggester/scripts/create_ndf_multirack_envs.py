import re
import numpy as np
import os.path as osp
from ndf_robot.utils import path_util
import shutil
import os

# Define the new origin coordinates
def get_random_angles():
    return [0, 0, np.random.random()*2*np.pi]

def get_random_pos():
    return [np.random.random()*0.8-0.1, np.random.random()*0.5-0.25,  0]

def is_intersecting(pos1, pos2, thresh=0.33):
    dist = np.linalg.norm(np.array(pos1) - np.array(pos2))
    if dist < thresh:
        return True
    return False

def format_to_str(arr):
    return f"{arr[0]} {arr[1]} {arr[2]}"

def get_all_pairs(num_racks):
    pairs = []
    for i in range(num_racks):
        for j in range(i+1, num_racks):
            pairs.append((i, j))
    return pairs


def main(args):
    thresh = 0.33 if args.num_racks <= 5 else 0.28

    # Read the URDF file
    template_urdf = f"create_multirack_envs_templates/table_rack_{args.num_racks}rack_TEMPLATE.urdf"
    # template_urdf = osp.join(path_util.get_ndf_descriptions(), f"hanging/table/table_rack_{args.num_racks}rack_TEMPLATE.urdf")
    with open(template_urdf, 'r') as file:
        base_urdf_content = file.read()

    folder = osp.join(path_util.get_ndf_descriptions(), f"hanging/table/{args.num_racks}rack_rand")

    # if folder doesn't exist, create it
    if not osp.exists(folder):
        os.mkdir(folder)
    # if folder is empty, do nothing
    elif len(os.listdir(folder)) == 0:
        pass
    # if the folder is not empty, ask for user confirmation with input()
    else:
        response = input(f"\nFolder {folder} is not empty. Overwrite? (y/n): ")
        while response not in ['y', 'n']:
            response = input(f"Folder {folder} is not empty. Overwrite? (y/n): ")
        if  response == 'y':
            shutil.rmtree(folder)
            os.mkdir(folder)
        else:
            exit()

    # set numpy random seed
    # note: this random seed wasn't used for generating the 2 rack and 3 rack cases. just keep that data forever
    np.random.seed(0)

    for i in range(args.num_envs):
        my_urdf_content = base_urdf_content
        rack_rpys = [format_to_str(get_random_angles()) for _ in range(args.num_racks)]

        intersects = True
        while intersects:
            rack_xyzs = [get_random_pos() for _ in range(args.num_racks)]
            intersects = np.any([is_intersecting(rack_xyzs[i], rack_xyzs[j], thresh=thresh) for i, j in get_all_pairs(args.num_racks)])

        rack_xyzs = [format_to_str(pos) for pos in rack_xyzs]

        # Paste the new coordinates in the URDF file
        for j in range(args.num_racks):
            my_urdf_content = re.sub(rf'RACK{j+1}_RPY', rack_rpys[j], my_urdf_content)
            my_urdf_content = re.sub(rf'RACK{j+1}_XYZ', rack_xyzs[j], my_urdf_content)

        # Write the modified URDF content back to a file
        urdf_fname = osp.join(
                folder, f"table_rack_{args.num_racks}rack_{i}.urdf"
            )

        with open(urdf_fname, 'w') as file:
            file.write(my_urdf_content)

    print(f"Created {args.num_envs} random {args.num_racks}-rack environments at {folder}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    # add argument to argparse to choose between 2 racks or 3 racks
    parser.add_argument(
        "--num_envs",
        type=int,
        default=50,
        help="Number of environments to create",
    )
    parser.add_argument(
        "--num_racks",
        type=int,
        help="Number of racks to create per environment",
    )
    args = parser.parse_args()
    main(args)