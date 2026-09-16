from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.connect import Client
from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import argparse
import yaml



def get_mat4_from_mate_connector(mate_connector):# # #{
    """Find TF in Onshape Tree. Return Translation and Rotation(euler_xyz)."""
        
    # occurrence path this mate connector is attached to
    occurrence_key = cad.get_path_key(mate_connector.occurrence)
    occurrence = cad.occurrences[occurrence_key]

    # part-local -> world/assembly transform
    part_to_world_tf = occurrence.tf

    # mate connector's transform relative to its own part
    part_to_mate_tf = mate_connector.mateConnectorCS.to_tf

    # mate connector's transform in world/assembly coordinates
    world_to_mate_tf = part_to_world_tf @ part_to_mate_tf

    # print(world_to_mate_tf)
    return world_to_mate_tf


# # #}

def get_tf_from_mat4(transform4x4):# # #{
    """Convert a 4x4 transformation matrix to translation and Euler angles."""
    translation = transform4x4[:3, 3]
    rotation = Rotation.from_matrix(transform4x4[:3, :3])
    euler_xyz_rad = rotation.as_euler("xyz", degrees=False)
    euler_xyz_deg = rotation.as_euler("xyz", degrees=True)

    print("    Translation (x, y, z):", translation)
    print("    Euler angles XYZ (radians):", euler_xyz_rad)
    print("    Euler angles XYZ (degrees):", euler_xyz_deg)

    return translation, euler_xyz_deg, euler_xyz_rad

# # #}

def plot_frame(ax, tf, label="frame", axis_length=0.05, colors=("r", "g", "b")):# # #{
    """Plot a coordinate frame given a 4x4 transformation matrix."""
    origin = tf[:3, 3]
    x_axis = tf[:3, 0]
    y_axis = tf[:3, 1]
    z_axis = tf[:3, 2]

    ax.quiver(*origin, *x_axis, length=axis_length, color=colors[0], normalize=True)
    ax.quiver(*origin, *y_axis, length=axis_length, color=colors[1], normalize=True)
    ax.quiver(*origin, *z_axis, length=axis_length, color=colors[2], normalize=True)

    ax.text(*origin, label, fontsize=8)

# # #}

def set_axes_equal(ax):# # #{
    """Force equal aspect ratio for a 3D matplotlib axis."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max(x_range, y_range, z_range)

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# # #}

def get_mate_connectors_with_prefix(cad, prefix: str):# # #{
    """Return all mate connectors whose name starts with the given prefix."""
    return [mc for mc in cad.mate_connectors if mc.name.startswith(prefix)]
# # #}

def get_mate_connectors_with_name(cad, name: str):# # #{
    """Return all mate connectors whose name starts with the given prefix."""
    for mc in cad.mate_connectors:
        if mc.name == name:
            return mc
    return None 
# # #}

def round_list(values, ndigits=4):# # #{
    return [round(float(v), ndigits) for v in values]
# # #}

def parse_args():# # #{
    parser = argparse.ArgumentParser(description="Compute TFs and visualize mate connectors from an Onshape assembly.")
    parser.add_argument("--cad-url", default="https://cad.onshape.com/documents/640e6cf1141dd92e4e3bb681/w/8707da88bc8b3fb629d5ddc6/e/873b4feae19d410382f413a7", help="Onshape CAD document URL.")
    parser.add_argument("--plot", action="store_true", help="Show the 3D matplotlib visualization.")
    parser.add_argument(
        "--output", default="tf.yaml", help="Path to output YAML file with computed transforms (default: tf.yaml)."
    )
    return parser.parse_args()

# # #}

class CompactDumper(yaml.Dumper):# # #{
    """YAML dumper that renders scalar lists (e.g. [x, y, z]) in flow style,
    while keeping dicts in block style."""

    def represent_sequence(self, tag, sequence, flow_style=None):
        return super().represent_sequence(tag, sequence, flow_style=True)
# # #}

if __name__ == "__main__":
    args = parse_args()

    client = Client(env=".env")

    cad = CAD.from_url(
        args.cad_url,
        client=client,
    )


    fcu_name = "_fcu"
    fcu_mate = get_mate_connectors_with_name(cad, fcu_name)

    if fcu_mate is None:
        print(f"No mate connector found with name '{fcu_name}'")
        print("Settting Origin as the FCU frame")
        world_to_fcu = np.eye(4)
    else:
        world_to_fcu = get_mat4_from_mate_connector(fcu_mate)

    mates = get_mate_connectors_with_prefix(cad, "_")
    
    if args.plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        # world/global frame at origin for reference
        plot_frame(ax, np.eye(4), label="world", axis_length=0.1)
        plot_frame(ax, world_to_fcu, label=fcu_name, axis_length=0.05)

    tf_data = {}

    for mate in mates:
        if mate.name == fcu_name:
            continue

        print()
        print(f"Mate Connector: {mate.name}")
        print(f"  TF in {fcu_name} frame:")

        world_to_mate_mat4= get_mat4_from_mate_connector(mate)
        
        # compute the transformation from the FCU frame to the mate connector's frame
        fcu_to_mate_mat4 = np.linalg.inv(world_to_fcu) @ world_to_mate_mat4

        translation, rot_euler, rot_rad = get_tf_from_mat4(fcu_to_mate_mat4)

        tf_data[mate.name] = {
            "translation": round_list(translation.tolist()),
            "rotation_euler_xyz_deg": round_list(rot_euler.tolist()),
            "rotation_euler_xyz_rad": round_list(rot_rad.tolist()),
        }

        if args.plot:
                   # frame_fcu mate connector in world coordinates
            plot_frame(ax, world_to_mate_mat4, label=mate.name, axis_length=0.05)

            # optional: also plot the part's own occurrence frame
            # plot_frame(ax, part_to_world_tf, label="part_occurrence", axis_length=0.05)
   

    with open(args.output, "w") as f:
        yaml.dump(tf_data, f, Dumper=CompactDumper, default_flow_style=False, sort_keys=False)

    print(f"\nSaved transforms for {len(tf_data)} mate connector(s) to {args.output}")

    if args.plot:
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect([1, 1, 1])  # equal aspect ratio
        set_axes_equal(ax)
        plt.show()
