# Mate Connector TFs Generator

Computes 4x4 transformation matrices for **Onshape Mate Connectors** in your Assembly,
use name conventions `_<sensor_name>` for exported reference frames,
converts them to translation + Euler angles (XYZ, deg/rad) relative to a
chosen reference frame (`_fcu` by default), optionally visualizes them in
3D with matplotlib, and saves the results to a YAML file.

## 1. Setup

### First-time installation

```bash
chmod +x install.sh
./install.sh
```

### Subsequent runs

```bash
source .venv/bin/activate
```

## 2. Onshape API credentials

The script uses an Onshape API `Client`, which requires credentials in a
local `.env` file (not committed to version control):

```bash name=.env
ONSHAPE_ACCESS_KEY=your_access_key
ONSHAPE_SECRET_KEY=your_secret_key
```

Generate API keys from your Onshape account under
**Developer Portal → API Keys**.

## 3. Usage

```bash
python onshapeTFs2yaml.py --cad-url <onshape-assembly-url> [--plot] [--output tf.yaml]
```

### Arguments

| Flag         | Default                                                   | Description                                              |
|--------------|------------------------------------------------------------|------------------------------------------------------------|
| `--cad-url`  | (example assembly URL baked into the script)               | Onshape assembly document URL to analyze.                  |
| `--plot`     | off                                                         | Show an interactive 3D matplotlib visualization of frames. |
| `--output`   | `tf.yaml`                                                   | Path to write the resulting transforms as YAML.             |

### Examples

Run with default assembly URL, no visualization:

```bash
python onshapeTFs2yaml.py
```

Run against a specific assembly and show the 3D plot:

```bash
python onshapeTFs2yaml.py --cad-url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>" --plot
```

Save results to a custom file:

```bash
python onshapeTFs2yaml.py --output my_transforms.yaml
```

## 4. How it works

1. Connects to Onshape via `Client` using credentials from `.env`.
2. Loads the assembly via `CAD.from_url(...)`.
3. Looks for a mate connector named `_fcu` to use as the reference frame.
   - If not found, the **world origin** is used as the reference frame
     instead (with a warning printed).
4. Finds all mate connectors whose name starts with `_` (the naming
   convention used for exported reference frames).
5. For each connector (excluding `_fcu` itself):
   - Computes its transform in world coordinates.
   - Computes its transform **relative to the `_fcu` frame**
     (`fcu_to_mate = inverse(world_to_fcu) @ world_to_mate`).
   - Extracts translation (x, y, z) and Euler angles (XYZ convention, in
     both degrees and radians).
   - Rounds all values to 4 decimal places.
6. Optionally plots every frame (including `world` and `_fcu`) in a 3D
   matplotlib figure with equal-aspect axes, for visual sanity-checking
   against Onshape's own measurement tool.
7. Writes all computed transforms to a YAML file (default `tf.yaml`) in
   compact flow-style lists, e.g.:

```yaml
_livox_back:
  translation: [-0.0262, 0.0629, -0.1157]
  rotation_euler_xyz_deg: [150.0, 0.0, -90.0]
  rotation_euler_xyz_rad: [2.618, 0.0, -1.5708]
```
