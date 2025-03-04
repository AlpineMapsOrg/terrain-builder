import argparse
import subprocess
import os
import shutil
from pathlib import Path

def convert_command(terrainconverter_path, input_file, output_file):
    command = [
        terrainconverter_path,
        "--input", input_file,
        "--output", output_file,
        "--verbosity", "info"]
    try:
        subprocess.run(command, check=False)
    except subprocess.CalledProcessError as e:
        print("ERROR")
        pass

def main():
    parser = argparse.ArgumentParser(description="Recursively convert all .tile files in a folder structure to .glb files")
    parser.add_argument("--terrainconverter", required=True, help="Path to the terrainconverter executable")
    parser.add_argument("--dir", required=True, help="The root folder containing the .tile files")
    parser.add_argument("--suffix", help="A suffix that will get inserted like so: <filename><suffix>.glb to the output files")

    args = parser.parse_args()

    terrainconverter_path = args.terrainconverter
    tiles_root_path = args.dir
    suffix = args.suffix

    pathlist = Path(tiles_root_path).glob('**/*.tile')
    for path in pathlist:
        in_file = path
        out_file = path.with_stem(f"{path.stem}{suffix if suffix else ''}").with_suffix(".glb")

        exists = os.path.exists(out_file)

        print(f"{in_file} >>> {out_file}{' SKIP' if exists else ''}")

        if not exists:
            convert_command(terrainconverter_path, in_file, out_file)


if __name__ == "__main__":
    main()