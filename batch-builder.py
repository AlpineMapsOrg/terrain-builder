import argparse
import subprocess
import os

def get_children(tile_x, tile_y):
    children = []
    for i in range(4):
        x = 2 * tile_x + (i % 2)
        y = 2 * tile_y + (i // 2)
        children.append((x, y))
    return children

def build_command(terrainbuilder_path, tile_x, tile_y, zoom, output_dir, force_rebuild, args):
    output_path = os.path.join(output_dir, str(zoom), str(tile_y), f"{tile_x}.glb")
    print(output_path)
    if (not force_rebuild) and os.path.exists(output_path):
        print("\tSkipped")
        return
    
    command = [
        terrainbuilder_path,
        "--tile", str(zoom), str(tile_x), str(tile_y),
        *args,
        "--output", output_path
    ]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        pass

def process_tile(terrainbuilder_path, tile_x, tile_y, zoom, output_dir, args, build_intermediate, force_rebuild):
    if build_intermediate or zoom == args.target_zoom:
        print(f"({zoom}, {tile_x}, {tile_y})")
        build_command(terrainbuilder_path, tile_x, tile_y, zoom, output_dir, force_rebuild, [str(arg) for arg in args.command_args])
        print()

    if zoom < args.target_zoom:
        children = get_children(tile_x, tile_y)
        for child in children:
            process_tile(terrainbuilder_path, child[0], child[1], zoom + 1, output_dir, args, build_intermediate, force_rebuild)

def main():
    parser = argparse.ArgumentParser(description="Calculate and build a command for EPSG 3857 tiles.")
    parser.add_argument("--terrainbuilder", required=True, help="Path to the terrainbuilder executable")
    parser.add_argument("--tile", nargs=3, type=int, required=True, help="EPSG 3857 tile Zoom, X and Y (e.g., 10 1234 5678)")
    parser.add_argument("--target-zoom", type=int, required=True, help="Target zoom level")
    parser.add_argument("--output-dir", required=True, help="Root output directory path")
    parser.add_argument("--build-intermediate", action="store_true", help="Execute for intermediate nodes as well")
    parser.add_argument("--force-rebuild", action="store_true", help="Force rebuild existing tiles")
    parser.add_argument("command_args", nargs=argparse.REMAINDER, help="Command arguments for terrainbuilder")

    args = parser.parse_args()

    terrainbuilder_path = args.terrainbuilder
    root_zoom, tile_x, tile_y = args.tile[0], args.tile[1], args.tile[2]
	
    process_tile(terrainbuilder_path, tile_x, tile_y, root_zoom, args.output_dir, args, args.build_intermediate, args.force_rebuild)

if __name__ == "__main__":
    main()