import argparse
import subprocess
import os
import shutil

def get_children(tile_x, tile_y):
    children = []
    for i in range(4):
        x = 2 * tile_x + (i % 2)
        y = 2 * tile_y + (i // 2)
        children.append((x, y))
    return children

def build_command(terrainmerger_path, tile_x, tile_y, zoom, children_to_merge, output_dir, force_rebuild, args):
    output_path = os.path.join(output_dir, str(zoom), str(tile_y), f"{tile_x}.tile")
    if (not force_rebuild) and os.path.exists(output_path):
        print("\tSkipped")
        return
    
    command = [
        terrainmerger_path,
        "--input"
    ] + children_to_merge + ["--output", output_path, "--verbosity", "trace"]
    print(command)
    try:
        subprocess.run(command, check=False)
    except subprocess.CalledProcessError as e:
        print("ERROR")
        pass

def process_tile(terrainmerger_path, tile_x, tile_y, zoom, output_dir, force_rebuild, args):
    indent_level = zoom - args.root_tile[0] + 1
    parent_indent = "  " * (indent_level - 1)
    indent = "  " * indent_level
    
    if zoom == args.max_zoom:
        # If we are at the max zoom level, just attempt to copy the mesh tile to the output folder
        input_path = os.path.join(args.input_dir, str(zoom), str(tile_y), f"{tile_x}.tile")
        output_path = os.path.join(output_dir, str(zoom), str(tile_y), f"{tile_x}.tile")
        print(f"{parent_indent}[{zoom},{tile_x},{tile_y}]", end="")
        if not force_rebuild and os.path.exists(output_path):
            print(" SKIPPING COPY")
            return;
        
        print(f" PROCESSING")
        
        if os.path.exists(input_path):
            output_path_dir = os.path.dirname(output_path)
            if not os.path.exists(output_path_dir):
                print(f"{indent} CREATE {output_path_dir}")
                os.makedirs(output_path_dir, exist_ok=True)
            print(f"{indent} COPY {input_path} -> {output_path}")
            shutil.copy2(input_path, output_path)
        else:
            print(f"{indent} NOT FOUND {input_path}")
    
    if zoom < args.max_zoom:
        # print(output_dir)
        print(f"{parent_indent}[{zoom},{tile_x},{tile_y}]", end="")
        parent_output_path = os.path.join(output_dir, str(zoom), str(tile_y), f"{tile_x}.tile")
        print(parent_output_path)
        if not force_rebuild and os.path.exists(parent_output_path):
            print(" SKIPPING")
            return;
        
        print(f" PROCESSING")
    
        children = get_children(tile_x, tile_y)
        # Check if each child tile meshes exist, if not try to build them.
        children_to_merge = []
        for child in children:
            child_tile_x = child[0]
            child_tile_y = child[1]
            child_zoom = zoom + 1
            print(f"{indent}[{child_zoom},{child_tile_x},{child_tile_y}]", end="")
            child_output_path = os.path.join(output_dir, str(child_zoom), str(child_tile_y), f"{child_tile_x}.tile")
            if not os.path.exists(child_output_path):
                print(f" {child_output_path} doesn't exist, PROCESSING")
                process_tile(terrainmerger_path, child_tile_x, child_tile_y, child_zoom, output_dir, force_rebuild, args)
            
            # If the building of the child tile succeeded, add it to the list for merging
            if os.path.exists(child_output_path):
                children_to_merge.append(child_output_path)
        
        if len(children_to_merge) > 0:
            print(f"{parent_indent}[{zoom},{tile_x},{tile_y}] MERGING {children_to_merge}")
            build_command(terrainmerger_path, tile_x, tile_y, zoom, children_to_merge, output_dir, force_rebuild, args)
        else:
            print(f"{parent_indent}[{zoom},{tile_x},{tile_y}] NOTHING TO MERGE")

def main():
    parser = argparse.ArgumentParser(description="Create tile hierarchy by merging bottom up from the lowest level.")
    parser.add_argument("--terrainmerger", required=True, help="Path to the terrainmerger executable")
    parser.add_argument("--root-tile", nargs=3, type=int, required=True, help="EPSG 3857 tile Zoom, X and Y (e.g., 10 1234 5678)")
    parser.add_argument("--max-zoom", type=int, required=True, help="The maximum zoom level to search for tile meshes")
    parser.add_argument("--input-dir", required=True, help="Root input directory path")
    parser.add_argument("--output-dir", required=True, help="Root output directory path")
    parser.add_argument("--force-rebuild", action="store_true", help="Force rebuild/remerge existing tiles")
    parser.add_argument("command_args", nargs=argparse.REMAINDER, help="Command arguments for terrainmerger")

    args = parser.parse_args()

    terrainmerger_path = args.terrainmerger
    root_zoom, tile_x, tile_y = args.root_tile[0], args.root_tile[1], args.root_tile[2]
    
    process_tile(terrainmerger_path, tile_x, tile_y, root_zoom, args.output_dir, args.force_rebuild, args)

if __name__ == "__main__":
    main()