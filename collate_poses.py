"""
Collate refined TLS SE3 poses into a single text file.
"""

import argparse
import os
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(
        description="Collate aligned SE3 poses into one text file"
    )
    parser.add_argument(
        "tls_dir", type=str,
        help=(
            "Base directory containing project subfolders with 'trans' dirs, "
            "e.g., '/.../whu_tls_1030', which has "
            "'project1/trans/absolute_pgo', "
            "'project2/trans/absolute_pgo', and 'project2/trans/uniform_trans'."
        )
    )
    parser.add_argument(
        "output_txt", type=str,
        help="Output file path for collated poses"
    )
    return parser.parse_args()

def load_pose_lines(filename):
    """Read non-empty lines from a text file."""
    with open(filename, 'r') as f:
        return [line.strip() for line in f if line.strip()]

def line_to_pose(line):
    """Convert a space-separated line to a 3×4 pose matrix."""
    vals = list(map(float, line.split()))
    T = np.zeros((4, 4), dtype=float)
    T[:3, :4] = np.array(vals).reshape(3, 4)
    T[3, 3] = 1.0
    return T

if __name__ == '__main__':
    args = parse_args()

    # Define input folders
    p1_pgo = os.path.join(args.tls_dir, 'project1', 'trans', 'absolute_pgo')
    p2_pgo = os.path.join(args.tls_dir, 'project2', 'trans', 'absolute_pgo')
    p2_uni = os.path.join(args.tls_dir, 'project2', 'trans', 'uniform_trans')

    def sorted_txt_files(folder):
        files = [
            os.path.join(folder, f)
            for f in os.listdir(folder)
            if f.lower().endswith('.txt') and os.path.splitext(f)[0].isdigit()
        ]

        files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))
        return files

    files1 = sorted_txt_files(p1_pgo)
    files2 = sorted_txt_files(p2_pgo)
    files3 = sorted_txt_files(p2_uni)

    assert files1, f"No .txt files in {p1_pgo}"
    assert len(files2) == len(files3), "Mismatch between PGO and uniform transforms"

    count = 0
    with open(args.output_txt, 'w') as out:
        # Project1: append all pose lines
        for fn in files1:
            print(f"Collating poses from {fn}")
            for ln in load_pose_lines(fn):
                out.write(ln + '\n')
                count += 1

        # Project2: chain uniform and PGO
        for fn_pgo, fn_uni in zip(files2, files3):
            print(f"Collating poses from {fn_pgo}")
            pgo_line = load_pose_lines(fn_pgo)[0]
            uni_line = load_pose_lines(fn_uni)[0]
            T_pgo = line_to_pose(pgo_line)
            T_uni = line_to_pose(uni_line)
            T_combined = T_uni.dot(T_pgo)
            # Write 3×4 block row-wise with 12 decimal places
            for i in range(3):
                out.write(' '.join(f"{T_combined[i,j]:.10f}" for j in range(4)) + ' ')
            out.write('\n')
            count += 1

    print(f"Collated {count} poses into '{args.output_txt}'")
