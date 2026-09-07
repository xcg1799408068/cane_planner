#!/usr/bin/env python3
"""Generate local SVG evidence; no ROS or external publishing."""
import argparse
from pathlib import Path
import time
import numpy as np
from prototype import Grid, corridor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, help='Local SVG output path')
    args = parser.parse_args()
    scenarios = []
    for name, path, obstacle in [
            ('Open straight', [[1, 3], [11, 3]], False),
            ('90-degree corner', [[1, 1], [1, 5], [11, 5]], False),
            ('S-shaped original polyline', [[1, 1], [4, 1], [4, 5], [8, 5], [8, 1], [11, 1]], False),
            ('Whole-cell obstacle detour', [[1, 2], [4, 2], [4, 5], [8, 5], [11, 2]], True)]:
        g = Grid(np.zeros((24, 48), int), .25, np.zeros(2))
        if obstacle:
            g.values[6:17, 20:24] = 1
        t = time.monotonic()
        regions, overlaps = corridor(g, path)
        elapsed = time.monotonic()-t
        scenarios.append((name, g, path, regions, overlaps, elapsed))
        print(name, 'regions=', len(regions), 'seconds=', round(elapsed, 6),
              'min_overlap_area=', round(min((a for a, _ in overlaps), default=0), 4),
              'min_overlap_depth=', round(min((d for _, d in overlaps), default=0), 4))
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="720" viewBox="0 0 1100 720">',
           '<rect width="1100" height="720" fill="white"/>',
           '<text x="25" y="25" font-family="sans-serif" font-size="18">Isolated 2D FIRI adaptation — static geometry only</text>']
    colors = ['#0072B2', '#E69F00', '#009E73', '#CC79A7', '#56B4E9', '#D55E00']
    for index, (name, g, path, regions, overlaps, elapsed) in enumerate(scenarios):
        x0, y0 = 30+(index % 2)*550, 80+(index//2)*310
        def points(v):
            return ' '.join(f'{x0+40*x:.3f},{y0+240-40*y:.3f}' for x, y in v)
        svg.append(f'<text x="{x0}" y="{y0-12}" font-family="sans-serif" font-size="15">{name}: {len(regions)} regions</text>')
        svg.append(f'<rect x="{x0}" y="{y0}" width="480" height="240" fill="#fafafa" stroke="#999"/>')
        for r, color in zip(regions, (colors*20)):
            svg.append(f'<polygon points="{points(r.vertices)}" fill="{color}" fill-opacity="0.17" stroke="{color}" stroke-width="1"/>')
        for iy, ix in np.argwhere(g.values != 0):
            svg.append(f'<rect x="{x0+ix*10}" y="{y0+240-(iy+1)*10}" width="10" height="10" fill="#333"/>')
        svg.append(f'<polyline points="{points(path)}" fill="none" stroke="#111" stroke-width="2"/>')
        for r in regions:
            x, y = r.seed
            svg.append(f'<circle cx="{x0+40*x}" cy="{y0+240-40*y}" r="3" fill="white" stroke="#111"/>')
    svg.append('<text x="30" y="695" font-family="sans-serif" font-size="13">Black: original path / blocked cells. Circles: point seeds. Translucent polygons: certified static regions. No LFPC guarantee.</text></svg>')
    Path(args.output).write_text('\n'.join(svg)+'\n')


if __name__ == '__main__':
    main()
