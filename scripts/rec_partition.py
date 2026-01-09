import numpy as np

def rec_partition(grid: np.ndarray, thr: int) -> list:
    S = []  # 長方形の集合
    h, w = grid.shape  # h:高さ, w:幅

    visited = (grid >= thr)  # 無効マスは既に訪問済みにする

    while not all_visited(visited, h, w):
        progress = visited.sum() / (h * w)
        print(f"Progress: {progress:.2%}")

        max_rec = (0, 0, 0, 0)  # (y, x, hi, wi)

        for y in range(h):
            for x in range(w):
                # progress_in_loop = x * y / (h * w)
                # print(f"  process_in_loop: {progress_in_loop:.2%}") 
                max_height = h - y + 1
                max_width = w - x + 1
                if visited[y, x]:
                    continue
                for hi in range(1, max_height):
                    for wi in range(1, max_width):
                        area = hi * wi
                        if max_rec[2] * max_rec[3] >= area:
                            continue
                        rect_region = visited[y:y+hi, x:x+wi]
                        if np.any(rect_region):
                            break
                        
                        max_rec = (y, x, hi, wi)

        if max_rec[2] * max_rec[3] == 0:
            raise ValueError("最大長方形のサイズ０")

        # 長方形を追加し、訪問済みにする
        S.append(max_rec)
        print(f"  Found rectangle at (x={max_rec[1]}, y={max_rec[0]}) with size (w={max_rec[3]}, h={max_rec[2]})")
        y0, x0, hi, wi = max_rec
        for yy in range(y0, y0 + hi):
            for xx in range(x0, x0 + wi):
                visited[yy, xx] = True

    return S

def all_visited(visited: np.ndarray, h: int, w: int) -> bool:
    for y in range(h):
        for x in range(w):
            if not visited[y, x]:
                return False
    return True